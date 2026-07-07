#include "SoapyHPSDR.hpp"
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <stdexcept>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <net/if_arp.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <semaphore.h>
#include <math.h>
#include <sys/select.h>
#include "ReceiveThread.hpp"

#define RADIOBERRY_BUFFER_SIZE	4096	

/***********************************************************************
 * Device interface
 **********************************************************************/

SoapyHPSDR::SoapyHPSDR(const SoapySDR::Kwargs &args)
	: _addr("192.168.88.24"), _port(1024)
	{
	char message[180];
		
	SoapySDR_setLogLevel(SOAPY_SDR_INFO);
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::SoapyHPSDR  constructor called");
	mox = false;
	no_channels = 1;
	data_socket = -1;
	sequence = 0;
	send_sequence = 0;
	num_hpsdr_receivers = 1;
	rx_frequency = 7074000;
	tx_databuffer.resize(PACKETSIZE);

	if (args.count("addr"))
		_addr = args.at("addr");
	if (args.count("port"))
		std::stringstream(args.at("port")) >> _port;

	data_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (data_socket < 0)
	{
		SoapySDR_log(SOAPY_SDR_ERROR, "SoapyHPSDR::SoapyHPSDR create socket failed for data_socket");
	}
	
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 100000;
	if (setsockopt(data_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
	{
		SoapySDR_log(SOAPY_SDR_ERROR, "SoapyHPSDR::SoapyHPSDR Error setting timeout");
		close(data_socket);
		return;
	}

	int optval = 1;
	setsockopt(data_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
	setsockopt(data_socket, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
	optval = 0xffff;
	if (setsockopt(data_socket, SOL_SOCKET, SO_SNDBUF, &optval, sizeof(optval)) < 0)
	{
		perror("data_socket: SO_SNDBUF");
	}
	if (setsockopt(data_socket, SOL_SOCKET, SO_RCVBUF, &optval, sizeof(optval)) < 0)
	{
		perror("data_socket: SO_RCVBUF");
	}

	struct sockaddr_in serv_addr;
	memset((char *)&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(_port); // Convert port to network byte order

	if (inet_pton(AF_INET, _addr.c_str(), &serv_addr.sin_addr) <= 0)
	{
		SoapySDR_log(SOAPY_SDR_ERROR, "SoapyHPSDR::SoapyHPSDR Invalid address");
		close(data_socket);
		return;
	}
	
	// "Connect" the Datagram socket
	// NOTE: This does NOT send a handshake. It just binds the local socket
	// to the remote destination in the kernel's routing table.
	if (connect(data_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Error connecting datagram socket");
		close(data_socket);
		exit(EXIT_FAILURE);
	}
	sprintf(message, "Socket 'connected' to %s:%d (Kernel routing established).\n", _addr.c_str(), _port);
	SoapySDR_log(SOAPY_SDR_INFO, message);

	ReceiveThread::create_receive_thread(data_socket, this);

	// Discovery
	char payload[60]{0};
	payload[0] = 0xEF;
	payload[1] = 0xFE;
	payload[2] = 0x02;
	payload[3] = 0x00;

	if (send(data_socket, payload, 60, 0) < 0)
	{
		SoapySDR_log(SOAPY_SDR_ERROR, "SoapyHPSDR::SoapyHPSDR Error sending data");
		close(data_socket);
		return;
	}
	SoapySDR_log(SOAPY_SDR_INFO, "Send discovery");
	for(int i = 0; i < 1000; i++)
	{
		usleep(1000);
	}
	/*
	setSampleRate(SOAPY_SDR_RX, 0, 48000);
	setFrequency(SOAPY_SDR_RX, 0, 7074000);
	SoapySDR::Stream *stream = setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
	for (int i = 0; i < 1000; i++)
	{
		usleep(1000);
	}
	closeStream(stream);
*/
}

SoapyHPSDR::~SoapyHPSDR(void)
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::SoapyHPSDR  destructor called");
	for (auto con : streams)
		delete (con);
	if (data_socket != 0)
		close(data_socket);
	//printf("databuffer size %d\n", databuffer.size());
}

void SoapyHPSDR::controlHPSDR(uint32_t command, uint32_t command_data) {

	std::unique_lock<std::mutex> soapy_lock(send_command);
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::controlHPSDR called");

	uint8_t packet[PACKETSIZE];
	
	memset(packet, 0, sizeof(packet));
	// 1. Header 0x0201feef
	packet[0] = 0xEF; // Sync 1
	packet[1] = 0xFE; // Sync 2
	packet[2] = 0x01; // Command Type: Control
	packet[3] = 0x02; // 

	packet[4] = 0x01; // 
	packet[5] = 0x00; // EP

	uint32_t net_sequence = htonl(0);
	memcpy(&packet[6], &net_sequence, 4);
	
	// 2. Payload (C0 to C4)
	packet[11] = command; // Sub-command (0x00 for Freq, 0x01 for Rate)
	packet[523] = command; // Sub-command (0x00 for Freq, 0x01 for Rate)
	// Convert the 32-bit parameter to Network Byte Order (Big Endian)
	uint32_t net_param = htonl(command_data);
	memcpy(&packet[12], &net_param, 4);
	memcpy(&packet[524], &net_param, 4);
	printf("C0 %x C1 %x C2 %x C3 %x\n", packet[12], packet[13], packet[14], packet[15]);

	// 3. Send the 9-byte UDP packet
	ssize_t sent = send(data_socket, packet, sizeof(packet), 0);
	if (sent < 0)
	{
		perror("Error sending Metis command");
	}
	else
	{
		printf("Sent 9-byte Metis packet (C0=0x%02X, Param=0x%08X)\n", command, command_data);
	}
}

void SoapyHPSDR::startDataStream(void)
{ 
	// start receiver
	char payload[64]{0};
	payload[0] = 0xEF;
	payload[1] = 0xFE;
	payload[2] = 0x04;
	payload[3] = 0x01;

	if (send(data_socket, payload, 64, 0) < 0)
	{
		SoapySDR_log(SOAPY_SDR_ERROR, "SoapyHPSDR::SoapyHPSDR Error sending data");
	}
}

void SoapyHPSDR::stopDataStream(void)
{
	// start receiver
	char payload[64]{0};
	payload[0] = 0xEF;
	payload[1] = 0xFE;
	payload[2] = 0x04;
	payload[3] = 0x00;

	if (send(data_socket, payload, 64, 0) < 0)
	{
		SoapySDR_log(SOAPY_SDR_ERROR, "SoapyHPSDR::SoapyHPSDR Error sending data");
	}
}

std::string SoapyHPSDR::getDriverKey( void ) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getDriverKey called");
	
	return "hpsdr";
}

std::string SoapyHPSDR::getHardwareKey( void ) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getHardwareKey called");

	return "hpsdr v0.1";
}

SoapySDR::Kwargs SoapyHPSDR::getHardwareInfo( void ) const
{
	
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getHardwareInfo called");

	SoapySDR::Kwargs info{};

	return info;
}

size_t SoapyHPSDR::getNumChannels( const int direction ) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getNumChannels called");
	
	//if (direction == SOAPY_SDR_RX) return(4);
	
	return(1); //1 RX and 1 TX channel; making this for standalone radioberry!
}

bool SoapyHPSDR::getFullDuplex( const int direction, const size_t channel ) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getFullDuplex called");
	
	return(true);
}

std::vector<double> SoapyHPSDR::listBandwidths( const int direction, const size_t channel ) const
{
	// radioberry does nor support bandwidth
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::listBandwidths called");
		
	std::vector<double> options;
	return(options);
}

std::vector<double> SoapyHPSDR::listSampleRates( const int direction, const size_t channel ) const
{
	
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::listSampleRates called");
	
	
	std::vector<double> options;
	
	if (direction == SOAPY_SDR_RX) {
		options.push_back(0.048e6);  
		options.push_back(0.096e6);
		options.push_back(0.192e6);
		options.push_back(0.384e6);
		//options.push_back(0.768e6);
		//options.push_back(1.536e6);
	}
	if (direction == SOAPY_SDR_TX) {
		options.push_back(0.048e6);
	}
	return(options);
}

double SoapyHPSDR::getBandwidth( const int direction, const size_t channel ) const
{
	
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getBandwidth called");
	
    long long bandwidth = 48000.0;

	if(direction==SOAPY_SDR_RX){
      
	  //depends on settings.. TODO

	}

	else if(direction==SOAPY_SDR_TX){
       bandwidth = 48000.0;
	}

	return double(bandwidth);
}

SoapySDR::RangeList SoapyHPSDR::getFrequencyRange( const int direction, const size_t channel)  const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getFrequencyRange called");
	
	SoapySDR::RangeList rangeList;
	
	rangeList.push_back(SoapySDR::Range(10000.0, 30000000.0, 1.0));
	
	return rangeList;
}

std::vector<std::string> SoapyHPSDR::listAntennas( const int direction, const size_t channel ) const
{
	
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::listAntennas called");
	
	std::vector<std::string> options;
	if(direction == SOAPY_SDR_RX) options.push_back( "ANTENNA RX" );
	if(direction == SOAPY_SDR_TX) options.push_back( "ANTENNA TX" );
	return(options);
}


/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyHPSDR::listGains( const int direction, const size_t channel ) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::listGains called");
	
	std::vector<std::string> options;
	//options.push_back("PGA"); //in pihpsdr no additional gain settings.
	return(options);
}

SoapySDR::Range SoapyHPSDR::getGainRange( const int direction, const size_t channel) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getGainRange called");
	
	if(direction==SOAPY_SDR_RX)
		return(SoapySDR::Range(-12, 48));
	return(SoapySDR::Range(0,15));
}

void SoapyHPSDR::setGain( const int direction, const size_t channel, const double value ) {
	
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::setGain called");
	
	uint32_t command = 0;
	uint32_t command_data = (0x40 | (((uint32_t)value)  & 0x3F));
	
	if (direction == SOAPY_SDR_RX)	 
	{
		if (mox)
			command = 0x15;
		else
			command = 0x14;//14
		command_data = (0x40 | (((uint32_t)value + 12) & 0x3F));
	}
	
	if(direction==SOAPY_SDR_TX) 
	{ // 0 -7 TX RF gain
		drive = value;
		command_data = (uint32_t)value;
		if (value > 15)
			command_data = 15;
		if (value < 0.0)
			command_data = 0;
		command_data = command_data << 28;
		command = 0x13;
		this->SoapyHPSDR::controlHPSDR(command, command_data);
	}
	
}

/*******************************************************************
 * Frequency API 
 ******************************************************************/
void SoapyHPSDR::setFrequency( const int direction, const size_t channel,  const double frequency, const SoapySDR::Kwargs &args ) {
	
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::setFrequency called");
	
	uint32_t command = 0;

	if (direction == SOAPY_SDR_RX)
	{
		if (mox)
			command = 5;
		else
			command = 4;
		rx_frequency = frequency;
	}

	if (direction == SOAPY_SDR_TX)
	{
		if (!mox)
			return;
		command = 3;
		tx_frequency = frequency;
	}

	uint32_t command_data = frequency;

	//if (!mox)
		this->SoapyHPSDR::controlHPSDR(command, command_data);
}

void SoapyHPSDR::writeI2C(const int addr, const std::string &data)
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::writeI2C called");

	if (!i2c_available)
		return;
/*	i2c_ptr->addr(addr);
	try
	{
		i2c_ptr->write((uint8_t *)data.c_str(), data.size());
	}
	catch (const std::exception &e)
	{
		SoapySDR_log(SOAPY_SDR_WARNING, e.what());
	}
*/
}

std::string SoapyHPSDR::readI2C(const int addr, const size_t numBytes)
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::readI2C called");

	std::string data;

	if (!i2c_available)
		return std::string("");
/*	i2c_ptr->addr(addr);
	data.reserve(numBytes);
	try
	{
		i2c_ptr->read((uint8_t *)data.c_str(), numBytes);
		data.resize(numBytes);
	}
	catch (std::string s)
	{
		printf("%s", s.c_str());
	}
*/
	return data;
}

SoapySDR::ArgInfoList SoapyHPSDR::getSettingInfo() const
{
	SoapySDR::ArgInfoList args;
	SoapySDR::ArgInfo arg;

	arg.key = "PowerAmp";
	arg.name = "Power Amplifier";
	arg.type = SoapySDR::ArgInfo::STRING;
	arg.options = {"Operate", "Standby"};
	arg.optionNames = {"Amplifier Status"};

	args.push_back(arg);
	return args;
}

SoapySDR::ArgInfoList SoapyHPSDR::getSettingInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList args;
	SoapySDR::ArgInfo arg;

	return args;
}

void SoapyHPSDR::writeSetting(const std::string &key, const std::string &value)
{
}

std::string SoapyHPSDR::readSetting(const std::string &key) const
{
	return "";
}

// end of source.

