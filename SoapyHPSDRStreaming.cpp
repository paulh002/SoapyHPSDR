#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <cmath>
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
#include "SoapyHPSDR.hpp"
#include "ReceiveThread.hpp"

#define HEADER1 0
#define HEADER2 1
#define HEADER3 2
#define EP 3

#define SEQ1 4
#define SEQ2 5
#define SEQ3 5
#define SEQ4 6

#define SYNC1 7
#define SYNC2 8

#define C0 11
#define C1 12
#define C2 13
#define C3 14
#define C4 15


void SoapyHPSDR::setSampleRate( const int direction, const size_t channel, const double rate ) {
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::setSampleRate called");

	uint8_t packet[PACKETSIZE];
	int irate = floor(rate);
	uint32_t	ucom =0x4;
	uint32_t	command = 0;

	sample_rate = rate;

	if (direction == SOAPY_SDR_TX)
	{
		command = 0x00;
	}
	if (direction == SOAPY_SDR_RX)
	{
		command = 0x01;
	}

	// incase of transmit still the receive samplerate need to be send
	memset(packet, 0, sizeof(packet));
	// 1. Header 0x0201feef
	packet[0] = 0xEF; // Sync 1
	packet[1] = 0xFE; // Sync 2
	packet[2] = 0x01; // Command Type: Control
	packet[3] = 0x02; //

	packet[4] = 0x00; //
	packet[5] = 0x00; // EP

	uint32_t net_sequence = htonl(0);
	memcpy(&packet[6], &net_sequence, 4);

	// 2. Payload (C0 to C4)
	packet[11] = command; // Sub-command (0x00 for Freq, 0x01 for Rate)
	packet[523] = command; // Sub-command (0x00 for Freq, 0x01 for Rate)
	if (sample_rate < 48001.0)
	{
		packet[12] = EncodeSampleRate();
		packet[524] = EncodeSampleRate();
	}
	if (sample_rate > 48000.0 && sample_rate < 96001.0)
	{
		packet[12] = EncodeSampleRate();
		packet[524] = EncodeSampleRate();
	}
	if (sample_rate > 96000.0 && sample_rate < 192001.0)
	{
		packet[12] = EncodeSampleRate();
		packet[524] = EncodeSampleRate();
	}
	if (sample_rate > 192000.0)
	{
		packet[12] = EncodeSampleRate();
		packet[524] = EncodeSampleRate();
	}
	printf("SetSamplerate C0 %x C1 %x C2 %x C3 %x\n", packet[11], packet[12], packet[13], packet[14]);
	// 3. Send the 9-byte UDP packet
	ssize_t sent = send(data_socket, packet, sizeof(packet), 0);
	if (sent < 0)
	{
		perror("Error sending Metis command");
	}
}

char SoapyHPSDR::EncodeSampleRate()
{
	char c = 0x00;
	if (sample_rate < 48001.0)
	{
		c= 0x00;
	}
	if (sample_rate > 48000.0 && sample_rate < 96001.0)
	{
		c = 0x00;;
	}
	if (sample_rate > 96000.0 && sample_rate < 192001.0)
	{
		c = 0x02;
	}
	if (sample_rate > 192000.0)
	{
		c = 0x03;
	}
	return c;
}

SoapySDR::RangeList SoapyHPSDR::getSampleRateRange(const int direction, const size_t channel) const 
{
	
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getSampleRateRange called");
	
	SoapySDR::RangeList rangeList;
	
	if (direction == SOAPY_SDR_RX) 	rangeList.push_back(SoapySDR::Range(48000.0, 384000.0, 48000.0));
	if (direction == SOAPY_SDR_TX) 	rangeList.push_back(SoapySDR::Range(48000.0, 48000.0, 48000.0));
	
	return rangeList;
}

std::vector<std::string> SoapyHPSDR::getStreamFormats(const int direction, const size_t channel) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getStreamFormats called");
	
	std::vector<std::string> formats;

	formats.push_back(SOAPY_SDR_CF32);
	formats.push_back(SOAPY_SDR_CS16);

	return formats;
}

std::string SoapyHPSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getNativeStreamFormat called");
	
	if (direction == SOAPY_SDR_RX) {
		fullScale = 2048; // RX expects 12 bit samples LSB aligned
	}
	else if (direction == SOAPY_SDR_TX) {
		fullScale = 32768; // TX expects 12 bit samples MSB aligned
	}
	return SOAPY_SDR_CF32;
}

SoapySDR::ArgInfoList SoapyHPSDR::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;
	
	SoapySDR::ArgInfo bufflenArg;
    bufflenArg.key = "bufflen";
    bufflenArg.value = "64"; 
    bufflenArg.name = "Buffer Size";
    bufflenArg.description = "Number of bytes per buffer, multiples of 512 only.";
    bufflenArg.units = "bytes";
    bufflenArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(bufflenArg);

	return streamArgs;
}

auto startTime = std::chrono::high_resolution_clock::now();


SoapySDR::Stream *SoapyHPSDR::setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels,
		const SoapySDR::Kwargs &args )
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::setupStream called");
	startTime = std::chrono::high_resolution_clock::now();
	//check the format
	sdr_stream *ptr;
	ptr = new sdr_stream(direction);
	if (direction == SOAPY_SDR_TX)
	{
		ibuf = 16; // skip header
		send_sequence = 0;
	}
	if (format == SOAPY_SDR_CF32) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
		ptr->set_stream_format(HPSDR_SDR_CF32);
		mox = true;
	}
	else
	{
		throw std::runtime_error(
			"setupStream invalid format '" + format + "' -- Only CF32 is supported by SoapyHPSDRSDR module.");
	}

	streams.push_back(ptr);
	startDataStream();
	SoapySDR_log(SOAPY_SDR_INFO, "Send receiver");
	ptr_rx_thread->SetStreamActive(true);
	return (SoapySDR::Stream *)ptr;
}

void SoapyHPSDR::closeStream(SoapySDR::Stream *stream)
{
	int i = 0;
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::closeStream");
	for (auto con : streams)
	{
		if ((sdr_stream *)stream == con)
		{
			if (((sdr_stream *)stream)->get_direction() == SOAPY_SDR_TX)
			{	// switch off TX stream
				mox = false;
				setSampleRate(SOAPY_SDR_RX, 0, sample_rate);	
			}
			delete ((sdr_stream *)stream);
			streams.erase(streams.begin() + i);
		}
		i++;
	}
	if (streams.size() == 0)
		stopDataStream();
}

void SoapyHPSDR::push_iqsamples(std::vector<std::complex<float>> iqsamples)
{
	rx_databuffer.push(std::move(iqsamples));
}

int SoapyHPSDR::readStream(
		SoapySDR::Stream *handle,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs )
{
	int npackages = numElems / 63;
	int iq = 0;
	void *buff_base = buffs[0];
	float	*target_buffer = (float *) buff_base;
	sdr_stream *ptr = (sdr_stream *)handle;

	for(int ii = 0 ; ii < npackages ; ii++)
	{
		if (ptr->get_stream_format() == HPSDR_SDR_CF32)
		{
			std::vector<std::complex<float>> iqsamples = rx_databuffer.pull();
			if (iqsamples.empty())
			{
				return (ii * 63);
			}

			for (auto con : iqsamples) {
				target_buffer[iq++] = con.real();      // 24 bit sample
				target_buffer[iq++] = con.imag();      // 24 bit sample
			}
		}
		//printf("nr_samples %d sample: %f %f \n", nr_samples, con.real(), con.imag());
	}
	return (npackages * 63); //return the number of IQ samples
}


int SoapyHPSDR::writeStream(SoapySDR::Stream *stream, const void * const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs)
{
	int iq = 0; 
	size_t ret = -1;
	int nr_samples = 126;

	void const		*buff_base = buffs[0];
	float			*target_buffer = (float *) buff_base;
	sdr_stream *ptr = (sdr_stream *)stream;

	// LEFT_SAMPLE_HI, LEFT_SAMPLE_MID, LEFT_SAMPLE_LOW, RIGHT_SAMPLE_HI, RIGHT_SAMPLE_MID, RIGHT_SAMPLE_LOW, MIC_SAMPLE_HI, MIC_SAMPLE_LOW
	if (ptr->get_stream_format() == HPSDR_SDR_CF32)
	{
		for (int ii = 0; ii < nr_samples; ii++)
		{
			const float gain = 32767.0; // 8388608.0f;
			int isample = target_buffer[iq] >= 0.0 ? (long)floor(target_buffer[iq] * gain + 0.5) : (long)ceil(target_buffer[iq] * gain - 0.5);
			int qsample = target_buffer[iq + 1] >= 0.0 ? (long)floor(target_buffer[iq + 1] * gain + 0.5) : (long)ceil(target_buffer[iq + 1] * gain - 0.5);
			qsample = qsample * -1;
			//printf("nr_samples %d sample: %d %d \n", iq, isample,qsample );

			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = isample >> 8;
			tx_databuffer.at(ibuf++) = isample & 0xFF;
			tx_databuffer.at(ibuf++) = qsample >> 8;
			tx_databuffer.at(ibuf++) = qsample & 0xFF;
			if (ibuf == 522)
				ibuf += 6;
			if (ibuf == PACKETSIZE)
			{
				ibuf = 16;
				transmit_buffer();
			}
			iq++;
			iq++;
		}
	}
	return nr_samples;
}

int SoapyHPSDR::transmit_buffer()
{
	tx_databuffer.at(HEADER1) = 0xEF; // Sync 1
	tx_databuffer.at(HEADER2) = 0xFE; // Sync 2
	tx_databuffer.at(HEADER3) = 0x01; // Command Type: Control
	tx_databuffer.at(EP) = 0x02; // Command Type: Control
	tx_databuffer.at(SEQ1) = 0x00; // Command Type: Control
	tx_databuffer.at(SEQ2) = 0x00; // Command Type: Control

	uint32_t net_sequence = htonl(send_sequence);
	std::memcpy(tx_databuffer.data() + SEQ3, &net_sequence, sizeof(net_sequence));
	send_sequence++;
	
	tx_databuffer.at(C0) = 0x01; // Mox
	tx_databuffer.at(C1) = EncodeSampleRate(); //
	tx_databuffer.at(C2) = 0x00; // 
	tx_databuffer.at(C3) = 0x00; // 
	tx_databuffer.at(C4) = 0x04; // duplex
	tx_databuffer.at(C4) = tx_databuffer.at(C4) | (num_hpsdr_receivers - 1) << 3;

	//tx_databuffer.at(HEADER1 + 528) = 0xEF; // Sync 1
	//tx_databuffer.at(HEADER2 + 528) = 0xFE; // Sync 2
	//tx_databuffer.at(HEADER3 + 528) = 0x01; // Command Type: Control
	//tx_databuffer.at(SEQ1 + 528) = 0x00;		  // Sequence
	//tx_databuffer.at(SEQ2 + 528) = 0x00;		  //
	//tx_databuffer.at(SEQ3 + 528) = 0x00;		  // Sequence
	//tx_databuffer.at(SEQ4 + 528) = 0x00;		  //

	tx_databuffer.at(523) = 0x01; // Mox
	tx_databuffer.at(524) = EncodeSampleRate();
	tx_databuffer.at(525) = 0x00;
	tx_databuffer.at(526) = 0x00;
	tx_databuffer.at(527) = 0x04;
	//uint32_t net_param = htonl(tx_frequency);
	//std::memcpy(tx_databuffer.data() + 540, &net_param, sizeof(net_param));

	//tx_databuffer.at(C0 + 528) = 0x03; // Command TX frequency
	//tx_databuffer.at(C4 + 512) |= 0x04; // duplex
	errno = 0; // Clear it right before the call!
	ssize_t sent = send(data_socket, tx_databuffer.data(), PACKETSIZE, 0);
	if (sent < 0 || sent != PACKETSIZE)
	{
		char str[128];
		sprintf(str, "SoapyHPSDR Error sending data: %s (errno: %d)",
				strerror(errno), errno);
		SoapySDR_log(SOAPY_SDR_ERROR, str);
	}
	return 0;
}
