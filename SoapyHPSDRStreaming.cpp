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
		packet[12] = 0x00;
		packet[524] = 0x00;
	}
	if (sample_rate > 48000.0 && sample_rate < 96001.0)
	{
		packet[12] = 0x01;
		packet[524] = 0x01;
	}
	if (sample_rate > 96000.0 && sample_rate < 192001.0)
	{
		packet[12] = 0x02;
		packet[524] = 0x02;
	}
	if (sample_rate > 192000.0)
	{
		packet[12] = 0x03;
		packet[524] = 0x03;
	}
	printf("SetSamplerate C0 %x C1 %x C2 %x C3 %x\n", packet[11], packet[12], packet[13], packet[14]);
	// 3. Send the 9-byte UDP packet
	ssize_t sent = send(data_socket, packet, sizeof(packet), 0);
	if (sent < 0)
	{
		perror("Error sending Metis command");
	}
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

	if (format == SOAPY_SDR_CF32) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
		ptr->set_stream_format(HPSDR_SDR_CF32);
		mox = true;
	}
	else if(format == SOAPY_SDR_CS16 && direction == SOAPY_SDR_TX)
	{
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
		ptr->set_stream_format(HPSDR_SDR_CS16);
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
	int iq = 0, ibuf = 8; 
	size_t ret = -1;
	int nr_samples = numElems / PACKETSIZE;
	int maxElements = 2 * PACKETSIZE;

	void const		*buff_base = buffs[0];
	float			*target_buffer = (float *) buff_base;
	sdr_stream *ptr = (sdr_stream *)stream;

	nr_samples = numElems;
	if (numElems > maxElements)
		nr_samples = maxElements;

	if (ptr->get_stream_format() == HPSDR_SDR_CF32)
	{
		for (int ii = 0; ii < nr_samples; ii++)
		{
			const float gain = 8388608.0f;
			int isample = target_buffer[iq] >= 0.0 ? (long)floor(target_buffer[iq] * gain + 0.5) : (long)ceil(target_buffer[iq] * gain - 0.5);
			int qsample = target_buffer[iq + 1] >= 0.0 ? (long)floor(target_buffer[iq + 1] * gain + 0.5) : (long)ceil(target_buffer[iq + 1] * gain - 0.5);
			qsample = qsample * -1;
			//printf("nr_samples %d sample: %d %d \n", iq, isample,qsample );

			tx_databuffer.at(ibuf++) = isample >> 16;
			tx_databuffer.at(ibuf++) = (isample & 0xFF00) >> 8;
			tx_databuffer.at(ibuf++) = isample & 0xFF;
			tx_databuffer.at(ibuf++) = qsample >> 16;
			tx_databuffer.at(ibuf++) = (qsample & 0xFF00) >> 8;
			tx_databuffer.at(ibuf++) = qsample & 0xFF;
			tx_databuffer.at(ibuf++) = 0; // mic sample
			tx_databuffer.at(ibuf++) = 0; // mic sample
			
			iq++;
			iq++;
		}
	}
	return nr_samples;
}