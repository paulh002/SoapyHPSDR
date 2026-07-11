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
#define SEQ3 6
#define SEQ4 7

#define SYNC1 8
#define SYNC2 9
#define SYNC3 10
#define C0 11
#define C1 12
#define C2 13
#define C3 14
#define C4 15

void SoapyHPSDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::setSampleRate called");

	std::vector<char> packet;
	uint8_t command = 0;

	packet.resize(PACKETSIZE, 0);
	if (direction == SOAPY_SDR_TX)
	{
		command = 0x01;
		tx_samplerate = rate;
	}
	if (direction == SOAPY_SDR_RX)
	{
		command = 0x00;
		rx_samplerate = rate;
	}
	transmit_buffer(packet, command, rate);
}

char SoapyHPSDR::EncodeSampleRate(double sample_rate)
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
	return formats;
}

std::string SoapyHPSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::getNativeStreamFormat called");
	
	if (direction == SOAPY_SDR_RX) {
		fullScale = 32768; // RX expects 16 bit samples LSB aligned
	}
	else if (direction == SOAPY_SDR_TX) {
		fullScale = 32768; // TX expects 16 bit samples MSB aligned
	}
	return SOAPY_SDR_CF32;
}

SoapySDR::ArgInfoList SoapyHPSDR::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;
	
	SoapySDR::ArgInfo bufflenArg;
    bufflenArg.key = "bufflen";
    bufflenArg.value = "2016"; 
    bufflenArg.name = "Buffer Size";
	bufflenArg.description = "Number of Elements per buffer";
    bufflenArg.units = "Elements";
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
	if (direction == SOAPY_SDR_RX)
	{
		startDataStream();
		SoapySDR_log(SOAPY_SDR_INFO, "Send receiver");
	}
	ptr_receive_thread->SetStreamActive(true);
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
				setSampleRate(SOAPY_SDR_RX, 0, rx_samplerate);	
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

auto timeLastMeasure = std::chrono::high_resolution_clock::now();

int SoapyHPSDR::writeStream(SoapySDR::Stream *stream, const void * const *buffs, const size_t numElems, int &flags, const long long timeNs, const long timeoutUs)
{
	int iq = 0; 
	int nr_samples = numElems;

	void const		*buff_base = buffs[0];
	float			*target_buffer = (float *) buff_base;
	sdr_stream *ptr = (sdr_stream *)stream;
	
	//const auto now = std::chrono::high_resolution_clock::now();
	//const auto timePassed1 = std::chrono::duration_cast<std::chrono::microseconds>(now - timeLastMeasure);
	//timeLastMeasure = now;
	//printf("Time measure %ld us send %ld\n", timePassed1.count(), numElems);
	
	// LEFT_SAMPLE_HI, LEFT_SAMPLE_MID, LEFT_SAMPLE_LOW, RIGHT_SAMPLE_HI, RIGHT_SAMPLE_MID, RIGHT_SAMPLE_LOW, MIC_SAMPLE_HI, MIC_SAMPLE_LOW
	if (ptr->get_stream_format() == HPSDR_SDR_CF32)
	{
		for (int ii = 0; ii < nr_samples; ii++)
		{
			const float gain = 32767.0; // 8388608.0f;
			int isample = target_buffer[iq] >= 0.0 ? (long)floor(target_buffer[iq] * gain + 0.5) : (long)ceil(target_buffer[iq] * gain - 0.5);
			int qsample = target_buffer[iq + 1] >= 0.0 ? (long)floor(target_buffer[iq + 1] * gain + 0.5) : (long)ceil(target_buffer[iq + 1] * gain - 0.5);
			qsample = qsample * -1;
			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = 0; // tone sample
			tx_databuffer.at(ibuf++) = isample >> 8;
			tx_databuffer.at(ibuf++) = isample & 0xFF;
			tx_databuffer.at(ibuf++) = qsample >> 8;
			tx_databuffer.at(ibuf++) = qsample & 0xFF;
			if (ibuf == 520)
				ibuf += 8;
			if (ibuf == PACKETSIZE)
			{
				ibuf = 16;
				transmit_buffer(tx_databuffer, 0x01, tx_samplerate);	
			}
			iq++;
			iq++;
		}
	}
	return nr_samples;
}


int SoapyHPSDR::transmit_buffer(std::span<char> databuffer, char command, double samplerate)
{
	databuffer[HEADER1] = 0xEF; // Sync 1
	databuffer[HEADER2] = 0xFE; // Sync 2
	databuffer[HEADER3] = 0x01; // Command Type: Control
	databuffer[EP] = 0x02; // Command Type: Control
	send_sequence++;
	databuffer[4] = (send_sequence >> 24) & 0xFF;
	databuffer[5] = (send_sequence >> 16) & 0xFF;
	databuffer[6] = (send_sequence >> 8) & 0xFF;
	databuffer[7] = (send_sequence) & 0xFF;
	
	databuffer[SYNC1] = 0x7F; // 
	databuffer[SYNC2] = 0x7F; // 
	databuffer[SYNC2] = 0x7F; // 
	
	//uint32_t net_sequence = htonl(send_sequence);
	//std::memcpy(databuffer.data() + SEQ3, &net_sequence, sizeof(net_sequence));

	databuffer[C0] = command;
	databuffer[C1] = EncodeSampleRate(samplerate); //
	databuffer[C2] = 0x00; // 
	databuffer[C3] = 0x00; // 
	databuffer[C4] = 0x04; // duplex
	databuffer[C4] = 0x00;//databuffer[C4] | (num_hpsdr_receivers - 1) << 3;

	databuffer[SYNC1 + 512] = 0x7F; // 
	databuffer[SYNC2 + 512] = 0x7F; // 
	databuffer[SYNC2 + 512] = 0x7F; // 
	databuffer[C0 + 512] = command;
	databuffer[C1 + 512] = EncodeSampleRate(samplerate);
	databuffer[C2 + 512] = 0x00;
	databuffer[C3 + 512] = 0x00;
	databuffer[C4 + 512] = 0x04;

	//const auto now1 = std::chrono::high_resolution_clock::now();
	errno = 0; // Clear it right before the call!
	ssize_t sent = send(data_socket, databuffer.data(), PACKETSIZE, 0);
	if (sent < 0 || sent != PACKETSIZE)
	{
		
		char str[128];
		sprintf(str, "SoapyHPSDR Error sending data: %s (errno: %d)",
				strerror(errno), errno);
		SoapySDR_log(SOAPY_SDR_ERROR, str);
	}

	return 0;
}
