#pragma once
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include "DataBuffer.h"


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <memory>
#include <string.h>
#include <mutex>
#include <cstring>
#include <atomic>
#include <complex>
#include <array>

#define TX_MAX 4800
#define TX_MAX_BUFFER (TX_MAX * 8)
#define PACKETSIZE 1032

extern const int packetsize;

typedef enum HpsdrStreamFormat
{
	HPSDR_SDR_CF32,
	HPSDR_SDR_CS16
} hpsdrstreamFormat;

class sdr_stream
{
  public:
	sdr_stream(int dir)
	{
		direction = dir;
	}
	int get_direction() { return direction; }
	void set_stream_format(hpsdrstreamFormat sf) { streamFormat = sf; };
	hpsdrstreamFormat get_stream_format() { return streamFormat; };

  private:
	int direction;
	hpsdrstreamFormat streamFormat;
};

class SoapyHPSDR : public SoapySDR::Device
{
  public:
	SoapyHPSDR(const SoapySDR::Kwargs &args);
	~SoapyHPSDR();

	/*******************************************************************
	 * Identification API
	 ******************************************************************/

	std::string getDriverKey(void) const;

	std::string getHardwareKey(void) const;

	SoapySDR::Kwargs getHardwareInfo(void) const;

	/*******************************************************************
	 * Channels API
	 ******************************************************************/

	size_t getNumChannels(const int direction) const;

	bool getFullDuplex(const int direction, const size_t channel) const;

	/*******************************************************************
	 * Stream API
	 ******************************************************************/
	SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

	std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

	SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

	void closeStream(SoapySDR::Stream *stream);

	SoapySDR::Stream *setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

	int readStream(
		SoapySDR::Stream *stream,
		void *const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs = 100000);

	int writeStream(
		SoapySDR::Stream *stream,
		const void *const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs = 0,
		const long timeoutUs = 100000);

	/*******************************************************************
	 * Sample Rate API
	 ******************************************************************/

	void setSampleRate(const int direction, const size_t channel, const double rate);

	double getBandwidth(const int direction, const size_t channel) const;

	std::vector<double> listBandwidths(const int direction, const size_t channel) const;
	std::vector<double> listSampleRates(const int direction, const size_t channel) const;

	/*******************************************************************
	 * Frequency API
	 ******************************************************************/

	void setFrequency(
		const int direction,
		const size_t channel,
		const double frequency,
		const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

	SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel) const;

	/*******************************************************************
	 * Antenna API
	 ******************************************************************/

	std::vector<std::string> listAntennas(const int direction, const size_t channel) const;

	/*******************************************************************
	 * Gain API
	 ******************************************************************/

	std::vector<std::string> listGains(const int direction, const size_t channel) const;

	void setGain(const int direction, const size_t channel, const double value);

	SoapySDR::Range getGainRange(const int direction, const size_t channel) const;

	void controlHPSDR(uint32_t command, uint32_t command_data);

	/*******************************************************************
	 * I2C API
	 ******************************************************************/

	std::string readI2C(const int addr, const size_t numBytes);
	void writeI2C(const int addr, const std::string &data);

	/*******************************************************************
	 * Settings API
	 ******************************************************************/

	SoapySDR::ArgInfoList getSettingInfo() const;
	SoapySDR::ArgInfoList getSettingInfo(const int direction, const size_t channel) const;
	void writeSetting(const std::string &key, const std::string &value);
	std::string readSetting(const std::string &key) const;
	bool IsStreamActive();
	void push_iqsamples(std::vector<std::complex<float>> iqsamples);

  private:
	std::string _addr;
	unsigned short _port;
	int data_socket;
	int tcp_socket;
	double rx_samplerate;
	double tx_samplerate;
	uint32_t rx_frequency;
	uint32_t tx_frequency;
	int no_channels;
	std::mutex send_command;
	std::vector<sdr_stream *> streams;
	bool i2c_available;
	bool mox;
	bool poweramp_operational;
	uint32_t drive;
	std::atomic<uint32_t> sequence;
	DataBuffer<std::complex<float>> rx_databuffer;
	std::vector<char> tx_databuffer;
	uint32_t send_sequence;
	uint32_t num_hpsdr_receivers;
	int ibuf;
	

	void startDataStream(void);
	void stopDataStream(void);
	int transmit_buffer();
	char EncodeSampleRate(double sample_rate);
};
