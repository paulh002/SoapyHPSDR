#include <semaphore.h>
#include <thread>
#include <complex>
#include <atomic>
#include <memory>
#include <functional>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include "SoapyHPSDR.hpp"

typedef struct
{
	uint8_t mac[6];
	uint8_t gateware_version;
} HPSDR_Device_Info;

#define SYNC 0x7F
enum
{
	SYNC_0 = 0,
	SYNC_1,
	SYNC_2,
	CONTROL_0,
	CONTROL_1,
	CONTROL_2,
	CONTROL_3,
	CONTROL_4,
	LEFT_SAMPLE_HI,
	LEFT_SAMPLE_MID,
	LEFT_SAMPLE_LOW,
	RIGHT_SAMPLE_HI,
	RIGHT_SAMPLE_MID,
	RIGHT_SAMPLE_LOW,
	MIC_SAMPLE_HI,
	MIC_SAMPLE_LOW,
	SKIP
};

class ReceiveThread
{
  public:
	ReceiveThread(int receive_socket, SoapyHPSDR *_device);
	static bool create_receive_thread(int receive_socket, SoapyHPSDR *device);
	static void destroy_receive_thread();
	void operator()();
	std::atomic_bool stop_flag{false};
	void SetStreamActive(bool active);

  private:
	int _receive_socket;
	bool metis;
	int state;
	unsigned char control_in[5];
	int nreceiver, num_hpsdr_receivers;
	int left_sample;
	int right_sample;
	int nsamples;
	int iq_samples;
	int rx1channel;
	float left_sample_float;
	float right_sample_float;
	bool duplex;
	bool validate_hpsdr_response(const uint8_t *buffer, int length, HPSDR_Device_Info *info);
	void process_input_buffer(const uint8_t *buffer);
	void process_byte(int b, std::vector<std::complex<float>> &iqsamples);
	void process_control_bytes();
	std::atomic_bool stream_active{false};
	SoapyHPSDR *device;
};

extern std::shared_ptr<ReceiveThread> ptr_receive_thread;