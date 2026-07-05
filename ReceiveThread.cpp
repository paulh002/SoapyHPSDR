#include "ReceiveThread.hpp"
#include "DataBuffer.h"
#include <sys/socket.h>
#include <sys/time.h> // For struct timeval (timeout)
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

std::thread rx_thread;
std::shared_ptr<ReceiveThread> ptr_rx_thread;

const int buffer_size = 65536;

bool ReceiveThread::create_receive_thread(int receive_socket, SoapyHPSDR *device)
{
	if (ptr_rx_thread != nullptr)
		return false;

	ptr_rx_thread = std::make_shared<ReceiveThread>(receive_socket, device);
	rx_thread = std::thread(&ReceiveThread::operator(), ptr_rx_thread);
	return true;
}

ReceiveThread::ReceiveThread(int receive_socket, SoapyHPSDR *_device)
	: _receive_socket(receive_socket), metis(false), device (_device)
{
	state = SYNC_0;
	memset(control_in,0, sizeof(control_in));
	nreceiver = 0;
	num_hpsdr_receivers = 0;
	duplex = false;
	nsamples = 0;
	rx1channel = 0;
}

void ReceiveThread::SetStreamActive(bool active)
{
	stream_active = active;
}

void ReceiveThread::operator()()
{
	uint8_t buffer[buffer_size] = {0};
	uint32_t sequence{0}, last_seq_num{0};
	int bytes_received;

	bytes_received = recv(_receive_socket, buffer, buffer_size - 1, 0);
	printf("Bytes received %d\n", bytes_received);
	HPSDR_Device_Info device_info;
	if (validate_hpsdr_response(buffer, bytes_received, &device_info))
	{
		num_hpsdr_receivers = 1;
		while (1)
		{
			std::chrono::time_point<std::chrono::high_resolution_clock> stoptReadTime;
			std::chrono::microseconds timePassed{};
						
			const auto startTime = std::chrono::high_resolution_clock::now();

			bytes_received = recv(_receive_socket, buffer, sizeof(buffer) - 1, MSG_WAITALL);
			sequence = ((buffer[4] & 0xFF) << 24) + ((buffer[5] & 0xFF) << 16) + ((buffer[6] & 0xFF) << 8) + (buffer[7] & 0xFF);

			if (sequence != 0 && sequence != last_seq_num + 1)
			{
				char str[180];
				sprintf(str, "SoapyHPSDR::SoapyHPSDR Sequence error got %d expect %d", sequence, last_seq_num + 1);
				SoapySDR_log(SOAPY_SDR_ERROR, str);
			}
			last_seq_num = sequence;
			stoptReadTime = std::chrono::high_resolution_clock::now();
			timePassed = std::chrono::duration_cast<std::chrono::microseconds>(stoptReadTime - startTime);
			if (stream_active && bytes_received == PACKETSIZE)
			{
				//printf("Stream active Bytes received %d sequence %d ep %d micro sec %ld\n", bytes_received, sequence, (int)(buffer[3] & 0xFF), timePassed.count());
				if ((buffer[3] & 0xFF) == 6)
				{
					process_input_buffer(&buffer[8]);
					process_input_buffer(&buffer[520]);
				}
			}
		}
	}
}

void ReceiveThread::process_input_buffer(const uint8_t *buffer)
{
	std::vector<std::complex<float>> iqsamples;
	for (int i = 0; i < 512; i++)
	{
		process_byte(buffer[i] & 0xFF, iqsamples);	
	}
	device->push_iqsamples(iqsamples);
}

void ReceiveThread::process_byte(int b, std::vector<std::complex<float>> &iqsamples)
{
	switch (state)
	{
	case SYNC_0:
		if (b == SYNC)
		{
			state++;
		}
		break;
	case SYNC_1:
		if (b == SYNC)
		{
			state++;
		}
		break;
	case SYNC_2:
		if (b == SYNC)
		{
			state++;
		}
		break;
	case CONTROL_0:
		control_in[0] = b;
		state++;
		break;
	case CONTROL_1:
		control_in[1] = b;
		state++;
		break;
	case CONTROL_2:
		control_in[2] = b;
		state++;
		break;
	case CONTROL_3:
		control_in[3] = b;
		state++;
		break;
	case CONTROL_4:
		control_in[4] = b;
		process_control_bytes();
		nreceiver = 0;
		iq_samples = (512 - 8) / ((num_hpsdr_receivers * 6) + 2);
		nsamples = 0;
		state++;
		break;
	case LEFT_SAMPLE_HI:
		left_sample = (int)((signed char)b << 16);
		state++;
		break;
	case LEFT_SAMPLE_MID:
		left_sample |= (int)((((unsigned char)b) << 8) & 0xFF00);
		state++;
		break;
	case LEFT_SAMPLE_LOW:
		left_sample |= (int)((unsigned char)b & 0xFF);
		left_sample_float = (float)left_sample / 8388607.0; // 24 bit sample 2^23-1
		state++;
		break;
	case RIGHT_SAMPLE_HI:
		right_sample = (int)((signed char)b << 16);
		state++;
		break;
	case RIGHT_SAMPLE_MID:
		right_sample |= (int)((((unsigned char)b) << 8) & 0xFF00);
		state++;
		break;
	case RIGHT_SAMPLE_LOW: {
		right_sample |= (int)((unsigned char)b & 0xFF);
		right_sample_float = (float)right_sample / 8388607.0; // 24 bit sample 2^23-1

		std::complex<float> new_iqsample(right_sample_float, left_sample_float);
		iqsamples.push_back(new_iqsample);
		
		nreceiver++;
		if (nreceiver == num_hpsdr_receivers)
		{
			state++;
		}
		else
		{
			state = LEFT_SAMPLE_HI;
		}
		break;
	}
	case MIC_SAMPLE_HI:
		state++;
		break;
	case MIC_SAMPLE_LOW:
		nsamples++;
		if (nsamples == iq_samples)
		{
			state = SYNC_0;
		}
		else
		{
			nreceiver = 0;
			state = LEFT_SAMPLE_HI;
		}
		break;
	}
}

void ReceiveThread::process_control_bytes()
{
	
}

bool ReceiveThread::validate_hpsdr_response(const uint8_t *buffer, int length, HPSDR_Device_Info *info)
{
	// 1. VALIDATE LENGTH
	// The HPSDR discovery response is strictly 60 bytes.
	if (length != 60)
	{
		printf("[-] Validation Failed: Invalid packet length. Expected 60, got %d.\n", length);
		//return false;
	}

	// 2. VALIDATE HPSDR MAGIC HEADER
	// Bytes 0 and 1 must be 0xEF and 0xFE.
	if (buffer[0] != 0xEF || buffer[1] != 0xFE)
	{
		printf("[-] Validation Failed: Invalid HPSDR header. Expected EF FE, got %02X %02X.\n",
			   buffer[0], buffer[1]);
		return false;
	}

	// 3. VALIDATE RESPONSE CODE
	// Byte 2 must be 0x04 to indicate a Discovery Response.
	if (buffer[2] == 0x02)
	{
		SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::validate_hpsdr_response  metis protocol discovered");
		metis = true;
	}
	else if (buffer[2] == 0x03 || buffer[2] == 0x04)
	{
		SoapySDR_log(SOAPY_SDR_INFO, "SoapyHPSDR::validate_hpsdr_response  Hermes protocol discovered");
		metis = true;
	}
	else 
	{
		printf("[-] Validation Failed: Not a discovery response. Expected command 0x02, 0x03, 0x04, got 0x%02X.\n",
			   buffer[2]);
		return false;
	}

	// ---------------------------------------------------------
	// IF WE REACH HERE, THE PACKET IS VALID. PARSE THE DATA.
	// ---------------------------------------------------------

	// 4. Extract MAC Address (Bytes 4 through 9)
	// We use memcpy for safe, optimized memory copying on the ARM processor.
	std::memcpy((char *)info->mac, &buffer[4], 6);

	// 5. Extract Gateware/Firmware Version (Byte 10)
	info->gateware_version = buffer[10];

	return true;
}