#pragma once

#include "tara/xunit_lib_tara.h"
#include <mutex>
#include <thread>

class ImuDevice
{
	public:

		typedef std::function<void(float, const IMUDATAOUTPUT_TypeDef&)> Callback_t;

		ImuDevice();
		~ImuDevice();

		// data processing function. Blocking!
		void run(Callback_t callback);

	private:

		float imu_update_interval_;
		IMUDATAINPUT_TypeDef imu_update_mode_;

		std::mutex data_ready_event_;

		IMUDATAOUTPUT_TypeDef* data_buffer_;
		IMUDATAOUTPUT_TypeDef* next_measurement_;

		// data aquisition thread.
		std::thread thread_;

	// helper functions

		void enableDevice();
		void disableDevice();

		void configureDevice();

		// data aquisition function. Blocking!
		void getImuValues();
};
