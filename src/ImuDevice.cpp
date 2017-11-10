#include "ImuDevice.hpp"
#include "tara/Tara.h"
#include <pthread.h>
#include <signal.h>

/* Killing the thread */
void KillThread(int sig)
{
	pthread_exit( EXIT_SUCCESS );
}

bool initCamera()
{
	int device_id;
	cv::Size image_size;

	// Read the device ID to stream
	Tara::CameraEnumeration camera_enumeration(&device_id, &image_size);

	// Check for a valid device ID
	if(device_id < 0)
		throw std::runtime_error("Please select a valid device");

	//Init the extension units
	if(not InitExtensionUnit(camera_enumeration.DeviceInfo))
		throw std::runtime_error("Extension Unit Initialisation Failed");

	return true;
}

/* returns the interval time for sampling the values of the IMU */
float GetIMUIntervalTime(IMUCONFIG_TypeDef device_config)
{
	float lIMUIntervalTime = 10;
	if(device_config.IMU_MODE == IMU_ACC_GYRO_ENABLE)
	{
		switch(device_config.IMU_ODR_CONFIG)
		{
			case IMU_ODR_10_14_9HZ:
				lIMUIntervalTime = float(1000.00/14.90);
				break;

			case IMU_ODR_50_59_9HZ:
				lIMUIntervalTime = float(1000.00/59.50);
				break;

			case IMU_ODR_119HZ:
				lIMUIntervalTime = float(1000.00/119.00);
				break;

			case IMU_ODR_238HZ:
				lIMUIntervalTime = float(1000.00/238.00);
				break;

			case IMU_ODR_476HZ:
				lIMUIntervalTime = float(1000.00/476.00);
				break;

			case IMU_ODR_952HZ:
				lIMUIntervalTime = float(1000.00/952.00);
				break;
		}
	}
	else if(device_config.IMU_MODE == IMU_ACC_ENABLE)
	{
		switch(device_config.IMU_ODR_CONFIG)
		{
			case IMU_ODR_10_14_9HZ:
				lIMUIntervalTime = float(1000.00/10.00);
				break;

			case IMU_ODR_50_59_9HZ:
				lIMUIntervalTime = float(1000.00/50.00);
				break;

			case IMU_ODR_119HZ:
				lIMUIntervalTime = float(1000.00/119.00);
				break;

			case IMU_ODR_238HZ:
				lIMUIntervalTime = float(1000.00/238.00);
				break;

			case IMU_ODR_476HZ:
				lIMUIntervalTime = float(1000.00/476.00);
				break;

			case IMU_ODR_952HZ:
				lIMUIntervalTime = float(1000.00/952.00);
				break;
		}
	}
	return lIMUIntervalTime;
}

///////////////////////////////////////////////////////////////////////////

ImuDevice::ImuDevice()
	: data_buffer_( nullptr ), next_measurement_( nullptr )
{
	// Initialise the camera
	if ( not initCamera() )
		throw std::runtime_error("Camera Initialisation Failed");

	configureDevice();

	enableDevice();

	// Allocating buffers for output structure
	if(imu_update_mode_.IMU_UPDATE_MODE == IMU_CONT_UPDT_EN)
		data_buffer_ = (IMUDATAOUTPUT_TypeDef*)malloc(IMU_AXES_VALUES_MAX * sizeof(IMUDATAOUTPUT_TypeDef));
	else
		data_buffer_ = (IMUDATAOUTPUT_TypeDef*)malloc(1 * sizeof(IMUDATAOUTPUT_TypeDef));

	next_measurement_ = data_buffer_;

	// Memory validation
	if(not next_measurement_)
		throw std::runtime_error("Memory Allocation for otput failed");

	next_measurement_->IMU_VALUE_ID = 0;

	// Semaphore should start locked, because there is no data avaible.
	data_ready_event_.lock();
}

ImuDevice::~ImuDevice()
{
	disableDevice();

	//Freeing the memory
	delete data_buffer_;
	DeinitExtensionUnit();
}

void ImuDevice::run(ImuDevice::Callback_t callback)
{
	thread_ = std::thread(&ImuDevice::getImuValues, this);

	IMUDATAOUTPUT_TypeDef* lIMUOutputAdd = next_measurement_;

	//Blocking call waits for unlock event
	while((imu_update_mode_.IMU_UPDATE_MODE == IMU_CONT_UPDT_EN) || 
		(next_measurement_->IMU_VALUE_ID <= imu_update_mode_.IMU_NUM_OF_VALUES))
	{
		if(imu_update_mode_.IMU_UPDATE_MODE != IMU_CONT_UPDT_DIS)
			data_ready_event_.lock();

		callback(imu_update_interval_, *next_measurement_);

		//Round robin mechanism to use the same buffer
		if(next_measurement_->IMU_VALUE_ID < IMU_AXES_VALUES_MAX)
			next_measurement_++;
		else
			next_measurement_ = lIMUOutputAdd;
	}

	// C++11 thread objects don't implement signaling,
	// so we need to convert back to the native type.
	pthread_kill(thread_.native_handle(), SIGUSR1);
}

void ImuDevice::enableDevice()
{
	IMUDATAINPUT_TypeDef enable_config;

	// Configuring IMU update mode
	enable_config.IMU_UPDATE_MODE = IMU_CONT_UPDT_EN;
	enable_config.IMU_NUM_OF_VALUES = IMU_AXES_VALUES_MIN;

	//Setting the IMU update mode using HID command
	if (not ControlIMUCapture( &enable_config ))
		throw std::runtime_error("ControlIMUCapture Failed");

	imu_update_mode_ = enable_config;
}

void ImuDevice::disableDevice()
{
	IMUDATAINPUT_TypeDef disable_config;
	disable_config.IMU_UPDATE_MODE = IMU_CONT_UPDT_DIS;
	disable_config.IMU_NUM_OF_VALUES = IMU_AXES_VALUES_MIN;

	// Resetting the IMU update to disable mode
	if (not ControlIMUCapture( &disable_config ))
		throw std::runtime_error("ControlIMUCapture Failed");
}

void ImuDevice::configureDevice()
{
	IMUCONFIG_TypeDef device_config;

	//Configuring IMU rates
	device_config.IMU_MODE = IMU_ACC_GYRO_ENABLE;
	device_config.ACC_AXIS_CONFIG = IMU_ACC_X_Y_Z_ENABLE;
	device_config.IMU_ODR_CONFIG = IMU_ODR_119HZ;
	device_config.ACC_SENSITIVITY_CONFIG = IMU_ACC_SENS_2G;
	device_config.GYRO_AXIS_CONFIG = IMU_GYRO_X_Y_Z_ENABLE;
	device_config.GYRO_SENSITIVITY_CONFIG = IMU_GYRO_SENS_245DPS;

	//Setting the configuration using HID command
	if (not SetIMUConfig(device_config))
		throw std::runtime_error("SetIMUConfig Failed");

	//Reading the configuration to verify the values are set 
	if (not GetIMUConfig(&device_config))
		throw std::runtime_error("GetIMUConfig Failed");

	//Finding the sampling interval time
	imu_update_interval_ = GetIMUIntervalTime(device_config);
}

void ImuDevice::getImuValues()
{
	signal(SIGUSR1, KillThread);

	// This command is blocking and signals the data ready event.
	if(not GetIMUValueBuffer(data_ready_event_.native_handle() , next_measurement_))
		throw std::runtime_error("GetIMUValueBuffer Ended");
}
