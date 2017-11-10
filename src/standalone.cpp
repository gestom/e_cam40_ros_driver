#include "ImuDevice.hpp"
#include "AHRS.hpp"
#include <iostream>

// This needs to be outside so the signal handle can stop it properly.
std::unique_ptr<ImuDevice> device;

// Imu model used to integrate angular velocity values.
Tara::AHRS ahrs;

void sigintHandler(int sig)
{
  device.reset();
}

void processData(float dt, const IMUDATAOUTPUT_TypeDef& measurement)
{
  //~ std::cout << measurement.gyroX << " " << measurement.gyroY << " " << measurement.gyroZ << std::endl;

  //Calculating angles based on the current raw values from IMU			
  ahrs.getInclination(dt, measurement.gyroX, measurement.gyroY, measurement.gyroZ, measurement.accX, measurement.accY, measurement.accZ);

  float angleX, angleY, angleZ;
  ahrs.getAngles(angleX, angleY, angleZ);
  std::cout << (int)angleX << " " << (int)angleY << " " << (int)angleZ << std::endl;
}

int main()
{
  /*std::cout << std::endl << " IMU Sample Application " << std::endl << std::endl;
  std::cout << " Application to illustrate the IMU unit LSM6DS0 integrated with Tara Camera" << std::endl;
  std::cout << " Demonstrating the rotations of camera around x-axis and y-axis " << std::endl;
  std::cout << " IMU values are limited from -90 to +90 degrees for illustration " << std::endl << std::endl;*/

  signal(SIGINT, sigintHandler);

  device = std::unique_ptr<ImuDevice>(new ImuDevice());

  std::cout << "Hit Ctrl-c to stop" << std::endl;

  device->run( processData );

  std::cout << "Keyboard hitted" << std::endl;

	return EXIT_SUCCESS;
}
