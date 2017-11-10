#include "AHRS.hpp"
#include <math.h>

#define M_PI			3.14159265358979323846
#define DEG2RAD		(float)(M_PI / 180.f)
#define RAD2DEG		(float)(180.f / M_PI)

using namespace Tara;

/* Initialises all the values */
AHRS::AHRS(void)
	: angleX_( 0.0f ), angleY_( 0.0f ), angleZ_( 0.0f )
{}

/* Square of a number */
float AHRS::squared(float x)
{
	return x * x;
}

/* Computes the angle of rotation with respect to the axes */
void AHRS::getInclination(float dt, float g_x, float g_y, float g_z, float a_x, float a_y, float a_z)
{
	int w = 0;
	float tmpf = 0.0;
	int signRzGyro;
	static bool firstSample = true;
	float wGyro = 10.0;
	float norm;

	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	float RwAcc[3] = {a_x, a_y, a_z};
	float RwGyro[3] = { g_x, g_y, g_z };
	float Awz[2];
	float Gyro[3];


	if (firstSample)
	{ 
		//initialize with accelerometer readings
		for (w = 0; w <= 2; w++) 
		{
			RwEst[w] = RwAcc[w];    
		}
	}

	else 
	{
		//evaluate Gyro vector
		if (fabs(RwEst[2]) < 0.1) 
		{
			//Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
			//in this case skip the gyro data and just use previous estimate
			for (w = 0;w <= 2;w++) 
			{
				Gyro[w] = RwEst[w];
			}
		}
		else {
			//get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
			for (w = 0;w <= 1;w++) 
			{
				tmpf = RwGyro[w];                        //get current gyro rate in deg/s
				tmpf *= dt / 1000.0f;                     //get angle change in deg
				Awz[w] = atan2(RwEst[w], RwEst[2]) * 180 / float(M_PI);   //get angle and convert to degrees 
				Awz[w] += tmpf;             //get updated angle according to gyro movement
			}

			//estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
			//RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			signRzGyro = (cos(Awz[0] * float(M_PI) / 180) >= 0) ? 1 : -1;

			//reverse calculation of Gyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
			for (w = 0; w <= 1; w++) 
			{
				Gyro[0] = sin(Awz[0] * float(M_PI) / 180);
				Gyro[0] /= sqrt(1 + squared(cos(Awz[0] * float(M_PI) / 180)) * squared(tan(Awz[1] * float(M_PI) / 180)));
				Gyro[1] = sin(Awz[1] * float(M_PI) / 180);
				Gyro[1] /= sqrt(1 + squared(cos(Awz[1] * float(M_PI) / 180)) * squared(tan(Awz[0] * float(M_PI) / 180)));
			}
			Gyro[2] = signRzGyro * sqrt(1 - squared(Gyro[0]) - squared(Gyro[1]));
		}

		//combine Accelerometer and gyro readings
		for (w = 0; w <= 2; w++) 
			RwEst[w] = (RwAcc[w] + wGyro * Gyro[w]) / (1 + wGyro);

		//Normalizing the estimates
		norm = sqrt(RwEst[0] * RwEst[0] + RwEst[1] * RwEst[1] + RwEst[2] * RwEst[2]);
		RwEst[0] /= norm;
		RwEst[1] /= norm;
		RwEst[2] /= norm;
	}

	firstSample = false;

	//Computing the angles
	angleX_ = RwEst[0] * float(M_PI) / 2 * RAD2DEG;
	angleY_ = RwEst[1] * float(M_PI) / 2 * RAD2DEG;
	angleZ_ = RwEst[2] * float(M_PI) / 2 * RAD2DEG;

	//printf("%0.2f %0.2f %0.2f , %0.2f %0.2f %0.2f , %0.2f %0.2f %0.2f \n", g_x, g_y, g_z, a_x, a_y, a_z, angleX, angleY, angleZ);
}

void AHRS::getAngles(float& angleX, float& angleY, float& angleZ) const
{
	angleX = angleX_;
	angleY = angleY_;
	angleZ = angleZ_;
}
