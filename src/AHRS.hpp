#pragma once

namespace Tara
{

class AHRS
{
	public:

		//Constructor
		AHRS(void);

		//Initialises the variables
		int Init();

		// Function declarations
		void getInclination(float dt, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);

		void getAngles(float& angleX, float& angleY, float& angleZ) const;

	private:

		// Rotational angle for cube [NEW]
		float angleX_, angleY_, angleZ_;
		float RwEst[3];

		float squared(float x);
};

};
