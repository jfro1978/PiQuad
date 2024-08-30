#ifndef _LOOP_CLASS
#define _LOOP_CLASS

#include "imu.h"

static const float IMU_DATA_FILTER = 0.5;

class Loop 
{
public:
	void run();

private:
	short mRawIMU_GyroPitchData;
	short mRawIMU_GyroRollData;
	short mRawIMU_GyroYawData;

	short mRawIMU_AccelX_Data;
	short mRawIMU_AccelY_Data;
	short mRawIMU_AccelZ_Data;

	short mGyroPitch;
	short mGyroRoll;
	short mGyroYaw;

	short mAccelX;
	short mAccelY;
	short mAccelZ;
};

#endif // _LOOP_CLASS