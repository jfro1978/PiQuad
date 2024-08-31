#ifndef _LOOP_CLASS
#define _LOOP_CLASS

#include "imu.h"

static const float IMU_DATA_FILTER = 0.5;

enum class quadStateEnum
{
	NOT_STARTED,
	PREPARING_FOR_FLIGHT,
	READY_FOR_FLIGHT
};

class Loop 
{
public:
	Loop();
	~Loop() = default;

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

	quadStateEnum mQuadState;

	unsigned int mReceiverChannel1;
	unsigned int mReceiverChannel2;
	unsigned int mReceiverChannel3;
	unsigned int mReceiverChannel4;
};

#endif // _LOOP_CLASS