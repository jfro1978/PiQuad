#ifndef _LOOP_CLASS
#define _LOOP_CLASS

#include "imu.h"
#include "pid.h"

static const float IMU_DATA_FILTER = 0.5;

static const float PID_P_ROLL_GAIN = 1.0f; 
static const float PID_I_ROLL_GAIN = 0.12f;
static const float PID_D_ROLL_GAIN = 15.0f;
static const int max_roll_rate = 250; //Max roll rate

static const float PID_P_PITCH_GAIN = 1.0f;
static const float PID_I_PITCH_GAIN = 0.12f;
static const float PID_D_PITCH_GAIN = 15.0f;
static const int max_pitch_rate = 250; //Max pitch rate

static const float PID_P_YAW_GAIN = 3.0f;
static const float PID_I_YAW_GAIN = 0.02f;
static const float PID_D_YAW_GAIN = 0.0f;
static const int max_yaw_rate = 250; //Max yaw rate

namespace Quad
{
	namespace Loop
	{

		enum class quadStateEnum
		{
			STANDBY,
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

			std::chrono::system_clock::time_point mLoopTimer;
		};

} // namespace Loop
} // namespace Quad

#endif // _LOOP_CLASS