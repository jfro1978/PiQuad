#ifndef _LOOP_CLASS
#define _LOOP_CLASS

#include "imu.h"
#include "pid.h"
#include <wiringPi.h>

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

static const float PID_P_ALTITUDE_GAIN = 3.0f;
static const float PID_I_ALTITUDE_GAIN = 0.02f;
static const float PID_D_ALTITUDE_GAIN = 0.0f;
static const int max_altitude = 10; // Units: metres

static const int THROTTLE_INTERRUPT_PIN = 17;

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
			// Member variables
			float mRawIMU_GyroPitchData;
			float mRawIMU_GyroRollData;
			float mRawIMU_GyroYawData;

			float mRawIMU_AccelX_Data;
			float mRawIMU_AccelY_Data;
			float mRawIMU_AccelZ_Data;

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

			std::chrono::system_clock::time_point mLoopStartTime;

			unsigned int mPWM_FrontRight;
			unsigned int mPWM_FrontLeft;
			unsigned int mPWM_RearRight;
			unsigned int mPWM_RearLeft;

			uint8_t rxCh1Pin;
			uint8_t ch1PinLastLevel;
			std::chrono::system_clock::time_point mRxCh1StartTime;
			uint8_t rxCh2Pin;
			uint8_t ch2PinLastLevel;
			std::chrono::system_clock::time_point mRxCh2StartTime;
			uint8_t rxCh3Pin;
			uint8_t ch3PinLastLevel;
			std::chrono::system_clock::time_point mRxCh3StartTime;
			uint8_t rxCh4Pin;
			uint8_t ch4PinLastLevel;
			std::chrono::system_clock::time_point mRxCh4StartTime;

		private:
			// Member functions
			void interruptHandlerPin7(void);
			void interruptHandlerPin8(void);
			void interruptHandlerPin9(void);
			void interruptHandlerPin10(void);

		};

} // namespace Loop
} // namespace Quad

#endif // _LOOP_CLASS