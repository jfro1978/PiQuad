#include "loop.h"
#include <chrono>
#include <thread>

namespace Quad
{
	namespace Loop
	{
		Loop::Loop() :
			mRawIMU_GyroPitchData(0),
			mRawIMU_GyroRollData(0),
			mRawIMU_GyroYawData(0),
			mRawIMU_AccelX_Data(0),
			mRawIMU_AccelY_Data(0),
			mRawIMU_AccelZ_Data(0),
			mAccelX(0),
			mAccelY(0),
			mAccelZ(0),
			mQuadState(quadStateEnum::STANDBY),
			mReceiverChannel1(1000),
			mReceiverChannel2(1000),
			mReceiverChannel3(1000),
			mReceiverChannel4(1000),
			mLoopTimer(std::chrono::system_clock::now())
		{
		}

		void Loop::run()
		{
			// TODO: Set LED on to indicate program is running

			// Create IMU object
			intertialMeasurementUnit mpu6050(gyroConfigEnum::FS_250_DPS, accelConfigEnum::AFS_2_G);

			mpu6050.initialiseIMU(); // TODO: This should be called from the constructor

			// Set LED to indicate that IMU calibration is complete

			// Create PID controller object
			pidController pid(PID_P_PITCH_GAIN, PID_P_ROLL_GAIN, PID_P_YAW_GAIN,
				PID_I_PITCH_GAIN, PID_I_ROLL_GAIN, PID_I_YAW_GAIN,
				PID_D_PITCH_GAIN, PID_D_ROLL_GAIN, PID_D_YAW_GAIN);

			while (1)
			{
				// 1) Read gyro
				mpu6050.readIMU_Data(mRawIMU_GyroPitchData, mRawIMU_GyroRollData, mRawIMU_GyroYawData,
					mRawIMU_AccelX_Data, mRawIMU_AccelY_Data, mRawIMU_AccelZ_Data);

				// 2) Filter the IMU data to limit the effect of spikes in the received IMU data
				mGyroPitch = (mGyroPitch * IMU_DATA_FILTER) + (mRawIMU_GyroPitchData * IMU_DATA_FILTER);
				mGyroRoll = (mGyroRoll * IMU_DATA_FILTER) + (mRawIMU_GyroRollData * IMU_DATA_FILTER);
				mGyroYaw = (mGyroYaw * IMU_DATA_FILTER) + (mRawIMU_GyroYawData * IMU_DATA_FILTER);

				mAccelX = (mAccelX * IMU_DATA_FILTER) + (mRawIMU_AccelX_Data * IMU_DATA_FILTER);
				mAccelY = (mAccelY * IMU_DATA_FILTER) + (mRawIMU_AccelY_Data * IMU_DATA_FILTER);
				mAccelZ = (mAccelZ * IMU_DATA_FILTER) + (mRawIMU_AccelZ_Data * IMU_DATA_FILTER);

				// 3) Check conditions for getting quad ready for flight, i.e.changing state of quadStateEnum variable
				if (mReceiverChannel3 < 1050 && mReceiverChannel4 < 1050)
				{
					mQuadState = quadStateEnum::PREPARING_FOR_FLIGHT;
				}

				/* 4) Check if state has just transitioned from PREPARING_FOR_FLIGHT to READY_FOR_FLIGHT.
					  If so, reset historical PID error values. */
				if ((mQuadState == quadStateEnum::PREPARING_FOR_FLIGHT) &&
					(mReceiverChannel3 < 1050) &&
					(mReceiverChannel4 > 1450))
				{
					pid.resetPreviousErrorValues();
					mQuadState = quadStateEnum::READY_FOR_FLIGHT;
				}

				// 5) Check conditions for stopping flight, i.e. to move back to STANDBY
				if ((mQuadState == quadStateEnum::READY_FOR_FLIGHT) &&
					(mReceiverChannel3 < 1050) &&
					(mReceiverChannel4 > 1950))
				{
					mQuadState = quadStateEnum::STANDBY;
				}

				// 6)

				// 7)

				// 8) Loop until 4ms since last loop has expired
				{
					std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
					std::chrono::system_clock::time_point timeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime);
				}

				/*

				6) Determine PID setpoints for pitch, roll, and yaw, i.e. max PWM signal of 2000us should correspond to +max rate, 1000us should correspond to -max rate, and 1500 should be zero

				7) if READY_FOR_FLIGHT,
					7a) throttle = receiver_channel_3;
					7b) calculate the pid pitch, roll, and yaw outputs, i.e. the error between actual and desired, times by gain
					7c) use these to determine value to send to each ESC
				else
					7d) set all ESC values to 1000us

				8) Loop until 4ms since last loop has expired

				9) Work out how long from now the ESCs have to be high for, i.e. get current time, then add the value from either 7c or 7d to this time.

				10) Set all ESCs to high

				11) While any of the ESCs are still high, loop on the following
					11a) - for each ESC output, check if the current time is less than the time when the output should go low. If less, stay high, otherwise go low.

				*/
			}


			/*
			short  x, y, z, a, b, c = 0;

			while (true)
			{
				readIMU_Data(x, y, z, a, b, c);

				float x1 = (x / GYRO_LSB_VALUE);
				std::cout << "Gyro X: " << (x1 - offsetGyroPitch) << ", ";

				float y1 = (y / GYRO_LSB_VALUE);
				std::cout << "Gyro Y: " << (y1 - offsetGyroRoll) << ", ";

				float z1 = (z / GYRO_LSB_VALUE);
				std::cout << "Gyro Z: " << (z1 - offsetGyroYaw) << ", ";

				float a1 = (a / ACCEL_LSB_VALUE);
				std::cout << "Accel X: " << (a1 - offsetAccelX) << ", ";

				float b1 = (b / ACCEL_LSB_VALUE);
				std::cout << "Accel Y: " << (b1 - offsetAccelY) << ", ";

				float c1 = (c / ACCEL_LSB_VALUE);
				std::cout << "Accel Z: " << (c1 - offsetAccelZ) << std::endl;


				usleep(10000);
			}

			*/
		}

	} // namespace Loop
} // namespace Quad
