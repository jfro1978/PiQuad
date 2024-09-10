#include "loop.h"
#include "imu.h"
#include "pid.h"
#include <pigpio.h>
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
			mReceiverChannel1(1500), // Hardcoded while developing altitude control algorithm
			mReceiverChannel2(1500), // Hardcoded while developing altitude control algorithm
			mReceiverChannel3(1000),
			mReceiverChannel4(1500), // Hardcoded while developing altitude control algorithm
			mLoopStartTime(std::chrono::system_clock::now())
		{
		}

		void Loop::run()
		{
			// TODO: Set LED on to indicate program is running

			// Create IMU object
			Quad::IMU::intertialMeasurementUnit mpu6050(Quad::IMU::gyroConfigEnum::FS_250_DPS, 
				Quad::IMU::accelConfigEnum::AFS_2_G);

			mpu6050.initialiseIMU(); // TODO: This should be called from the constructor. Also can it be done from pid class?
									 // Not sure it's needed in this loop class?

			// Set LED to indicate that IMU calibration is complete

			// Create PID controller object
			Quad::PID::pidController pid(PID_P_PITCH_GAIN, PID_P_ROLL_GAIN, PID_P_YAW_GAIN, PID_P_ALTITUDE_GAIN,
				PID_I_PITCH_GAIN, PID_I_ROLL_GAIN, PID_I_YAW_GAIN, PID_I_ALTITUDE_GAIN,
				PID_D_PITCH_GAIN, PID_D_ROLL_GAIN, PID_D_YAW_GAIN, PID_D_ALTITUDE_GAIN);

			while (1)
			{
				// Check conditions for preparing quad for flight
				if (mReceiverChannel3 < 1050 && mReceiverChannel4 < 1050)
				{
					mQuadState = quadStateEnum::PREPARING_FOR_FLIGHT;
				}

				/* Check if state has just transitioned from PREPARING_FOR_FLIGHT to READY_FOR_FLIGHT.
					If so, reset historical PID error values. */
				if ((mQuadState == quadStateEnum::PREPARING_FOR_FLIGHT) &&
					(mReceiverChannel3 < 1050) &&
					(mReceiverChannel4 > 1450))
				{
					pid.resetPreviousErrorValues();
					mQuadState = quadStateEnum::READY_FOR_FLIGHT;
				}

				// Check conditions for stopping flight, i.e. to move back to STANDBY
				if ((mQuadState == quadStateEnum::READY_FOR_FLIGHT) &&
					(mReceiverChannel3 < 1050) &&
					(mReceiverChannel4 > 1950))
				{
					mQuadState = quadStateEnum::STANDBY;
				}

				// Determine PWM value for each motor if quad is ready for flight
				if (quadStateEnum::READY_FOR_FLIGHT == mQuadState)
				{
					/* Impose upper limit (PWM: 1800) on throttle setting, then determine setpoints for pitch, 
						roll, yaw, and altitude. This is done to ensure that even at the max throttle setting, 
						there is still some (PWM) margin available to stabilise the quad. */

					if (mReceiverChannel3 > 1800)
					{
						mReceiverChannel3 = 1800;
					}
					pid.determineSetpoints(mReceiverChannel1, mReceiverChannel2, mReceiverChannel3, mReceiverChannel4);

					// Read gyro values
					mpu6050.readIMU_Data(mRawIMU_GyroPitchData, mRawIMU_GyroRollData, mRawIMU_GyroYawData,
						mRawIMU_AccelX_Data, mRawIMU_AccelY_Data, mRawIMU_AccelZ_Data);

					// Filter the IMU data to limit the effect of spikes in the received IMU data
					mGyroPitch = (mGyroPitch * IMU_DATA_FILTER) + (mRawIMU_GyroPitchData * IMU_DATA_FILTER);
					mGyroRoll = (mGyroRoll * IMU_DATA_FILTER) + (mRawIMU_GyroRollData * IMU_DATA_FILTER);
					mGyroYaw = (mGyroYaw * IMU_DATA_FILTER) + (mRawIMU_GyroYawData * IMU_DATA_FILTER);

					mAccelX = (mAccelX * IMU_DATA_FILTER) + (mRawIMU_AccelX_Data * IMU_DATA_FILTER);
					mAccelY = (mAccelY * IMU_DATA_FILTER) + (mRawIMU_AccelY_Data * IMU_DATA_FILTER);
					mAccelZ = (mAccelZ * IMU_DATA_FILTER) + (mRawIMU_AccelZ_Data * IMU_DATA_FILTER);

					pid.determineAxisPID_Outputs();

					// Determine mPWM_FrontLeft, mPWM_FrontRight, mPWM_RearLeft, and mPWM_RearRight

					// Write signals to ESC class
				}
				else if(quadStateEnum::PREPARING_FOR_FLIGHT == mQuadState)
				{
					mPWM_FrontLeft = 1000;
					mPWM_FrontRight = 1000;
					mPWM_RearLeft = 1000;
					mPWM_RearRight = 1000;

					// Write signals to ESC class
				}

				// 8) Loop until 4ms since last loop has expired
				std::this_thread::sleep_until(mLoopStartTime + std::chrono::milliseconds(4));
				mLoopStartTime = std::chrono::system_clock::now();
				

				/*
				9) Work out how long from now the ESCs have to be high for, i.e. get current time, then add the value from either 7c or 7d to this time.
					Do in ESC class
				10) Set all ESCs to high
					Do in ESC class

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

		void Loop::rcInterruptHandler(int gpio, int level, uint32_t tick) 
		{
			if (level == 1) {
				std::cout << "Rising edge detected on GPIO " << gpio << std::endl;
			}
			else if (level == 0) {
				std::cout << "Falling edge detected on GPIO " << gpio << std::endl;
			}
		}

	} // namespace Loop
} // namespace Quad
