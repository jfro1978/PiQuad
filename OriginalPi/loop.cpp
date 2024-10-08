#include "loop.h"
#include "imu.h"
#include "pid.h"
#include <chrono>
#include <thread>
#include <iostream>

namespace Quad
{
	namespace Loop
	{
		int fd;
		constexpr uint8_t Device_Address = 0x68;
		constexpr uint8_t PWR_MGMT_1 = 0x6B;
		constexpr uint8_t SMPLRT_DIV = 0x19;
		constexpr uint8_t CONFIG = 0x1A;
		constexpr uint8_t GYRO_CONFIG = 0x1B;
		constexpr uint8_t INT_ENABLE = 0x38;
		constexpr uint8_t ACCEL_XOUT_H = 0x3B;
		constexpr uint8_t ACCEL_YOUT_H = 0x3D;
		constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
		constexpr uint8_t GYRO_XOUT_H = 0x43;
		constexpr uint8_t GYRO_YOUT_H = 0x45;
		constexpr uint8_t GYRO_ZOUT_H = 0x47;

		void MPU6050_Init() {

			wiringPiI2CWriteReg8(fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
			wiringPiI2CWriteReg8(fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
			wiringPiI2CWriteReg8(fd, CONFIG, 0);		/* Write to Configuration register */
			wiringPiI2CWriteReg8(fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
			wiringPiI2CWriteReg8(fd, INT_ENABLE, 0x01);	/* Write to interrupt enable register */
		}

		short read_raw_data(int addr) {
			short high_byte, low_byte, value;
			high_byte = wiringPiI2CReadReg8(fd, addr);
			low_byte = wiringPiI2CReadReg8(fd, addr + 1);
			value = (high_byte << 8) | low_byte;
			return value;
		}

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
			mLoopStartTime(std::chrono::system_clock::now()),
			rxCh1Pin(1),
			rxCh2Pin(2),
			rxCh3Pin(3),
			rxCh4Pin(4),
			ch1PinLastLevel(0),
			ch2PinLastLevel(0),
			ch3PinLastLevel(0),
			ch4PinLastLevel(0)
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
			/*Quad::PID::pidController pid(PID_P_PITCH_GAIN, PID_P_ROLL_GAIN, PID_P_YAW_GAIN, PID_P_ALTITUDE_GAIN,
				PID_I_PITCH_GAIN, PID_I_ROLL_GAIN, PID_I_YAW_GAIN, PID_I_ALTITUDE_GAIN,
				PID_D_PITCH_GAIN, PID_D_ROLL_GAIN, PID_D_YAW_GAIN, PID_D_ALTITUDE_GAIN);*/

			// Create interrupt handler
			// TODO: Find more appropriate location for this code


			// Set GPIO receiver pins to input mode
			/*pinMode(rxCh1Pin, INPUT);
			pinMode(rxCh2Pin, INPUT);
			pinMode(rxCh3Pin, INPUT);
			pinMode(rxCh4Pin, INPUT);*/

			//// Initialize wiringPi and check for errors
			//if (wiringPiSetup() == -1) {
			//	throw std::runtime_error("Error initializing wiringPi!");
			//}

			//// Attach interrupt handlers for each pin
			//if (wiringPiISR(rxCh1Pin, INT_EDGE_BOTH, &Loop::interruptHandlerPin7) < 0) {
			//	throw std::runtime_error("Error initializing wiringPi!");
			//}

			//fd = wiringPiI2CSetup(Device_Address);   /*Initializes I2C with device Address*/
			//MPU6050_Init();

			while (true)
			{
				// Check conditions for preparing quad for flight
				/*if (mReceiverChannel3 < 1050 && mReceiverChannel4 < 1050)
				{
					mQuadState = quadStateEnum::PREPARING_FOR_FLIGHT;
				}*/

				/* Check if state has just transitioned from PREPARING_FOR_FLIGHT to READY_FOR_FLIGHT.
					If so, reset historical PID error values. */
				/*if ((mQuadState == quadStateEnum::PREPARING_FOR_FLIGHT) &&
					(mReceiverChannel3 < 1050) &&
					(mReceiverChannel4 > 1450))
				{
					pid.resetPreviousErrorValues();*/
				//	mQuadState = quadStateEnum::READY_FOR_FLIGHT;
				//}

				// Check conditions for stopping flight, i.e. to move back to STANDBY
				/*if ((mQuadState == quadStateEnum::READY_FOR_FLIGHT) &&
					(mReceiverChannel3 < 1050) &&
					(mReceiverChannel4 > 1950))
				{
					mQuadState = quadStateEnum::STANDBY;
				}*/

				// Determine PWM value for each motor if quad is ready for flight
				//if (quadStateEnum::READY_FOR_FLIGHT == mQuadState)
				//{
					/* Impose upper limit (PWM: 1800) on throttle setting, then determine setpoints for pitch, 
						roll, yaw, and altitude. This is done to ensure that even at the max throttle setting, 
						there is still some (PWM) margin available to stabilise the quad. */

					/*if (mReceiverChannel3 > 1800)
					{
						mReceiverChannel3 = 1800;
					}*/
					//pid.determineSetpoints(mReceiverChannel1, mReceiverChannel2, mReceiverChannel3, mReceiverChannel4);

					// Read gyro values
					/*mpu6050.readIMU_Data(mRawIMU_GyroPitchData, mRawIMU_GyroRollData, mRawIMU_GyroYawData,
						mRawIMU_AccelX_Data, mRawIMU_AccelY_Data, mRawIMU_AccelZ_Data);*/

				mpu6050.readIMU_Data();

						/*Read raw value of Accelerometer and gyroscope from MPU6050*/
				//float Acc_x = read_raw_data(ACCEL_XOUT_H);
				//float Acc_y = read_raw_data(ACCEL_YOUT_H);
				//float Acc_z = read_raw_data(ACCEL_ZOUT_H);

				//float Gyro_x = read_raw_data(GYRO_XOUT_H);
				//float Gyro_y = read_raw_data(GYRO_YOUT_H);
				//float Gyro_z = read_raw_data(GYRO_ZOUT_H);

				///* Divide raw value by sensitivity scale factor */
				//float Ax = Acc_x / 16384.0;
				//float Ay = Acc_y / 16384.0;
				//float Az = Acc_z / 16384.0;

				//float Gx = Gyro_x / 131;
				//float Gy = Gyro_y / 131;
				//float Gz = Gyro_z / 131;

				//printf("\n Gx=%.3f deg/s\tGy=%.3f deg/s\tGz=%.3f deg/s\tAx=%.3f g\tAy=%.3f g\tAz=%.3f g\n", Gx, Gy, Gz, Ax, Ay, Az);
				//delay(500);

					// Filter the IMU data to limit the effect of spikes in the received IMU data
					/*mGyroPitch = (mGyroPitch * IMU_DATA_FILTER) + (mRawIMU_GyroPitchData * IMU_DATA_FILTER);
					mGyroRoll = (mGyroRoll * IMU_DATA_FILTER) + (mRawIMU_GyroRollData * IMU_DATA_FILTER);
					mGyroYaw = (mGyroYaw * IMU_DATA_FILTER) + (mRawIMU_GyroYawData * IMU_DATA_FILTER);

					mAccelX = (mAccelX * IMU_DATA_FILTER) + (mRawIMU_AccelX_Data * IMU_DATA_FILTER);
					mAccelY = (mAccelY * IMU_DATA_FILTER) + (mRawIMU_AccelY_Data * IMU_DATA_FILTER);
					mAccelZ = (mAccelZ * IMU_DATA_FILTER) + (mRawIMU_AccelZ_Data * IMU_DATA_FILTER);*/

		/*			std::cout << "Pitch: " << mRawIMU_GyroPitchData << ", Roll: " << mRawIMU_GyroRollData << ", Yaw: " << mRawIMU_GyroYawData <<
						", Accel X: " << mRawIMU_AccelX_Data << ", Accel Y: " << mRawIMU_AccelY_Data << ", Accel Z: " << mRawIMU_AccelZ_Data << std::endl;*/

					//pid.determineAxisPID_Outputs(mGyroPitch, mGyroRoll, mGyroYaw, mAccelZ);

					// Get control axis outputs
					//float pitch = pid.getPID_PitchOutput();
					//float roll = pid.getPID_RollOutput();
					//float yaw = pid.getPID_YawOutput();
					//float throttle = pid.getPID_ThrottleOutput();

					// Determine PWM signal magnitude for each motor
					// The +/- sign for each axis depends on the orientation of the IMU
					// If positive feedback occurs on a specific axis, need to change the sign for that axis
					//int escFrontRight = throttle + roll - pitch + yaw;
					//int escFrontLeft = throttle - roll - pitch - yaw;
					//int escRearRight = throttle + roll + pitch - yaw;
					//int escRearLeft = throttle - roll + pitch + yaw;

					// Write signals to ESC class

					// Write data to SD card
				//}
				//else if(quadStateEnum::PREPARING_FOR_FLIGHT == mQuadState)
				//{
				//	mPWM_FrontLeft = 1000;
				//	mPWM_FrontRight = 1000;
				//	mPWM_RearLeft = 1000;
				//	mPWM_RearRight = 1000;

				//	// Write signals to ESC class
				//}

				// 8) Loop until 4ms since last loop has expired
				//std::this_thread::sleep_until(mLoopStartTime + std::chrono::milliseconds(10));
				//mLoopStartTime = std::chrono::system_clock::now();
				

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

		void Loop::interruptHandlerPin7(void)
		{
			int pin7State = digitalRead(rxCh1Pin);
			if ((pin7State == 1) && (ch1PinLastLevel == 0))
			{
				ch1PinLastLevel = 1;
				mRxCh1StartTime = std::chrono::system_clock::now();
			}
			else if ((pin7State == 0) && (ch1PinLastLevel == 1))
			{
				ch1PinLastLevel = 0;
				//mReceiverChannel1 = std::chrono::system_clock::now() - mRxCh1StartTime;
			}	
		}

		void Loop::interruptHandlerPin8(void)
		{}

		void Loop::interruptHandlerPin9(void)
		{}

		void Loop::interruptHandlerPin10(void)
		{}

	} // namespace Loop
} // namespace Quad
