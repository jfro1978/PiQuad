#include "imu.h"
#include "wiringPi.h"
#include <iostream>

namespace Quad
{
	namespace IMU
	{
		// Define the I2C address of the IMU
		constexpr uint8_t IMU_address = 0x68;

		// IMU register addresses
		constexpr uint8_t PWR_MGMT_1 = 0x6B;
		constexpr uint8_t SMPLRT_DIV = 0x19;
		constexpr uint8_t CONFIG = 0x1A;
		constexpr uint8_t GYRO_CONFIG = 0x1B;
		constexpr uint8_t ACCEL_CONFIG = 0x1C;
		constexpr uint8_t INT_ENABLE = 0x38;
		constexpr uint8_t ACCEL_XOUT_H = 0x3B;
		constexpr uint8_t ACCEL_YOUT_H = 0x3D;
		constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
		constexpr uint8_t GYRO_XOUT_H = 0x43;
		constexpr uint8_t GYRO_YOUT_H = 0x45;
		constexpr uint8_t GYRO_ZOUT_H = 0x47;

		intertialMeasurementUnit::intertialMeasurementUnit(const gyroConfigEnum& gyroFS, const accelConfigEnum& accelFS) 
		{
			switch (gyroFS)
			{
			case gyroConfigEnum::FS_250_DPS:
			{
				gyroFullScale = gyroConfigEnum::FS_250_DPS;
				break;
			}
			case gyroConfigEnum::FS_500_DPS:
			{
				gyroFullScale = gyroConfigEnum::FS_500_DPS;
				break;
			}
			case gyroConfigEnum::FS_1000_DPS:
			{
				gyroFullScale = gyroConfigEnum::FS_1000_DPS;
				break;
			}
			case gyroConfigEnum::FS_2000_DPS:
			{
				gyroFullScale = gyroConfigEnum::FS_2000_DPS;
				break;
			}
			default:
			{
				throw std::invalid_argument("Invalid value for gyro full-scale rate.");
			}
			}

			switch (accelFS)
			{
			case accelConfigEnum::AFS_2_G:
			{
				accelFullScale = accelConfigEnum::AFS_2_G;
				break;
			}
			case accelConfigEnum::AFS_4_G:
			{
				accelFullScale = accelConfigEnum::AFS_4_G;
				break;
			}
			case accelConfigEnum::AFS_8_G:
			{
				accelFullScale = accelConfigEnum::AFS_8_G;
				break;
			}
			case accelConfigEnum::AFS_16_G:
			{
				accelFullScale = accelConfigEnum::AFS_16_G;
				break;
			}
			default:
			{
				throw std::invalid_argument("Invalid value for accelerometer full-scale reading.");
			}
			}
		}

		intertialMeasurementUnit::~intertialMeasurementUnit()
		{
			// Close I2C connection
			close(imuHandle);
		}

		void intertialMeasurementUnit::initialiseIMU()
		{
			std::cout << "Starting IMU Initialisation ....." << std::endl;
			setIMU_MaxRates(gyroFullScale, accelFullScale);

			constexpr int imuResetMask = 0x01; // see register 107 on p40/46

			imuHandle = wiringPiI2CSetup(IMU_address);

			if (imuHandle == -1)
			{
				std::cout << "Failed to open I2C device." << std::endl;
			}

			wiringPiI2CWriteReg8(imuHandle, CONFIG, 0);		/* Write to Configuration register */
			wiringPiI2CWriteReg8(imuHandle, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
			wiringPiI2CWriteReg8(imuHandle, PWR_MGMT_1, 0x01);	/* Write to power management register */
			wiringPiI2CWriteReg8(imuHandle, GYRO_CONFIG, gyroConfigMask);	/* Write to Gyro Configuration register */
			wiringPiI2CWriteReg8(imuHandle, ACCEL_CONFIG, accelConfigMask);	/* Write to Accelerometere Configuration register */
			wiringPiI2CWriteReg8(imuHandle, INT_ENABLE, 0x01);	/* Write to interrupt enable register */

			calibrateIMU();
		}

		void intertialMeasurementUnit::calibrateIMU()
		{
			std::cout << "Starting IMU Calibration ....." << std::endl;
			// Determine offset when at rest for each axis of the gyro and the accelerometer
			constexpr uint8_t calCycles = 1000U;

			float accumulatedGyroPitch = 0.0f;
			float accumulatedGyroRoll = 0.0f;
			float accumulatedGyroYaw = 0.0f;

			float currentGyroPitch = 0.0f;
			float currentGyroRoll = 0.0f;
			float currentGyroYaw = 0.0f;

			float accumulatedAccelX = 0.0f;
			float accumulatedAccelY = 0.0f;
			float accumulatedAccelZ = 0.0f;
			
			float currentAccelX = 0.0f;
			float currentAccelY = 0.0f;
			float currentAccelZ = 0.0f;

			for (uint16_t counter = 0; counter < calCycles; counter++)
			{

				currentGyroPitch = read_raw_data(GYRO_XOUT_H) / gyroLSB_Value;
				accumulatedGyroPitch = accumulatedGyroPitch + currentGyroPitch;

				currentGyroRoll = read_raw_data(GYRO_YOUT_H) / gyroLSB_Value;
				accumulatedGyroRoll += currentGyroRoll;

				currentGyroYaw = read_raw_data(GYRO_ZOUT_H) / gyroLSB_Value;
				accumulatedGyroYaw += currentGyroYaw;

				currentAccelX = read_raw_data(ACCEL_XOUT_H) / accelLSB_Value;
				accumulatedAccelX += currentAccelX;

				currentAccelY = read_raw_data(ACCEL_YOUT_H) / accelLSB_Value;
				accumulatedAccelY += currentAccelY;

				currentAccelZ = read_raw_data(ACCEL_ZOUT_H) / accelLSB_Value;
				accumulatedAccelZ = accumulatedAccelZ + (1.0 - currentAccelZ);

				usleep(4000);
			}

			// Determine gyro offsets on each axis: 
			// Divide total number by 'calCycles' to get per sample value, then divide by LSB to get value in deg/sec
			offsetGyroPitch = accumulatedGyroPitch / calCycles;
			offsetGyroRoll = accumulatedGyroRoll / calCycles;
			offsetGyroYaw = accumulatedGyroYaw / calCycles;

			// Determine accelerometer offsets on each axis:
			offsetAccelX = accumulatedAccelX / calCycles;
			offsetAccelY = accumulatedAccelY / calCycles;
			offsetAccelZ = accumulatedAccelZ / calCycles;

			std::cout << "Finished IMU Calibration ....." << std::endl;
		}

		// Function to read IMU data
		void intertialMeasurementUnit::readIMU_Data(float& gyroPitch, float& gyroRoll, float& gyroYaw,
			float& accelX, float& accelY, float& accelZ)
		{
			float Acc_x = read_raw_data(ACCEL_XOUT_H);
			float Acc_y = read_raw_data(ACCEL_YOUT_H);
			float Acc_z = read_raw_data(ACCEL_ZOUT_H);

			float Gyro_x = read_raw_data(GYRO_XOUT_H);
			float Gyro_y = read_raw_data(GYRO_YOUT_H);
			float Gyro_z = read_raw_data(GYRO_ZOUT_H);

			/* Divide raw value by sensitivity scale factor, and subtract offset */
			accelX = (Acc_x / accelLSB_Value) - offsetAccelX;
			accelY = (Acc_y / accelLSB_Value) - offsetAccelY;
			accelZ = (Acc_z / accelLSB_Value) - offsetAccelZ;

			gyroPitch = (Gyro_x / gyroLSB_Value) - offsetGyroPitch;
			gyroRoll = (Gyro_y / gyroLSB_Value) - offsetGyroRoll;
			gyroYaw = (Gyro_z / gyroLSB_Value) - offsetGyroYaw;

		}

		float intertialMeasurementUnit::read_raw_data(int addr) {
			short high_byte, low_byte, value;
			high_byte = wiringPiI2CReadReg8(imuHandle, addr);
			low_byte = wiringPiI2CReadReg8(imuHandle, addr + 1);
			value = (high_byte << 8) | low_byte;
			return value;
		}

		void intertialMeasurementUnit::setIMU_MaxRates(const gyroConfigEnum& gyroCfg, const accelConfigEnum& accelCfg)
		{
			switch (gyroCfg)
			{
			case gyroConfigEnum::FS_250_DPS:
			{
				gyroConfigMask = 0x00;
				gyroLSB_Value = 131;
				break;
			}
			case gyroConfigEnum::FS_500_DPS:
			{
				gyroConfigMask = 0x08; // Corresponds to byte mask of 00001000
				gyroLSB_Value = 66.5;
				break;
			}
			case gyroConfigEnum::FS_1000_DPS:
			{
				gyroConfigMask = 0x10; // Corresponds to byte mask of 00010000
				gyroLSB_Value = 32.8;
				break;
			}
			case gyroConfigEnum::FS_2000_DPS:
			{
				gyroConfigMask = 0x18; // Corresponds to byte mask of 00010000
				gyroLSB_Value = 16.4;
				break;
			}
			default:
			{
				break;
			}
			}

			switch (accelCfg)
			{
			case accelConfigEnum::AFS_2_G:
			{
				accelConfigMask = 0x00;;
				accelLSB_Value = 16384;
				break;
			}
			case accelConfigEnum::AFS_4_G:
			{
				accelConfigMask = 0x08;
				accelLSB_Value = 8192;
				break;
			}
			case accelConfigEnum::AFS_8_G:
			{
				accelConfigMask = 0x10;
				accelLSB_Value = 4096;
				break;
			}
			case accelConfigEnum::AFS_16_G:
			{
				accelConfigMask = 0x18;
				accelLSB_Value = 2048;
				break;
			}
			default:
			{
				break;
			}
			}
		}
	} // namespace IMU
} // namespace Quad