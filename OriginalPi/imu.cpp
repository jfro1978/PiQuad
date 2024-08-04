#include "imu.h"
#include <iostream>

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
	constexpr uint8_t imuResetMask = 0b10000000; // see register 107 on p40/46

	imuHandle = wiringPiI2CSetup(IMU_address);

	if (imuHandle == -1)
	{
		std::cout << "Failed to open I2C device." << std::endl;
	}

	if (wiringPiI2CWriteReg8(imuHandle, PWR_MGMT_1_REGISTER_ADDRESS, imuResetMask) == -1)
	{
		std::cout << "Failed to write the imuResetMask to the IMU." << std::endl;
	}

	setIMU_MaxRates(gyroFullScale, accelFullScale);

	if (wiringPiI2CWriteReg8(imuHandle, GYRO_CONFIG_REGISTER_ADDRESS, gyroConfigMask) == -1)
	{
		std::cout << "Failed to write the gyro config data to the IMU." << std::endl;
	}

	if (wiringPiI2CWriteReg8(imuHandle, ACCEL_CONFIG_REGISTER_ADDRESS, accelConfigMask) == -1)
	{
		std::cout << "Failed to write the imuResetMask to the IMU." << std::endl;
	}

	calibrateIMU();
}

void intertialMeasurementUnit::calibrateIMU()
{
	// Determine offset when at rest for each axis of the gyro and the accelerometer
	static constexpr float calCycles = 1000.0f;

	float accumulatedGyroPitch, accumulatedGyroRoll, accumulatedGyroYaw = 0;
	short currentGyroPitch, currentGyroRoll, currentGyroYaw = 0;

	float accumulatedAccelX, accumulatedAccelY, accumulatedAccelZ = 0;
	short currentAccelX, currentAccelY, currentAccelZ = 0;

	for (uint16_t counter = 0; counter < calCycles; counter++)
	{
		readIMU_Data(currentGyroPitch, currentGyroRoll, currentGyroYaw,
			currentAccelX, currentAccelY, currentAccelZ);
		accumulatedGyroPitch += currentGyroPitch / GYRO_LSB_VALUE;
		accumulatedGyroRoll += currentGyroRoll / GYRO_LSB_VALUE;
		accumulatedGyroYaw += currentGyroYaw / GYRO_LSB_VALUE;

		accumulatedAccelX += currentAccelX / ACCEL_LSB_VALUE;
		accumulatedAccelY += currentAccelY / ACCEL_LSB_VALUE;
		accumulatedAccelZ = accumulatedAccelZ + (1.0 - (currentAccelZ / ACCEL_LSB_VALUE));

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
}

// Function to read IMU data
void intertialMeasurementUnit::readIMU_Data(short& gyroPitch, short& gyroRoll, short& gyroYaw,
	short& accelX, short& accelY, short& accelZ)
{
	// Code to read gyroscope and accelerometer data via I2C
	short gyroPitchH = wiringPiI2CReadReg8(imuHandle, GYRO_XOUT_H_REGISTER_ADDRESS);
	short gyroPitchL = wiringPiI2CReadReg8(imuHandle, GYRO_XOUT_L_REGISTER_ADDRESS);
	gyroPitch = gyroPitchH << 8 | gyroPitchL;
	gyroPitch = gyroPitch / GYRO_LSB_VALUE;
	gyroPitch = gyroPitch - offsetGyroPitch;

	short gyroRollH = wiringPiI2CReadReg8(imuHandle, GYRO_YOUT_H_REGISTER_ADDRESS);
	short gyroRollL = wiringPiI2CReadReg8(imuHandle, GYRO_YOUT_L_REGISTER_ADDRESS);
	gyroRoll = gyroRollH << 8 | gyroRollL;
	gyroRoll = gyroRoll / GYRO_LSB_VALUE;
	gyroRoll = gyroRoll - offsetGyroRoll;

	short gyroYawH = wiringPiI2CReadReg8(imuHandle, GYRO_ZOUT_H_REGISTER_ADDRESS);
	short gyroYawL = wiringPiI2CReadReg8(imuHandle, GYRO_ZOUT_L_REGISTER_ADDRESS);
	gyroYaw = gyroYawH << 8 | gyroYawL;
	gyroYaw = gyroYaw / GYRO_LSB_VALUE;
	gyroYaw = gyroYaw - offsetGyroYaw;

	short accelX_H = wiringPiI2CReadReg8(imuHandle, ACCEL_XOUT_H_REGISTER_ADDRESS);
	short accelX_L = wiringPiI2CReadReg8(imuHandle, ACCEL_XOUT_L_REGISTER_ADDRESS);
	accelX = accelX_H << 8 | accelX_L;
	accelX = accelX / ACCEL_LSB_VALUE;
	accelX = accelX - offsetAccelX;

	short accelY_H = wiringPiI2CReadReg8(imuHandle, ACCEL_YOUT_H_REGISTER_ADDRESS);
	short accelY_L = wiringPiI2CReadReg8(imuHandle, ACCEL_YOUT_L_REGISTER_ADDRESS);
	accelY = accelY_H << 8 | accelY_L;
	accelY = accelY / ACCEL_LSB_VALUE;
	accelY = accelY - offsetAccelY;

	short accelZ_H = wiringPiI2CReadReg8(imuHandle, ACCEL_ZOUT_H_REGISTER_ADDRESS);
	short accelZ_L = wiringPiI2CReadReg8(imuHandle, ACCEL_ZOUT_L_REGISTER_ADDRESS);
	accelZ = accelZ_H << 8 | accelZ_L;
	accelZ = accelZ / ACCEL_LSB_VALUE;
	accelZ = accelZ - offsetAccelZ;
}

void intertialMeasurementUnit::setIMU_MaxRates(const gyroConfigEnum& gyroCfg, const accelConfigEnum& accelCfg)
{
	switch (gyroCfg)
	{
	case gyroConfigEnum::FS_250_DPS:
	{
		gyroConfigMask = 0b00000000;
		GYRO_LSB_VALUE = 131;
		break;
	}
	case gyroConfigEnum::FS_500_DPS:
	{
		gyroConfigMask = 0b00001000;
		GYRO_LSB_VALUE = 66.5;
		break;
	}
	case gyroConfigEnum::FS_1000_DPS:
	{
		gyroConfigMask = 0b00010000;
		GYRO_LSB_VALUE = 32.8;
		break;
	}
	case gyroConfigEnum::FS_2000_DPS:
	{
		gyroConfigMask = 0b00011000;
		GYRO_LSB_VALUE = 16.4;
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
		accelConfigMask = 0b00000000;
		ACCEL_LSB_VALUE = 16384;
		break;
	}
	case accelConfigEnum::AFS_4_G:
	{
		accelConfigMask = 0b00001000;
		ACCEL_LSB_VALUE = 8192;
		break;
	}
	case accelConfigEnum::AFS_8_G:
	{
		accelConfigMask = 0b00010000;
		ACCEL_LSB_VALUE = 4096;
		break;
	}
	case accelConfigEnum::AFS_16_G:
	{
		accelConfigMask = 0b00011000;
		ACCEL_LSB_VALUE = 2048;
		break;
	}
	default:
	{
		break;
	}
	}
}