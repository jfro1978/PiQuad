#include "imu.h"
#include <iostream>

intertialMeasurementUnit::intertialMeasurementUnit()
{

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

	if (wiringPiI2CWriteReg8(imuHandle, GYRO_CONFIG_REGISTER_ADDRESS, gyroConfigMask) == -1)
	{
		std::cout << "Failed to write the gyro config data to the IMU." << std::endl;
	}

	if (wiringPiI2CWriteReg8(imuHandle, ACCEL_CONFIG_REGISTER_ADDRESS, accelConfigMask) == -1)
	{
		std::cout << "Failed to write the imuResetMask to the IMU." << std::endl;
	}
}