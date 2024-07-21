#include <iostream>
#include <unistd.h> // Needed for closing the I2C connection
#include <wiringPi.h>
#include <wiringPiI2C.h>

// Define the I2C address of the IMU
constexpr uint8_t IMU_address = 0b1101000;
int imuHandle = 0;

// IMU register addresses
constexpr uint8_t GYRO_CONFIG_REGISTER_ADDRESS = 0x1B;
constexpr uint8_t ACCEL_CONFIG_REGISTER_ADDRESS = 0x1C;
constexpr uint8_t PWR_MGMT_1_REGISTER_ADDRESS = 0x6B;

constexpr uint8_t ACCEL_XOUT_H_REGISTER_ADDRESS = 0x3B;
constexpr uint8_t ACCEL_XOUT_L_REGISTER_ADDRESS = 0x3C;

constexpr uint8_t ACCEL_YOUT_H_REGISTER_ADDRESS = 0x3D;
constexpr uint8_t ACCEL_YOUT_L_REGISTER_ADDRESS = 0x3E;

constexpr uint8_t ACCEL_ZOUT_H_REGISTER_ADDRESS = 0x3F;
constexpr uint8_t ACCEL_ZOUT_L_REGISTER_ADDRESS = 0x40;

constexpr uint8_t GYRO_XOUT_H_REGISTER_ADDRESS = 0x43;
constexpr uint8_t GYRO_XOUT_L_REGISTER_ADDRESS = 0x44;

constexpr uint8_t GYRO_YOUT_H_REGISTER_ADDRESS = 0x45;
constexpr uint8_t GYRO_YOUT_L_REGISTER_ADDRESS = 0x46;

constexpr uint8_t GYRO_ZOUT_H_REGISTER_ADDRESS = 0x47;
constexpr uint8_t GYRO_ZOUT_L_REGISTER_ADDRESS = 0x48;

enum gyroConfigEnum
{
	GYRO_FS_NOT_SET,
	FS_250_DPS,
	FS_500_DPS,
	FS_1000_DPS,
	FS_2000_DPS
};
gyroConfigEnum gyroFullScale = gyroConfigEnum::GYRO_FS_NOT_SET;
float GYRO_LSB_VALUE; // See p29/46 of register map document

enum accelConfigEnum
{
	AFS_NOT_SET,
	AFS_2_G,
	AFS_4_G,
	AFS_8_G,
	AFS_16_G,
};
accelConfigEnum accelFullScale = accelConfigEnum::AFS_NOT_SET;
float ACCEL_LSB_VALUE; // See p31/46 of register map document

float offsetGyroPitch, offsetGyroRoll, offsetGyroYaw = 0.0f;
float offsetAccelX, offsetAccelY, offsetAccelZ = 0.0f;

// IMU data masks
uint8_t gyroConfigMask; // See register 27 on p14/46 of register map document
uint8_t accelConfigMask; // See register 28 on p15/46 of register map document

// Function to read IMU data
void readIMU_Data(short& gyroPitch, short& gyroRoll, short& gyroYaw,
	short& accelX, short& accelY, short& accelZ)
{

	// Code to read gyroscope and accelerometer data via I2C
	short gyroPitchH = wiringPiI2CReadReg8(imuHandle, GYRO_XOUT_H_REGISTER_ADDRESS);
	short gyroPitchL = wiringPiI2CReadReg8(imuHandle, GYRO_XOUT_L_REGISTER_ADDRESS);
	gyroPitch = gyroPitchH << 8 | gyroPitchL;

	short gyroRollH = wiringPiI2CReadReg8(imuHandle, GYRO_YOUT_H_REGISTER_ADDRESS);
	short gyroRollL = wiringPiI2CReadReg8(imuHandle, GYRO_YOUT_L_REGISTER_ADDRESS);
	gyroRoll = gyroRollH << 8 | gyroRollL;

	short gyroYawH = wiringPiI2CReadReg8(imuHandle, GYRO_ZOUT_H_REGISTER_ADDRESS);
	short gyroYawL = wiringPiI2CReadReg8(imuHandle, GYRO_ZOUT_L_REGISTER_ADDRESS);
	gyroYaw = gyroYawH << 8 | gyroYawL;

	short accelX_H = wiringPiI2CReadReg8(imuHandle, ACCEL_XOUT_H_REGISTER_ADDRESS);
	short accelX_L = wiringPiI2CReadReg8(imuHandle, ACCEL_XOUT_L_REGISTER_ADDRESS);
	accelX = accelX_H << 8 | accelX_L;

	short accelY_H = wiringPiI2CReadReg8(imuHandle, ACCEL_YOUT_H_REGISTER_ADDRESS);
	short accelY_L = wiringPiI2CReadReg8(imuHandle, ACCEL_YOUT_L_REGISTER_ADDRESS);
	accelY = accelY_H << 8 | accelY_L;

	short accelZ_H = wiringPiI2CReadReg8(imuHandle, ACCEL_ZOUT_H_REGISTER_ADDRESS);
	short accelZ_L = wiringPiI2CReadReg8(imuHandle, ACCEL_ZOUT_L_REGISTER_ADDRESS);
	accelZ = accelZ_H << 8 | accelZ_L;
}

void initialiseIMU()
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

void calibrateIMU()
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

	std::cout << "Gyro pitch offset: " << offsetGyroPitch << std::endl;
	std::cout << "Gyro roll offset: " << offsetGyroRoll << std::endl;
	std::cout << "Gyro yaw offset: " << offsetGyroYaw << std::endl;

	// Determine accelerometer offsets on each axis:
	offsetAccelX = accumulatedAccelX / calCycles;
	offsetAccelY = accumulatedAccelY / calCycles;
	offsetAccelZ = accumulatedAccelZ / calCycles;

	std::cout << "Accel X offset: " << offsetAccelX << std::endl;
	std::cout << "Accel Y offset: " << offsetAccelY << std::endl;
	std::cout << "Accel Z offset: " << offsetAccelZ << std::endl;

}

void setIMU_MaxRates(gyroConfigEnum& gyroCfg, accelConfigEnum& accelCfg)
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


int main() {

	gyroFullScale = gyroConfigEnum::FS_250_DPS;
	accelFullScale = accelConfigEnum::AFS_2_G;
	setIMU_MaxRates(gyroFullScale, accelFullScale);

	initialiseIMU();

	calibrateIMU();

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

	// Close I2C connection
	close(imuHandle);

	return 0;
}


