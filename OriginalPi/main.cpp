#include <iostream>
#include <wiringPi.h>
#include "imu.h"

int main() {
	try
	{
		intertialMeasurementUnit mpu6050(gyroConfigEnum::FS_250_DPS, accelConfigEnum::AFS_2_G);

		mpu6050.initialiseIMU();
	}
	catch (...)
	{

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

	return 0;
}


