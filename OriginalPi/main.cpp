#include <iostream>
#include <wiringPi.h>
#include "imu.h"
#include "loop.h"

enum class quadStateEnum 
{
	NOT_STARTED,
	PREPARING_FOR_FLIGHT,
	READY_FOR_FLIGHT
};

int main() {
	try
	{
		intertialMeasurementUnit mpu6050(gyroConfigEnum::FS_250_DPS, accelConfigEnum::AFS_2_G);

		mpu6050.initialiseIMU();

		Loop loop;

		loop.run();
	}
	catch (...)
	{
		// TODO: Add catch statement here
	}

	return 0;
}