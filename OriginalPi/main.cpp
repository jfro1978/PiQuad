#include <iostream>
#include <wiringPi.h>
#include "imu.h"

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
	}
	catch (...)
	{
		// TODO: Add catch statement here
	}

	while (1)
	{
		/*
		1) Read gyro

		2) Filter the gyro data using 50/50 filter, i.e. previous (filtered) value contributes 80% to current value, current (raw) value contributes 20% to current (filtered) value

		3) Check conditions for getting quad ready for flight, i.e. changing state of quadStateEnum variable
			if(receiver_channel_3 < 1050 && receiver_channel_4 < 1050)
			  {
				start = 1;
			  }

		4) Check conditions for moving to READY_FOR_FLIGHT. This is simply to check if in PREPARING_FOR_FLIGHT, if so reset previous PID error values, then set state to READY_FOR_FLIGHT
				    pid_i_roll_output_prev = 0;
					pid_error_roll_prev = 0;
					pid_i_pitch_output_prev = 0;
					pid_error_pitch_prev = 0;
					pid_i_yaw_output_prev = 0;
					pid_error_yaw_prev = 0;    


		5) Check conditions for stopping flight, i.e. to move back to NOT_STARTED
				if(start == 2 && receiver_channel_3 < 1050 && receiver_channel_4 > 1950)
				  {
					start = 0;
				  }

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

	return 0;
}


