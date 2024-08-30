#include <iostream>
#include <wiringPi.h>
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
		Loop loop;

		loop.run();
	}
	catch (...)
	{
		// TODO: Add catch statement here
	}

	return 0;
}