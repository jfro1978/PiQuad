#include <iostream>
#include <wiringPi.h>
#include "loop.h"

int main() {
	try
	{
		Quad::Loop::Loop loop;

		loop.run();
	}
	catch (...)
	{
		// TODO: Add catch statement here
	}

	return 0;
}