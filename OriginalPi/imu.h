#ifndef _IMU_CLASS
#define _IMU_CLASS

#include <wiringPiI2C.h>

class intertialMeasurementUnit {
public:
	intertialMeasurementUnit::intertialMeasurementUnit();
	intertialMeasurementUnit::~intertialMeasurementUnit() = default;

	void initialiseIMU();
private:


};

#endif // _IMU_CLASS
