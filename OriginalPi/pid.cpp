#include "pid.h"

pidController::pidController(unsigned int pGain, unsigned int iGain, unsigned int dGain) :
	mP_Gain(pGain),
	mI_Gain(iGain),
	mD_Gain(dGain)
{
}
 