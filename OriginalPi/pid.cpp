#include "pid.h"

pidController::pidController(float pGainPitch, float pGainRoll, float pGainYaw,
							 float iGainPitch, float iGainRoll, float iGainYaw,
							 float dGainPitch, float dGainRoll, float dGainYaw) :
	mP_GainPitch(pGainPitch),
	mP_GainRoll(pGainRoll),
	mP_GainYaw(pGainYaw),
	mI_GainPitch(iGainPitch),
	mI_GainRoll(iGainRoll),
	mI_GainYaw(iGainYaw),
	mD_GainPitch(dGainPitch),
	mD_GainRoll(dGainRoll),
	mD_GainYaw(dGainYaw)
{
}

void pidController::resetPreviousErrorValues()
{
}
 