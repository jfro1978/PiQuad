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
	mPID_I_Roll_Output_Previous = 0.0f;
	mPID_Error_Roll_Previous = 0.0f;
	mPID_I_Pitch_Output_Previous = 0.0f;
	mPID_Error_Pitch_Previous = 0.0f;
	mPID_I_Yaw_Output_Previous = 0.0f;
	mPID_Error_Yaw_Previous = 0.0f;
}
 