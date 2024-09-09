#include "pid.h"

static const unsigned int MINIMUM_PID_VALUE = 1000;
static const float ALTITUDE_SCALING_FACTOR = 0.01; // Units: metres per increment of PWM signal

namespace Quad
{
	namespace PID
	{
		pidController::pidController(float pGainPitch, float pGainRoll, float pGainYaw, float pGainAltitude,
			float iGainPitch, float iGainRoll, float iGainYaw, float iGainAltitude,
			float dGainPitch, float dGainRoll, float dGainYaw, float dGainAltitude) :
			mP_GainPitch(pGainPitch),
			mP_GainRoll(pGainRoll),
			mP_GainYaw(pGainYaw),
			mP_GainAltitude(pGainAltitude),
			mI_GainPitch(iGainPitch),
			mI_GainRoll(iGainRoll),
			mI_GainYaw(iGainYaw),
			mI_GainAltitude(iGainAltitude),
			mD_GainPitch(dGainPitch),
			mD_GainRoll(dGainRoll),
			mD_GainYaw(dGainYaw),
			mD_GainAltitude(dGainAltitude),
			mPID_I_Roll_Output_Previous(0.0f),
			mPID_Error_Roll_Previous(0.0f),
			mPID_I_Pitch_Output_Previous(0.0f),
			mPID_Error_Pitch_Previous(0.0f),
			mPID_I_Yaw_Output_Previous(0.0f),
			mPID_Error_Yaw_Previous(0.0f)
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
		void pidController::determineSetpoints(unsigned int channel1, unsigned int channel2, unsigned int channel3, unsigned int channel4)
		{
			// Pitch - Channel 1
			mPitchSetpoint = 0;
			if (channel1 > 1508)
			{
				mPitchSetpoint = (channel1 - 1508) / 2;
			}
			else if (channel1 < 1492)
			{
				mPitchSetpoint = (channel1 - 1492) / 2;
			}

			// Roll - Channel 2
			mRollSetpoint = 0;
			if (channel2 > 1508)
			{
				mRollSetpoint = (channel2 - 1508) / 2;
			}
			else if (channel2 < 1492)
			{
				mRollSetpoint = (channel2 - 1492) / 2;
			}

			// Yaw - Channel 4
			mYawSetpoint = 0;
			if (channel4 > 1508)
			{
				mYawSetpoint = (channel4 - 1508) / 2;
			}
			else if (channel4 < 1492)
			{
				mYawSetpoint = (channel4 - 1492) / 2;
			}

			// Altitude - Channel 3
			mAltitudeSetpoint = (channel3 - MINIMUM_PID_VALUE) * ALTITUDE_SCALING_FACTOR;

		}
	} // namespace PID
} // namespace Quad
 