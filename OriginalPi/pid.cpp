#include "pid.h"
#include <chrono>

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
			mPID_Error_Yaw_Previous(0.0f),
			mPID_I_Altitude_Output_Previous(0.0f),
			mPID_Error_Altitude_Previous(0.0f),
			mLastCalculationTime(std::chrono::high_resolution_clock::now()),
			mLastLoopAltitude(0.0f),
			mLastLoopVelocity(0.0f),
			mLastZ_AccelValue(0.0f)
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
			mPID_I_Altitude_Output_Previous = 0.0f;
			mPID_Error_Altitude_Previous = 0.0f;
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

		void pidController::determineAxisPID_Outputs(short currentPitch, short currentRoll, short currentYaw,
			short currentZ_accel)
		{
			// Pitch 
			{
				// Calculate error between actual pitch rate and desired pitch rate
				float pitchError = currentPitch - mPitchSetpoint;

				// Calculate Proportional (P) control contribution to overall pitch output
				mPID_P_PitchOutput = pitchError * mP_GainPitch;

				// Calculate Integral (I) control contribution to overall pitch output
				mPID_I_PitchOutput = mPID_I_Pitch_Output_Previous + (pitchError * mI_GainPitch);

				// Calculate Derivative (D) control contribution to overall pitch output
				mPID_D_PitchOutput = (pitchError - mPID_Error_Pitch_Previous) * mD_GainPitch;

				// Calculate overall pitch output
				mPID_PitchOutput = mPID_P_PitchOutput + mPID_I_PitchOutput + mPID_D_PitchOutput;

				/* Can do a check here to ensure the max pitch rate is not exceeded, but probably not needed for now.*/

				// Assign current values to 'previous' members to be used in next loop iteration
				mPID_I_Pitch_Output_Previous = mPID_I_PitchOutput;
				mPID_Error_Pitch_Previous = pitchError;
			}

			// Roll
			{
				// Calculate error between actual roll rate and desired roll rate
				float rollError = currentRoll - mRollSetpoint;

				// Calculate Proportional (P) control contribution to overall roll output
				mPID_P_RollOutput = rollError * mP_GainRoll;

				// Calculate Integral (I) control contribution to overall roll output
				mPID_I_RollOutput = mPID_I_Roll_Output_Previous + (rollError * mI_GainRoll);

				// Calculate Derivative (D) control contribution to overall roll output
				mPID_D_RollOutput = (rollError - mPID_Error_Roll_Previous) * mD_GainRoll;

				// Calculate overall roll output
				mPID_RollOutput = mPID_P_RollOutput + mPID_I_RollOutput + mPID_D_RollOutput;

				/* Can do a check here to ensure the max roll rate is not exceeded, but probably not needed for now.*/

				// Assign current values to 'previous' members to be used in next loop iteration
				mPID_I_Roll_Output_Previous = mPID_I_RollOutput;
				mPID_Error_Roll_Previous = rollError;
			}

			// Yaw 
			{
				// Calculate error between actual yaw rate and desired yaw rate
				float yawError = currentYaw - mYawSetpoint;

				// Calculate Proportional (P) control contribution to overall yaw output
				mPID_P_YawOutput = yawError * mP_GainYaw;

				// Calculate Integral (I) control contribution to overall yaw output
				mPID_I_YawOutput = mPID_I_Yaw_Output_Previous + (yawError * mI_GainYaw);

				// Calculate Derivative (D) control contribution to overall yaw output
				mPID_D_YawOutput = (yawError - mPID_Error_Yaw_Previous) * mD_GainYaw;

				// Calculate overall yaw output
				mPID_YawOutput = mPID_P_YawOutput + mPID_I_YawOutput + mPID_D_YawOutput;

				/* Can do a check here to ensure the max yaw rate is not exceeded, but probably not needed for now.*/

				// Assign current values to 'previous' members to be used in next loop iteration
				mPID_I_Yaw_Output_Previous = mPID_I_YawOutput;
				mPID_Error_Yaw_Previous = yawError;
			}

			// Altitude
			{
				// Integrate to get velocity (area under acceleration v time curve)
				std::chrono::system_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> timeDelta = currentTime - mLastCalculationTime;
				float currentVelocity = mLastLoopVelocity + (timeDelta.count() * currentZ_accel);

				// Determine current altitude using s = s0 + ut + 1/2*a*t^2
				float currentAltitude = mLastLoopAltitude + (currentVelocity * timeDelta.count()) +
					(0.5f * currentZ_accel * timeDelta.count() * timeDelta.count());

				// Set current values to be used as last values in next calculation
				mLastCalculationTime = currentTime;
				mLastLoopAltitude = currentAltitude;
				mLastLoopVelocity = currentVelocity;

				// Determine altitude error
				float altitudeError = currentAltitude - mAltitudeSetpoint;

				// Calculate Proportional (P) control contribution to overall throttle output
				mPID_P_ThrottleOutput = altitudeError * mP_GainAltitude;

				// Calculate Integral (I) control contribution to overall throttle output
				mPID_I_ThrottleOutput = mPID_I_Altitude_Output_Previous + (altitudeError * mI_GainAltitude);

				// Calculate Derivative (D) control contribution to overall throttle output
				mPID_D_ThrottleOutput = (altitudeError - mPID_Error_Altitude_Previous) * mD_GainAltitude;

				// Calculate overall altitude output
				mPID_ThrottleOutput = mPID_P_ThrottleOutput + mPID_I_ThrottleOutput + mPID_D_ThrottleOutput;

				// Assign current values to 'previous' members to be used in next loop iteration
				mPID_I_Altitude_Output_Previous = mPID_I_ThrottleOutput;
				mPID_Error_Altitude_Previous = altitudeError;
			}
		}
		float pidController::getPID_PitchOutput()
		{
			return mPID_PitchOutput;
		}
		float pidController::getPID_RollOutput()
		{
			return mPID_RollOutput;
		}
		float pidController::getPID_YawOutput()
		{
			return mPID_YawOutput;
		}
		float pidController::getPID_ThrottleOutput()
		{
			return mPID_ThrottleOutput;
		}
	} // namespace PID
} // namespace Quad
 