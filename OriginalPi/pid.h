#ifndef _PID_CLASS
#define _PID_CLASS

#include <chrono>

namespace Quad
{
	namespace PID
	{
		class pidController {
		public:
			// Constructor and Destructor
			pidController(float pGainPitch, float pGainRoll, float pGainYaw, float pGainAltitude,
				float iGainPitch, float iGainRoll, float iGainYaw, float iGainAltitude,
				float dGainPitch, float dGainRoll, float dGainYaw, float dGainAltitude);
			~pidController() = default;

		public:
			void resetPreviousErrorValues();
			void determineSetpoints(unsigned int channel1,
				unsigned int channel2, unsigned int channel3, unsigned int channel4);

			void determineAxisPID_Outputs(short currentPitch, short currentRoll, short currentYaw, 
				short currentZ_accel);

			float getPID_PitchOutput();
			float getPID_RollOutput();
			float getPID_YawOutput();
			float getPID_ThrottleOutput();

		private:
			// Proportional gain values
			float mP_GainPitch;
			float mP_GainRoll;
			float mP_GainYaw;
			float mP_GainAltitude;

			// Integral gain values
			float mI_GainPitch;
			float mI_GainRoll;
			float mI_GainYaw;
			float mI_GainAltitude;

			// Derivative gain values
			float mD_GainPitch;
			float mD_GainRoll;
			float mD_GainYaw;
			float mD_GainAltitude;

			// Previous controller loop error values
			float mPID_I_Pitch_Output_Previous;
			float mPID_Error_Pitch_Previous;
			float mPID_I_Roll_Output_Previous;
			float mPID_Error_Roll_Previous;
			float mPID_I_Yaw_Output_Previous;
			float mPID_Error_Yaw_Previous;
			float mPID_I_Altitude_Output_Previous;
			float mPID_Error_Altitude_Previous;

			// Setpoints
			int mPitchSetpoint;
			int mRollSetpoint;
			int mYawSetpoint;
			unsigned int mAltitudeSetpoint;

			// P, I, and D contributions to each axis control signal
			float mPID_P_PitchOutput;
			float mPID_I_PitchOutput;
			float mPID_D_PitchOutput;
			float mPID_PitchOutput;

			float mPID_P_RollOutput;
			float mPID_I_RollOutput;
			float mPID_D_RollOutput;
			float mPID_RollOutput;

			float mPID_P_YawOutput;
			float mPID_I_YawOutput;
			float mPID_D_YawOutput;
			float mPID_YawOutput;

			float mPID_P_ThrottleOutput;
			float mPID_I_ThrottleOutput;
			float mPID_D_ThrottleOutput;
			float mPID_ThrottleOutput;
			std::chrono::system_clock::time_point mLastCalculationTime;
			float mLastLoopAltitude;
			float mLastLoopVelocity;
			float mLastZ_AccelValue;
		};

	} // namespace PID
} // namespace Quad
#endif // _PID_CLASS