#ifndef _PID_CLASS
#define _PID_CLASS

namespace Quad
{
	namespace PID
	{
		class pidController {
		public:
			// Constructor and Destructor
			pidController::pidController(float pGainPitch, float pGainRoll, float pGainYaw, float pGainAltitude,
				float iGainPitch, float iGainRoll, float iGainYaw, float iGainAltitude,
				float dGainPitch, float dGainRoll, float dGainYaw, float dGainAltitude);
			pidController::~pidController() = default;

		public:
			void resetPreviousErrorValues();
			void determineSetpoints(unsigned int channel1,
				unsigned int channel2, unsigned int channel3, unsigned int channel4);

			void determineAxisPID_Outputs();

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

			// Controller loop error values
			float mPID_I_Roll_Output_Previous;
			float mPID_Error_Roll_Previous;
			float mPID_I_Pitch_Output_Previous;
			float mPID_Error_Pitch_Previous;
			float mPID_I_Yaw_Output_Previous;
			float mPID_Error_Yaw_Previous;

			// Setpoints
			int mPitchSetpoint;
			int mRollSetpoint;
			int mYawSetpoint;
			unsigned int mAltitudeSetpoint;
		};

	} // namespace PID
} // namespace Quad
#endif // _PID_CLASS