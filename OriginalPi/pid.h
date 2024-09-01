#ifndef _PID_CLASS
#define _PID_CLASS

class pidController {
public:
	// Constructor and Destructor
	pidController::pidController(float pGainPitch, float pGainRoll, float pGainYaw,
		float iGainPitch, float iGainRoll, float iGainYaw, 
		float dGainPitch, float dGainRoll, float dGainYaw);
	pidController::~pidController() = default;

public:
	void resetPreviousErrorValues();

private:
	// Proportional gain values
	float mP_GainPitch;
	float mP_GainRoll;
	float mP_GainYaw;

	// Integral gain values
	float mI_GainPitch;
	float mI_GainRoll;
	float mI_GainYaw;

	// Derivative gain values
	float mD_GainPitch;
	float mD_GainRoll;
	float mD_GainYaw;

	// Controller loop error values
	float mPID_I_Roll_Output_Previous;
	float mPID_Error_Roll_Previous;
	float mPID_I_Pitch_Output_Previous;
	float mPID_Error_Pitch_Previous;
	float mPID_I_Yaw_Output_Previous;
	float mPID_Error_Yaw_Previous;
};

#endif // _PID_CLASS