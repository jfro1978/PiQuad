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
	float mP_GainPitch;
	float mP_GainRoll;
	float mP_GainYaw;

	float mI_GainPitch;
	float mI_GainRoll;
	float mI_GainYaw;

	float mD_GainPitch;
	float mD_GainRoll;
	float mD_GainYaw;
};

#endif // _PID_CLASS