#ifndef _PID_CLASS
#define _PID_CLASS

class pidController {
public:
	pidController::pidController(unsigned int pGain, unsigned int iGain, unsigned int dGain);
	pidController::~pidController() = default;

private:
	unsigned int mP_Gain;
	unsigned int mI_Gain;
	unsigned int mD_Gain;
};

#endif // _PID_CLASS