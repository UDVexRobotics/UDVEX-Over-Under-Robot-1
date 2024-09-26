using namespace vex;

extern brain Brain;

//VEXcode devices
extern controller Controller1;
extern controller Controller2;

extern motor_group LeftDriveGroup;
extern motor_group RightDriveGroup;

// Declare flap motors
extern motor FlapMotorA;
extern motor FlapMotorB;

// Declare launher motor
extern motor LauncherMotor;
extern limit LimitSwitch;
extern double trackWidth;

extern motor IntakeMotor;
extern motor LLM;
extern motor RLM;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

