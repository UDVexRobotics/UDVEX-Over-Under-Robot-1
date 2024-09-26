#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);



//flap motors
motor FlapMotorA = motor(PORT11, ratio18_1, false);
motor FlapMotorB = motor(PORT12, ratio18_1, false);

//lift motor
motor LLM = motor(PORT4,ratio6_1, false); 
motor RLM = motor(PORT5, ratio6_1, false);

//left motor group
motor leftMotorA = motor(PORT6, ratio18_1, false);
motor leftMotorB = motor(PORT7, ratio18_1, false);
motor leftMotorC = motor(PORT8, ratio18_1, false);
motor_group LeftDriveGroup = motor_group(leftMotorA, leftMotorB, leftMotorC);

//right motor group
motor rightMotorA = motor(PORT10, ratio18_1, true);
motor rightMotorB = motor(PORT2, ratio18_1, true);
motor rightMotorC = motor(PORT3, ratio18_1, true);
motor_group RightDriveGroup = motor_group(rightMotorA, rightMotorB, rightMotorC);

//Intake Motor
motor IntakeMotor = motor(PORT1, ratio18_1, false);


//launcher motor
motor LauncherMotor = motor(PORT9, ratio6_1, false); 
//limit switch
limit LimitSwitch = limit(Brain.ThreeWirePort.A); // Connect the limit switch to port A

// Drivetrain control
distanceUnits units = distanceUnits::in;  //Imperial measurements - inches.
double wheelTravel =  3.25* M_PI;  //Circumference of the drive wheels (4" x PI)
double wheelBase = 8;          //Distince between the center of the front and back axle. 
double gearRatio = 1;           //Ratio of motor rotations to wheel rotations if using gears.
double trackWidth = 11.875; // Distance between the left and right center of wheel.


//Use the following Drivetrain code if NOT using an Inertial Sensor
drivetrain Drivetrain(LeftDriveGroup, RightDriveGroup, wheelTravel, trackWidth, wheelBase, distanceUnits::in );

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
}
