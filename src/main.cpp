#include "vex.h"
#include "robot-config.h"
#include <cmath>
#include <algorithm>
using namespace vex;

// A global instance of competition
competition Competition;

//function  prototypes:
void turnRight(double turnDegrees);
void moveToPosition(double position);
double convertFeetToDegrees(double feet);
void toggleflaps();
void toggleLauncher();
double calculatePID(double target);
void updatePositionAndRunPID();


int startTime = 0;
bool startNewAction = false;

const double wheelDiameterInches = 3.25; // Adjusted for new wheel size

//variable to track position of the flaps (extended or not)
bool flapsExtended = false;

//launcher stuff
bool isLauncherResetting = false;
bool launcherActive = false;


const double kP = 0.4;
//PID vatiables
double targetPosition = 0;
double currentPosition = 0;
double lastError = 0;
double totalError = 0;
double pidOutput = 0;
//Acceptable error:


// Function to convert linear distance in feet to encoder degrees
double convertFeetToDegrees(double feet) {
    double wheelCircumferenceInches = wheelDiameterInches * M_PI;
    double travelDistanceInches = feet * 12; // Convert feet to inches
    double degreesPerInch = 360 / wheelCircumferenceInches;
    double degrees = travelDistanceInches * degreesPerInch;
    return degrees;
}

void DowncontrolLift() {
    LLM.spin(directionType::fwd, 25, velocityUnits::pct);
    RLM.spin(directionType::rev, 25, velocityUnits::pct);
}

void UpcontrolLift() {
    LLM.spin(directionType::rev, 25, velocityUnits::pct);
    RLM.spin(directionType::fwd, 25, velocityUnits::pct);
}

// Intake control functions
void intakeIn() {
    IntakeMotor.spin(directionType::fwd, 12.0, voltageUnits::volt); 
}
void intakeOut() {
    IntakeMotor.spin(directionType::rev, 12.0, voltageUnits::volt); 
}
void intakeStop() {
    IntakeMotor.stop(brakeType::brake); // Stop the motor with braking
}
//ENCODER ONLY FUNCTIONS
//moveforward
void moveForward(double feet) {
    double targetDegrees = convertFeetToDegrees(feet);
    LeftDriveGroup.setPosition(0, degrees);
    RightDriveGroup.setPosition(0, degrees);
    while (LeftDriveGroup.position(degrees) < targetDegrees) {
        LeftDriveGroup.spin(forward, 50, percent);
        RightDriveGroup.spin(forward, 50, percent);
    }
    LeftDriveGroup.stop();
    RightDriveGroup.stop();
}

//movebackward
void moveBackward(double feet) {
    double targetDegrees = convertFeetToDegrees(feet);
    LeftDriveGroup.setPosition(0, degrees);
    RightDriveGroup.setPosition(0, degrees);
    while (LeftDriveGroup.position(degrees) > -targetDegrees) {
        LeftDriveGroup.spin(reverse, 50, percent);
        RightDriveGroup.spin(reverse, 50, percent);
    }
    LeftDriveGroup.stop();
    RightDriveGroup.stop();
}
//Encoder Left turn
void turnLeftEncoder(double degrees) {
    double targetTurnDegrees = (degrees * trackWidth) / wheelDiameterInches;
    
    LeftDriveGroup.setPosition(0, vex::rotationUnits::deg);
    RightDriveGroup.setPosition(0, vex::rotationUnits::deg);

    while (std::abs(LeftDriveGroup.position(vex::rotationUnits::deg)) < targetTurnDegrees) {
        LeftDriveGroup.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        RightDriveGroup.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
    }

    LeftDriveGroup.stop();
    RightDriveGroup.stop();
}
//Enoder right turn
void turnRightEncoder(double degrees) {
    double targetTurnDegrees = (degrees * trackWidth) / wheelDiameterInches;
    
    LeftDriveGroup.setPosition(0, vex::rotationUnits::deg);
    RightDriveGroup.setPosition(0, vex::rotationUnits::deg);

    while (std::abs(RightDriveGroup.position(vex::rotationUnits::deg)) < targetTurnDegrees) {
        LeftDriveGroup.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        RightDriveGroup.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    }

    LeftDriveGroup.stop();
    RightDriveGroup.stop();
}

//turn right
void turnRight(double turnDegrees) {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(1, 40, "Starting turnRight function");

    // Reset encoders
    LeftDriveGroup.setPosition(0, degrees);
    RightDriveGroup.setPosition(0, degrees);
    
    // Calculate the target encoder degree for the desired turn
    double targetTurnDegrees = (turnDegrees * trackWidth) / wheelDiameterInches;


    Brain.Screen.printAt(1, 80, "Target Degrees: %f", targetTurnDegrees);

    // Start turning
    LeftDriveGroup.spin(fwd, 50, velocityUnits::pct);
    RightDriveGroup.spin(reverse, 50, velocityUnits::pct);

    // Wait until the turn is complete
    while(LeftDriveGroup.position(degrees) < targetTurnDegrees) {
        Brain.Screen.printAt(1, 120, "Current Degrees: %f", LeftDriveGroup.position(degrees));
        task::sleep(10);
    }
    
    // Stop the motors after the turn
    LeftDriveGroup.stop();
    RightDriveGroup.stop();
    Brain.Screen.printAt(1, 160, "Turn completed");
}
//Turn left
void turnLeft(double turnDegrees) {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(1, 40, "Starting turnLeft function");

    // Reset encoders
    LeftDriveGroup.setPosition(0, degrees);
    RightDriveGroup.setPosition(0, degrees);

    // Calculate the target encoder degree for the desired turn
    double targetTurnDegrees = (turnDegrees * trackWidth) / wheelDiameterInches;

    Brain.Screen.printAt(1, 80, "Target Degrees: %f", targetTurnDegrees);

    // Start turning
    LeftDriveGroup.spin(reverse, 50, velocityUnits::pct);
    RightDriveGroup.spin(fwd, 50, velocityUnits::pct);

    // Wait until the turn is complete
    while (RightDriveGroup.position(degrees) < targetTurnDegrees) {
        Brain.Screen.printAt(1, 120, "Current Degrees: %f", RightDriveGroup.position(degrees));
        task::sleep(10);
    }

    // Stop the motors after the turn
    LeftDriveGroup.stop();
    RightDriveGroup.stop();
    Brain.Screen.printAt(1, 160, "Turn completed");
}


void checkAndLaunch() {
    const int timeout = 3000; // Timeout in milliseconds
    static int startTime = 0; 
    static bool launchInitiated = false; // Flag to track if the launch sequence has been initiated

    // Check if Button B is pressed and the launch sequence has not been initiated
    if (Controller2.ButtonB.pressing() && !launchInitiated) {
        LauncherMotor.rotateFor(directionType::rev, 70, rotationUnits::deg);
        launchInitiated = true; // Set the flag to indicate that the launch sequence has started
        startTime = Brain.Timer.time(msec); // Record the start time

        // Start the launch sequence
        if (LimitSwitch.pressing()) {
            LauncherMotor.rotateFor(5, vex::rotationUnits::deg, 50, vex::velocityUnits::pct, true);
        } else {
            LauncherMotor.spin(directionType::rev, 50, velocityUnits::pct);
        }
    }

    // Check if the LimitSwitch is pressed during the launch sequence
    if (launchInitiated && LimitSwitch.pressing()) {
        LauncherMotor.stop(); // Stop the motor if the LimitSwitch is pressed
        launchInitiated = false; // Reset the flag for the next launch
    }

    // Timeout check
    if (launchInitiated && (Brain.Timer.time(msec) - startTime) > timeout) {
        LauncherMotor.stop(); // Stop the motor if the timeout is reached
        launchInitiated = false; // Reset the flag for the next launch
    }
}

//autonomous launch function
bool ResetBall() {
    bool reset = false;
    LauncherMotor.spin(directionType::rev, 100, velocityUnits::pct); // Spin the motor in reverse
    while (!LimitSwitch.pressing()) { // Wait until the limit switch is pressed
        task::sleep(10); // Small delay to prevent hogging CPU
    }
    LauncherMotor.stop(); // Stop the motor once the limit switch is pressed
    reset = true;
    return reset;
}






// Function to toggle Flap A with timeout
void toggleFlapA() {
    static bool flapAExtended = false;
    int timeout = 500; // Timeout in milliseconds (half a second)
    int startTime = Brain.Timer.time(msec); // Get the start time

    if (flapAExtended) {
        FlapMotorA.startRotateTo(0, degrees, 50, velocityUnits::pct);
    } else {
        FlapMotorA.startRotateTo(-90, degrees, 50, velocityUnits::pct);
    }

    // Wait until the motor reaches the target position or timeout occurs
    while (!FlapMotorA.isDone() && (Brain.Timer.time(msec) - startTime) < timeout) {
        task::sleep(10);
    }

    FlapMotorA.stop(brakeType::hold); // Stop the motor and hold position
    flapAExtended = !flapAExtended; // Toggle the state
}
// Function to toggle Flap B with timeout
void toggleFlapB() {
    static bool flapBExtended = false;
    int timeout = 500; // Timeout in milliseconds (half a second)
    int startTime = Brain.Timer.time(msec); // Get the start time

    if (flapBExtended) {
        FlapMotorB.startRotateTo(0, degrees, 50, velocityUnits::pct);
    } else {
        FlapMotorB.startRotateTo(90, degrees, 50, velocityUnits::pct);
    }

    // Wait until the motor reaches the target position or timeout occurs
    while (!FlapMotorB.isDone() && (Brain.Timer.time(msec) - startTime) < timeout) {
        task::sleep(10);
    }

    FlapMotorB.stop(brakeType::hold); // Stop the motor and hold position
    flapBExtended = !flapBExtended; // Toggle the state
}
//function to calculate PID output:
double calculatePID(double target) {
    double error = target - currentPosition; // Error = Target - Actual
    return kP * error; // Proportional output   
}
//function to update the position of the PID and run the PID
void updatePositionAndRunPID() {
    currentPosition = (LeftDriveGroup.position(degrees) + RightDriveGroup.position(degrees)) / 2;
    pidOutput = calculatePID(targetPosition);

    // Use pidOutput to control motors
    LeftDriveGroup.spin(fwd, pidOutput, voltageUnits::volt);
    RightDriveGroup.spin(fwd, pidOutput, voltageUnits::volt);
}

/*
void skidSteer(int leftSpeedPct, int rightSpeedPct, double distanceInFeet, int timeoutInMilliseconds, directionType direction) {
    double targetDegrees = convertFeetToDegrees(distanceInFeet);
    LeftDriveGroup.resetRotation();
    RightDriveGroup.resetRotation();

    LeftDriveGroup.spin(direction, leftSpeedPct, velocityUnits::pct);
    RightDriveGroup.spin(direction, rightSpeedPct, velocityUnits::pct);

    int startTime = Brain.Timer.time(msec);

    while (std::abs(LeftDriveGroup.rotation(rotationUnits::deg)) < targetDegrees &&
           std::abs(RightDriveGroup.rotation(rotationUnits::deg)) < targetDegrees) {
        if (Brain.Timer.time(msec) - startTime > timeoutInMilliseconds) {
            break; 
        }
        task::sleep(10);
    }

    LeftDriveGroup.stop();
    RightDriveGroup.stop();
}
*/
//P-control skid-steer
void toggleLauncher() {
    if (!launcherActive) {
        LauncherMotor.spin(directionType::rev, 12.0, voltageUnits::volt); // Spin counter-clockwise at 7 volts
        launcherActive = true;
        isLauncherResetting = true; // Start the resetting process
        startTime = Brain.Timer.time(msec); // Record the start time
        Controller1.Screen.clearLine(1);
        Controller1.Screen.print("Launcher activated");
    } else if (isLauncherResetting) {
        // Check if the reset process is complete
        if (LimitSwitch.pressing() || Brain.Timer.time(msec) - startTime > 3000) {
            LauncherMotor.stop(); // Stop the motor once the limit switch is pressed or timeout occurs
            launcherActive = false;
            isLauncherResetting = false; // Resetting is complete
            Controller1.Screen.clearLine(2);
            Controller1.Screen.print("Launcher reset complete");
        }
    }
}




//manual launcher controls
void manualLauncherControl() {
    if (!isLauncherResetting) {
        if (Controller1.ButtonA.pressing()) {
            LauncherMotor.spin(directionType::rev, 12.0, voltageUnits::volt); // Spin counter-clockwise
        } else {
            LauncherMotor.stop(); // Stop the motor once the button is released
        }
    }
}

//skid steer
void skidSteer(int leftSpeedPct, int rightSpeedPct, double distanceInFeet, int timeoutInMilliseconds, directionType direction) {
    double targetDegrees = convertFeetToDegrees(distanceInFeet);
    LeftDriveGroup.resetRotation();
    RightDriveGroup.resetRotation();

    int startTime = Brain.Timer.time(msec);
    int minPower = 10;

    while (std::abs(LeftDriveGroup.rotation(rotationUnits::deg)) < targetDegrees &&
           std::abs(RightDriveGroup.rotation(rotationUnits::deg)) < targetDegrees) {
        double leftError = targetDegrees - std::abs(LeftDriveGroup.rotation(rotationUnits::deg));
        double rightError = targetDegrees - std::abs(RightDriveGroup.rotation(rotationUnits::deg));

        int leftPower = std::max(std::min(static_cast<int>(kP * leftError), leftSpeedPct), minPower);
        int rightPower = std::max(std::min(static_cast<int>(kP * rightError), rightSpeedPct), minPower);

        if (direction == directionType::rev) {
            leftPower = -leftPower;
            rightPower = -rightPower;
        }

        LeftDriveGroup.spin(directionType::fwd, leftPower, velocityUnits::pct);
        RightDriveGroup.spin(directionType::fwd, rightPower, velocityUnits::pct);

        if (Brain.Timer.time(msec) - startTime > timeoutInMilliseconds) {
            break;
        }
        task::sleep(10);
    }

    LeftDriveGroup.stop();
    RightDriveGroup.stop();
}

void launchball(){
  LauncherMotor.rotateFor(directionType::rev, 15, rotationUnits::deg);
}


void printDistanceToControllers() {
    double leftDistance = LeftDriveGroup.rotation(rotationUnits::deg) / 360.0 * (3.25 * M_PI) / 12.0;
    double rightDistance = RightDriveGroup.rotation(rotationUnits::deg) / 360.0 * (3.25 * M_PI) / 12.0;
    double averageDistance = (leftDistance + rightDistance) / 2.0;

    // Print individual distances on Controller1
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("L: %.2f ft", leftDistance);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("R: %.2f ft", rightDistance);

    // Print average distance on Controller2
    Controller2.Screen.clearLine(1);
    Controller2.Screen.setCursor(1, 1);
    Controller2.Screen.print("Avg: %.2f ft", averageDistance);
}

// Pre Autonomous Functions
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  LeftDriveGroup.setPosition(0, degrees);
  RightDriveGroup.setPosition(0, degrees);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void autonomous(void){
    
    skidSteer(70, 70, 4,10000, directionType::fwd);

    skidSteer(30,50, 4.65,10000, directionType::fwd);
    
    skidSteer(50,50, .3,10000, directionType::rev);

    skidSteer(50,50,1.15, 10000, directionType::fwd);

    //perform lineup
    skidSteer(10, 60, 1, 10000, directionType::rev);

    skidSteer(90,10, 1.25, 10000, directionType::rev);

    intakeIn();
    
    skidSteer(50,50,1.25, 10000, directionType::rev);

    skidSteer(50, 50, 1, 10000, directionType::fwd);

    intakeStop();
    
    turnLeftEncoder(90);
    
    intakeOut();

    wait(500, msec);

    intakeStop();

    turnRight(90);

    skidSteer(10,90,2.5,10000, directionType::rev);

    wait(15, sec);

    skidSteer(40,80, 3.5, 10000, directionType::fwd);

    skidSteer(35,50, 4, 10000, directionType::rev);


    


}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
//user control regular
/*
void usercontrol(void) {
  const int deadband = 5; // Deadband threshold for joysticks

  while (1) {
    // Get position of the joysticks
    int leftJoystick = Controller1.Axis3.position();
    int rightJoystick = Controller1.Axis2.position();

    // Flap control with Controller2
    if (Controller1.ButtonL1.pressing()) {
        toggleFlapA();
        while(Controller1.ButtonL1.pressing()) { // Wait for button release
            task::sleep(5);
        }
    }
    if (Controller1.ButtonR1.pressing()) {
        toggleFlapB();
        while(Controller1.ButtonR1.pressing()) { // Wait for button release
            task::sleep(5);
        }
    }

    // Apply deadband for precision control
    if (abs(leftJoystick) < deadband) {
      LeftDriveGroup.stop();
      leftJoystick = 0;
    } else {
      double leftVoltage = leftJoystick / 127.0 * 12.0; // Assuming joystick range is -127 to 127
      LeftDriveGroup.spin(forward, leftVoltage, voltageUnits::volt);
    }
    
    if (abs(rightJoystick) < deadband) {
      RightDriveGroup.stop();
      rightJoystick = 0;
    } else {
      double rightVoltage = rightJoystick / 127.0 * 12.0; // Adjusted for potential full range of joystick
      RightDriveGroup.spin(forward, rightVoltage, voltageUnits::volt);
    }


    // Intake control with R1 and R2 buttons
    if (Controller2.ButtonL1.pressing()) {
        intakeIn(); // Activates intake
    } else if (Controller2.ButtonR1.pressing()) {
        intakeOut(); // Activates outtake
    } else {
        intakeStop(); // Stops intake motor
    }

    //Launcher controls
    if (Controller2.ButtonB.pressing()) {
      toggleLauncher();
      while (Controller2.ButtonB.pressing()) { // Wait for button release
          task::sleep(5);
        }
    }

    if (Controller2.ButtonA.pressing()) {
      manualLauncherControl();
    }

    // Sleep the task for a short amount of time to prevent wasted resources.
    wait(20, msec);
  }
}
*/

void usercontrol(void) {
    const int deadband = 5; // Deadband threshold for joysticks

    while (1) {
        // Get position of the joysticks
        int forwardBackward = Controller1.Axis3.position(); // Left joystick vertical axis
        int turn = Controller1.Axis1.position(); // Right joystick horizontal axis

        // Flap control
        if (Controller1.ButtonL1.pressing()) {
            toggleFlapA();
            while(Controller1.ButtonL1.pressing()) { // Wait for button release
                task::sleep(5);
            }
        }
        if (Controller1.ButtonR1.pressing()) {
            toggleFlapB();
            while(Controller1.ButtonR1.pressing()) { // Wait for button release
                task::sleep(5);
            }
        }

        if (Controller2.ButtonB.pressing()) {
            LauncherMotor.rotateFor(directionType::rev, 50, rotationUnits::deg);
            toggleLauncher();
            startNewAction = true;
        }

        manualLauncherControl();

        // Apply deadband for precision control
        if (abs(forwardBackward) < deadband) {
            forwardBackward = 0;
        }
        if (abs(turn) < deadband) {
            turn = 0;
        }

        // Calculate motor speeds
        int leftMotorSpeed = forwardBackward + turn;
        int rightMotorSpeed = forwardBackward - turn;

        // Set motor speeds
        LeftDriveGroup.spin(forward, leftMotorSpeed, voltageUnits::volt);
        RightDriveGroup.spin(forward, rightMotorSpeed, voltageUnits::volt);

        // Lift control with Controller 2 left joystick
        int liftJoystick = Controller2.Axis3.position(); // Left joystick vertical axis on Controller 2
        if (abs(liftJoystick) > deadband) {
            LLM.spin(forward, liftJoystick, percent);
            RLM.spin(reverse, liftJoystick, percent);
        } else {
            LLM.stop(brakeType::hold);
            RLM.stop(brakeType::hold);
        }

        // Intake control with R1 and R2 buttons
        if (Controller2.ButtonL1.pressing()) {
            intakeIn(); // Activates intake
        } else if (Controller2.ButtonR1.pressing()) {
            intakeOut(); // Activates outtake
        } else {
            intakeStop(); // Stops intake motor
        }



        // Sleep the task for a short amount of time to prevent wasted resources.
        wait(20, msec);
    }
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Main Task                                    */
/*                                                                           */
/*    This task is used to setup the competition functions and callbacks     */
/*                                                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/
// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
