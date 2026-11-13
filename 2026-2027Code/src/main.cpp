#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

int goal = 0;
int stack = 1;
int reallevel = 0;
FILE* logfile = nullptr;
int flushCounter = 0;
bool toggleFlip = false;
bool checkDistance = false;
bool runWhile = false;

void updateRealLevel() {
    reallevel = 800 * stack + (goal * 260);
}

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-1, -2, -6},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({3, 4, 5}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// ARM - 2 motors total
pros::MotorGroup Arm({11, -12}, pros::MotorGearset::red); // left side of arm
// pros::MotorGroup ArmRight({-12}, pros::MotorGearset::red); // right side of arm

pros::MotorGroup SideToggle({-16, 18}, pros::MotorGearset::green);

pros::Motor Claw({-20}, pros::MotorGearset::green);

// Inertial Sensor on port 10
pros::Imu imu(13);
//hi

// Pneumatic claw on ADI Port A
pros::adi::DigitalOut claw('A');

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(9);

// vertical tracking wheel encoder, port 17, reversed
pros::Rotation verticalEnc(-17);
// Distance sensor
pros::Distance GoalDistance(19);
pros::Distance Under(14);
pros::Distance RWallDistance(15);
pros::Distance LWallDistance(10);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 3.5);

// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10.25, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            5, // derivative gain (kD)
                                            0, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             15, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for tis mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen   
    chassis.calibrate(); // calibrate the chassis
    while (imu.is_calibrating()) { // wait for the IMU to finish calibrating
        pros::delay(1000);
    }
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    
    pros::Task screenTask([&]() {
        flushCounter = 0;
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(4, "IMU Heading: %.2f", imu.get_heading());
            pros::lcd::print(5, "Hold: %d \n", Arm.get_brake_mode());
            pros::lcd::print(6, "Wall Distance: %d\n", RWallDistance.get());
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::lcd::print(7, "Goal Distance %d\n", GoalDistance.get());
            // pros::lcd::print(7, "Lift Torque %d\n", Arm.get_current_draw());
            printf("X: %f\n", chassis.getPose().x);
            printf("Y: %f\n", chassis.getPose().y);
            printf("Theta:  %f\n", chassis.getPose().theta);
            printf("Goal Distance %d\n", GoalDistance.get());
            // delay to save resources
            pros::delay(50);            
        }
    });

    SideToggle.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    Claw.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    int stacklevel = 0; // sets the initial value for the stack level
    Arm.tare_position_all(); // sets the lowest position of the arm to __ degrees.

    // Hold is okay at the final position, but for step movements we want
    // the motor to stop cleanly after a move_relative() command.
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  //  ArmRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void clawClose() {
    claw.set_value(true);
}

void clawOpen() {
    claw.set_value(false);
}
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void example_autonomous() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);

    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});

    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();


    // Turn to face the point x:45, y:-45. Timeout set to 1000
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});

    // Turn to face a direction of 90º. Timeout set to 1000
    chassis.turnToPoint(90, 1000, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});

    // Follow the path in path.txt
    chassis.follow(example_txt, 15, 4000, false);

    // wait until the movement is done
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");

    chassis.waitUntilDone();    
    pros::lcd::print(4, "pure pursuit finished!");
}
void driveUntilPin(bool clamp = true, int compensateX = 0, int compensateY = 0) {
    chassis.moveToPose((chassis.getPose().x + (GoalDistance.get()/25.4 + 7) * std::sin(chassis.getPose().theta * M_PI / 180)) + compensateX, (chassis.getPose().y + (GoalDistance.get()/25.4 + 7) * std::cos(chassis.getPose().theta * M_PI / 180)) + compensateY, chassis.getPose().theta, 2000, {.lead = 0.6, .maxSpeed = 50});
    while (chassis.isInMotion()) {
        if (GoalDistance.get() < 120) {
            printf("ENTERED IF CONDITION");
            if (clamp) {
                printf("claw closed");
                clawOpen();
            }
            chassis.cancelMotion();
        }
    }
}
void driveTo(float x, float y, int timeout, lemlib::MoveToPointParams params = {}, bool wait = true) {
    chassis.moveToPoint(x, y, timeout, params);
    if (wait) {
        chassis.waitUntilDone();
    }
}
float headingToPoint(float x, float y) {
    lemlib::Pose pose = chassis.getPose();
    float dx = x-pose.x;
    float dy = y-pose.y;
    return lemlib::radToDeg(atan2(dx, dy));
}
bool turnUntilPin(float x, float y, int timeout, lemlib::TurnToPointParams params = {}, int pinDistance = 0)
{
    float target = headingToPoint(x, y);
    chassis.turnToPoint(x, y, timeout, params);
    printf("Started turn \n");

    while (chassis.isInMotion())
    {
        printf("InMotion \n");
        //pros::lcd::print(5, "Entered InMotion");
        float degreesLeft = fabs(lemlib::angleError(target, chassis.getPose().theta, false));
        printf("DegreesLeft: %2.3f\n", degreesLeft);
        //pros::lcd::print(6, "DegreesLeft: %2.3f\n", degreesLeft);
        if (degreesLeft < 50)
        {
        // pros::lcd::print(7, "Entered DegLeft");
        printf("Entered DegLeft\n");
        
        int distanceToObj = GoalDistance.get();
        printf("REACHED WHILE LOOP, goal dist: %d\n", distanceToObj);
            
        //pros::lcd::print(7, "goal dist: %d\n", distanceToObj);
        if (abs(pinDistance - distanceToObj) < 150)
        {
            //pros::lcd::print(7, "ABT TO CANCEL MOTION: %d\n", distanceToObj);
            printf("FOUND PIN ABT TO CANCEL MOTION: %d\n", distanceToObj);
            chassis.cancelMotion();
            pros::delay(10);
            return true;
        }
        }
        pros::delay(3);
    }
    printf("Exiting InMotion");
    return false;
}   

void moveToPin() {
    pros::lcd::print(5, "entered moveToPin");
    int distanceToObj = GoalDistance.get();
    float d = distanceToObj / 25.4;
    pros::lcd::print(5, "d is %f ",d);
    lemlib::Pose p = chassis.getPose();
    float th = p.theta * (M_PI / 180.0);
    pros::lcd::print(5, "th is %f ",th);
    float StackX = p.x + d * sin(th);
    float StackY = p.y + d * cos(th);

    pros::lcd::print(5, "StackX: %d \n", StackX);
    pros::lcd::print(6, "StackY: %d\n", StackY);
    
    chassis.moveToPoint(StackX, StackY, 3000, {.maxSpeed = 50});
}    
void turnToPin() {
    runWhile = true;
    chassis.turnToPoint(20, 51.75, 1000);
    runWhile = false;
}
void distance(double xC, double yC) {
    while (xC - chassis.getPose().x <= 10 && yC - chassis.getPose().y <= 10) {
        chassis.moveToPoint(xC, yC, 1000, {.maxSpeed = 50});
    }
}
void slowDownWhenClose(double targetX, double targetY) {
    while (GoalDistance.get() > 1500) {
        chassis.moveToPoint(targetX, targetY, 1000, {.maxSpeed = 100});
    }
    while (GoalDistance.get() < 1500) {
        chassis.moveToPoint(targetX, targetY, 1000, {.maxSpeed = 75});
    }
    while (GoalDistance.get() < 1000) {
        chassis.moveToPoint(targetX, targetY, 1000, {.maxSpeed = 5});
    }
}
void clampWhenClose(double TargetX, double TargetY) {
    chassis.moveToPoint(TargetX, TargetY, 1000, {.maxSpeed = 50});
    while (GoalDistance.get() < 10) {
        clawOpen();
        Arm.move_absolute(400, 1000);
    }
}
void closeClawWhenClose() {
    while (checkDistance) {
        if (GoalDistance.get() < 10) {
            clawClose();
        }
        pros::delay(10);
    }
}
void AutonPoseReset(double xDistance, double yDistance) {
    const double headingRadians = imu.get_heading() * (M_PI / 180.0);
    double xCoord = 24 - xDistance * cos(headingRadians);
    double yCoord = 24 - yDistance * sin(headingRadians);
    chassis.setPose(xCoord, yCoord, imu.get_heading());
}
void HALF_AWP_COMP_LEFT() {
   chassis.setPose(0, 70, 90);
    Arm.tare_position_all(); // sets the lowest position of the arm to __ degrees.
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnToHeading(55, 1000);
    SideToggle.move(100);
    pros::delay(135);
    SideToggle.brake();
    chassis.turnToHeading(55, 1000);
    pros::delay(1000);
    SideToggle.move(100);
    pros::delay(105);
    SideToggle.brake();
    // pros::delay(250);
    // pros::delay(500);
    // // Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(-11, 85, 1000, {.forwards = false});
}
void RIGHT_PISTON_SECOND_PIN_HALF_AWP() {
    chassis.setPose(0, 0, 90);
    Arm.tare_position_all();
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    SideToggle.set_zero_position(0);

    SideToggle.move_relative(720, 100);
    pros::delay(600);
    Arm.move_absolute(600, 100);
    // chassis.turnToHeading(45, 1000);
    // chassis.moveToPose(24, 18, 90, 2500, {.horizontalDrift = 8, .minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPose(25, 17, 90, 5000, {.horizontalDrift = 8, .maxSpeed = 100});
    chassis.waitUntilDone();
    Arm.move_absolute(0, 50);
    pros::delay(500);
    clawClose();
    pros::delay(500);
    chassis.moveToPoint(0, 17, 2000, {.forwards = false});
    chassis.waitUntilDone();
    clawClose();
    turnUntilPin(0, 50, 5000, {.maxSpeed = 40}, 640);
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    driveUntilPin(true, -0.75, 0.25);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    chassis.moveToPoint(33, 41,1000);
    Arm.move_absolute(1200, 100);
    chassis.turnToHeading(175, 1000);
    chassis.moveToPose(33, 10, 180, 2000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    Arm.move_absolute(600, 100);
    pros::delay(500);
    clawClose();
    chassis.setPose(24, 28, 180);
}
void LEFT_PISTON_SECOND_PIN_HALF_AWP() {
    chassis.setPose(0, 0, 270);
    Arm.tare_position_all();
    Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    SideToggle.set_zero_position(0);

    SideToggle.move_relative(720, 100);
    pros::delay(600);
    Arm.move_absolute(600, 50);
    // chassis.turnToHeading(45, 1000);
    // chassis.moveToPose(24, 18, 90, 2500, {.horizontalDrift = 8, .minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPose(-25, 17, 270, 4000, {.horizontalDrift = 8, .maxSpeed = 127});
    chassis.waitUntilDone();
    Arm.move_absolute(0, 50);
    pros::delay(500);
    clawClose();
    pros::delay(500);
    chassis.moveToPoint(0, 17, 2000, {.forwards = false});
    chassis.waitUntilDone();
    clawClose();
    turnUntilPin(0, 50, 5000, {.maxSpeed = 40}, 640);
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    driveUntilPin(true, -0.75, -0.25);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    chassis.moveToPoint(-33, 41,1000);
    Arm.move_absolute(1200, 100);
    chassis.turnToHeading(285, 1000);
    chassis.moveToPose(-33, 8, 180, 2000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    Arm.move_absolute(600, 100);
    pros::delay(500);
    clawClose();
    chassis.setPose(-24, 28, 180);
}
void autonomous() {
    LEFT_PISTON_SECOND_PIN_HALF_AWP();

}
// Fixed-position up/down steps for the lift.
// Tune these numbers after testing on the real robot.
const double ARM_STEP_DEGREES = 750.0;
const int ARM_STEP_VELOCITY = 100;
uint32_t armTimeout = 2000; // Timeout in milliseconds

void opcontrol() {
    pros::adi::Pneumatics left_piston('a', false);
    pros::adi::Pneumatics right_piston('b', false, true);
    bool clawOpenState = false;
    bool armStepMoving = false;
    uint32_t armStepStartTime = 0;
    toggleFlip = false;
    int xCoord = 0;
    int yCoord = 0;
    while (true) {
        // =========================
        // Drive
        // =========================
        int goaldistance = GoalDistance.get() / 25.4;
        int walldistance = RWallDistance.get() / 25.4;
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);
        while (runWhile) {
        if (GoalDistance.get() <= 1000) {
            chassis.cancelMotion();
        }
    }

        // =========================
        // Arm
        // L1/L2 = manual hold control
        // Up/Down = one fixed step each press
        // =========================
        bool manualUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool manualDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        bool toggleFlip = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        if (manualUp) {
            Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
            Arm.move_velocity(75);
            armStepMoving = false;
        }
        else if (manualDown) {
            Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
            Arm.move_velocity(-50);
            armStepMoving = false;
        }

        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            Arm.move_relative(ARM_STEP_DEGREES, ARM_STEP_VELOCITY);
           // ArmRight.move_relative(ARM_STEP_DEGREES, ARM_STEP_VELOCITY);
            armStepMoving = true;
            armStepStartTime = pros::millis();
            armTimeout = 2000; // Timeout in milliseconds
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (goal == 1) {
                goal = 0;
            }
            else {
                goal++;
            }
            updateRealLevel();
        }
        // if (Arm.get_torque() >= 10) {
        //     Arm.brake();
        // }
        // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {

        // }
        // else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {                      

        // }
        // else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            // updateRealLevel();
            // if (stack == 4 || stack == 5) {
            //     Arm.move_absolute(625 * stack + (goal * 260), ARM_STEP_VELOCITY);
            //     armStepMoving = true;
            //     armStepStartTime = pros::millis();
            //     armTimeout = 10000; // Timeout in milliseconds   
            // }
            // else{
            // Arm.move_absolute(reallevel, ARM_STEP_VELOCITY);
            // armStepMoving = true;
            // armStepStartTime = pros::millis();
            // armTimeout = 10000; // Timeout in milliseconds
            //             if (stack == 6) {
            //     stack = 0;
            // }
            // else {
            //     stack++;
            // }
            // updateRealLevel();
            // }
                
        // }

        else if (!armStepMoving) {
            Arm.brake();
            Arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        }

 
        // =========================
        // Claw
        // X = Toggle
        // =========================
        bool spinforward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool spinbackward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool clawspinning = false;
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            clawOpenState = !clawOpenState;
            if (clawOpenState) {
                clawClose();
            }
            else {
                clawOpen();
                pros::delay(50);                 
                Arm.move_relative(350.0, ARM_STEP_VELOCITY);
          //      ArmRcight.move_relative(90.0, ARM_STEP_VELOCITY);
                armStepMoving = true;
                armStepStartTime = pros::millis();
                armTimeout = 300; // Timeout in milliseconds
            }
        }

        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        //     Claw.move(100);
        //     if (distance < 10) {
        //         Claw.brake();
        //     }
        //     pros::delay(20);
        // }

        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        //     Claw.move(100);
        //     Claw.brake();
        //     Claw.move(-100);
        //     pros::delay(80);
        //     Claw.brake();
        // }
        // if (spinforward = true) {
        //     clawspinning = true;
        //     Claw.move(100);
        //     if (distance < 10) {
        //     Claw.brake();
        //     clawspinning = false;
        //     }
        //     pros::delay(20);
        // }

        // if (spinbackward = true) {
        //     clawspinning = true;
        //     Claw.move(100);
        //     Claw.brake();
        //     Claw.move(-100);
        //     pros::delay(80);
        //     Claw.brake();
        //     clawspinning = false;
        // }
        // if (clawspinning = false){
        //     Claw.brake();
        //     pros::delay(20);
        // }

        //arm times out after armTimeout milliseconds
        if (armStepMoving && (pros::millis() - armStepStartTime) > armTimeout) {
            armStepMoving = false;
        }

        // if (toggleFlip = true) {
        //     SideToggle.move_velocity(100);
        // }
        else if (controller.get_digital_new_release((pros::E_CONTROLLER_DIGITAL_R1))) {
            SideToggle.move(100);
            pros::delay(150);
            SideToggle.brake();
        }
        pros::delay(10);
    }
}