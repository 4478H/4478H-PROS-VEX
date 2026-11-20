#include "main.h"
#include "devices.h"
#include "auton_selector.h"
#include "liftControl.h"
#include <algorithm>
#include <cmath>

using namespace pros;
using namespace lemlib;

bool colorSortEnabled = true; // color sorting is on by default

// Brain left button: previous auton route
void onLeft_button()
{
    competitionSelector.prevSelection();
    competitionSelector.displaySelectionBrain();
}

// Brain right button: next auton route
void onRight_button()
{
    competitionSelector.nextSelection();
    competitionSelector.displaySelectionBrain();
}

// Brain center button: switch alliance color
void onCenter_button()
{
    red = !red;
    competitionSelector.displaySelectionBrain();
}

/**
 * Runs as soon as the program starts. All other competition modes wait for
 * this to finish, so keep it under a few seconds.
 */
void initialize()
{
    wing.set_value(LOW);
    clamp.set_value(HIGH); // start with the clamp open
    lcd::initialize();
    chassis.calibrate();
    pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);

    competitionSelector.displaySelectionBrain();
    lcd::register_btn0_cb(onLeft_button);
    lcd::register_btn1_cb(onCenter_button);
    lcd::register_btn2_cb(onRight_button);

    // Background task: show the robot's position on the brain screen
    pros::Task screen_task([]() {
        while (true) {
            pros::lcd::print(3, "LemLib X:%.1f Y:%.1f", chassis.getPose().x, chassis.getPose().y);
            // pros::lcd::print(4, "Lift: %.1f deg", getLiftPosition());
            pros::lcd::print(5, "Heading: %.1f", chassis.getPose().theta);
            pros::delay(100);
        }
    });

    // Limit switch selector is off for now, the brain's left/right buttons pick the route instead
    // Background task: each press of the limit switch moves to the next auton route
    // pros::Task auton_select_task([]() {
    //     bool lastPressed = false;
    //     while (true) {
    //         bool pressed = autonLimitSwitch.get_value();
    //         if (pressed && !lastPressed) {
    //             competitionSelector.nextSelection();
    //             competitionSelector.displaySelectionBrain();
    //         }
    //         lastPressed = pressed;
    //         pros::delay(100);
    //     }
    // });
}

/**
 * Runs the selected autonomous route. Started in its own task when the
 * robot is enabled in autonomous mode. If the robot is disabled, the task is
 * stopped and re-enabling starts it over from the beginning.
 */
void autonomous()
{
    controller.clear();
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

    // Background task: show the robot's position on the controller
    pros::Task display_task([]() {
        while (true) {
            controller.print(0, 0, "Heading: %.1f", fmod(chassis.getPose().theta, 360.0));
            controller.print(2, 0, "LemLib X:%.1f", chassis.getPose().x);
            controller.print(4, 0, "LemLib Y:%.1f", chassis.getPose().y);
            pros::delay(100);
        }
    });

    competitionSelector.runSelection();

    // Hold still for a moment once the route is done, then let the motors coast
    all_motors.brake();
    delay(2000);
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

/**
 * Runs while the robot is disabled by the field or competition switch.
 * Exits when the robot is enabled.
 */
void disabled() {}

/**
 * Runs after initialize() and before autonomous when connected to the
 * field or a competition switch.
 */
void competition_initialize()
{
    competitionSelector.displaySelectionBrain();
    lcd::register_btn0_cb(onLeft_button);
    lcd::register_btn1_cb(onCenter_button);
    lcd::register_btn2_cb(onRight_button);
}

// ---------------- Drive curve ----------------
// Gives finer control at low joystick values while keeping full power at the top
const double SMOOTHING_DENOMINATOR = 220; // scales the curve back into range
const double EXPONENTIAL_POWER = 2.2;     // how aggressive the curve is

double logDriveJoystick(double joystickPCT)
{
    double magnitude = fabs(joystickPCT);
    double smoothedValue = pow(magnitude, EXPONENTIAL_POWER) / SMOOTHING_DENOMINATOR;
    return joystickPCT >= 0 ? smoothedValue : -smoothedValue; // put the sign back
}

// ---------------- Precision mode ----------------
// Press Y to toggle slower, smoother driving for lining up
bool precisionMode = false;
const double PRECISION_SPEED_SCALE = 0.4; // max speed in precision mode (40% of normal)
const double PRECISION_MAX_ACCEL = 15.0;  // max rpm change per 20 ms loop in precision mode

// Limits how fast a side's velocity can change so the robot doesn't jerk
// previous is updated to the new value
double limitAccel(double target, double &previous, double maxStep)
{
    double change = std::clamp(target - previous, -maxStep, maxStep);
    previous += change;
    return previous;
}

void handlePrecisionMode()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        precisionMode = !precisionMode;
        controller.rumble(precisionMode ? "." : "-"); // short = on, long = off
    }
}

// ---------------- Lift limiting ----------------
// Slows and smooths the drive as the lift goes up so the top heavy robot
// doesn't tip or jerk. Set LIFT_LIMIT_ENABLED to false to turn it off for good
const bool LIFT_LIMIT_ENABLED = true;
const double LIFT_LIMIT_START = 300;     // lift height (encoder deg) where limiting starts
const double LIFT_LIMIT_FULL = 700;      // lift height where limiting is strongest
const double LIFT_MIN_SPEED_SCALE = 0.5; // max speed at full height (50% of normal)
const double LIFT_START_ACCEL = 100.0;    // max rpm change per 20 ms loop at start height
const double LIFT_MIN_ACCEL = 40.0;      // max rpm change per 20 ms loop at full height
const double LIFT_TURN_SCALE = 1;      // turning strength at full height (80% of normal)

// Press DOWN to toggle lift limiting on and off during a match
bool liftLimitOn = true;

void handleLiftLimitToggle()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        liftLimitOn = !liftLimitOn;
        controller.rumble(liftLimitOn ? ".." : "--"); // two short = on, two long = off
    }
}

// Returns 0 when the lift is low, 1 at full height, and in between as it rises
double liftLimitAmount()
{
    if (!LIFT_LIMIT_ENABLED || !liftLimitOn) return 0;
    double amount = (getLiftPosition() - LIFT_LIMIT_START) / (LIFT_LIMIT_FULL - LIFT_LIMIT_START);
    return std::clamp(amount, 0.0, 1.0);
}

// Softens turning, top speed, and acceleration based on how high the lift is
void applyLiftLimit(double &left, double &right)
{
    static double liftPrevLeft = 0;
    static double liftPrevRight = 0;

    double t = liftLimitAmount();
    if (t == 0) {
        // Lift is down, drive normally
        liftPrevLeft = left;
        liftPrevRight = right;
        return;
    }

    // Reduce turning, sharp turns tip the robot sideways
    double forward = (left + right) / 2.0;
    double turn = (left - right) / 2.0;
    turn *= 1 - t * (1 - LIFT_TURN_SCALE);
    left = forward + turn;
    right = forward - turn;

    // Lower top speed, skipped in precision mode since it is already slow
    if (!precisionMode) {
        double speedScale = 1 - t * (1 - LIFT_MIN_SPEED_SCALE);
        left *= speedScale;
        right *= speedScale;
    }

    // Ramp speed changes, including stopping, so the weight up top doesn't throw the robot
    double maxStep = LIFT_START_ACCEL + t * (LIFT_MIN_ACCEL - LIFT_START_ACCEL);
    left = limitAccel(left, liftPrevLeft, maxStep);
    right = limitAccel(right, liftPrevRight, maxStep);
}

// ---------------- Driver controls ----------------

// Tank drive: left stick drives the left side, right stick the right side
void handleDriveTrain()
{
    static double prevLeft = 0;
    static double prevRight = 0;

    double leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // -127..127 to percent
    leftY /= 1.27;
    rightY /= 1.27;

    leftY = logDriveJoystick(leftY);
    rightY = logDriveJoystick(rightY);

    // Percent to rpm for the blue (600 rpm) cartridge
    leftY *= 6;
    rightY *= 6;

    if (precisionMode) {
        leftY = limitAccel(leftY * PRECISION_SPEED_SCALE, prevLeft, PRECISION_MAX_ACCEL);
        rightY = limitAccel(rightY * PRECISION_SPEED_SCALE, prevRight, PRECISION_MAX_ACCEL);
    } else {
        prevLeft = leftY;
        prevRight = rightY;
    }

    applyLiftLimit(leftY, rightY);

    left_motors.move_velocity(leftY);
    right_motors.move_velocity(rightY);
}

// R1 intake, R2 bottom out / top in, L1 intake with the top stage slowed
void handleIntake()
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        bottomStage.move(127);
        topStage.move(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        bottomStage.move(-127);
        topStage.move(127);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        bottomStage.move(127);
        topStage.move(60);
    }
    else {
        bottomStage.brake();
        topStage.brake();
    }
}

// Piston toggles, each press flips the piston
void handleMidDescore()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        midDescore.set_value(!midDescore.get_value());
    }
}

void handleWing()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        wing.set_value(!wing.get_value());
    }
}

// TUNE: how far (encoder degrees) the lift rises when the clamp closes
const double CLAMP_LIFT_BUMP = 90;
// The clamp value that means closed, the lift bumps up when it switches to this
const int CLAMP_CLOSED = HIGH;

void handleClamp()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        int newValue = !clamp.get_value();
        clamp.set_value(newValue);
        if (newValue == CLAMP_CLOSED) {
            bumpLift(CLAMP_LIFT_BUMP);
        }
    }
}

// ---------------- Toggle mechanism ----------------
// Press B to fire the toggle piston and start the toggle motor, press B again
// to pull the piston back and stop the motor
const int TOGGLE_MOTOR_SPEED = 40; // TUNE: flip the sign if the motor spins the wrong way
bool toggleOn = false;

void handleToggle()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        toggleOn = !toggleOn;
        toggle.set_value(toggleOn);
        if (toggleOn) {
            toggleMotor.move(TOGGLE_MOTOR_SPEED);
        } else {
            toggleMotor.brake();
        }
    }
}

/**
 * Driver control. Started in its own task when the robot is enabled in
 * driver mode, or right after initialize() with no competition control.
 */
void opcontrol()
{
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(0, 0, 0);

    while (true) {
        handlePrecisionMode();
        handleLiftLimitToggle();
        handleDriveTrain();
        // Off this season, their buttons are used by the lift (R1, R2, L2), lift limit toggle (DOWN), and toggle (B)
        // handleMidDescore();
        // handleWing();
        // handleIntake();
        handleClamp();
        handleToggle();
        handleLift();
        pros::delay(20);
    }
}
