#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton_selector.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include <cmath>  // For fabs()
#include <algorithm>
#include <cstdlib>
#include "liblvgl/llemu.hpp"
#include "pros/adi.h"
#include "liftControl.h"
using namespace pros;
using namespace lemlib;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 bool colorSortEnabled = true; // Color sorting is on by default
extern bool red;  // Declared in auton_selector.cpp

void onCenter_button()
{
    red = !red;
    competitionSelector.displaySelectionBrain();
}
void initialize() {
	wing.set_value(LOW);
	clamp.set_value(HIGH); // start with the clamp open
	lcd::initialize();
	chassis.calibrate();
	pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);
	// Show current route on brain screen
	competitionSelector.displaySelectionBrain();
	
	// Register button callback for alliance toggle
	lcd::register_btn1_cb(onCenter_button);
	
    // Start a background task to update the LCD with robot pose and selection
    pros::Task screen_task([&]()
                           {
        while (true) {
            
            // Display lemlib pose
            pros::lcd::print(3, "LemLib X:%.1f Y:%.1f", chassis.getPose().x, chassis.getPose().y);
            //pros::lcd::print(4, "Dectection Size: %d", backDistance.get_object_size());
           /* double averageLiftPosition =
            (liftLeft.get_position() + liftRight.get_position()) / 2.0;
            pros::lcd::print(4, "Lift: %.1f deg", averageLiftPosition);
            */
            pros::lcd::print(5, "Heading: %.1f", chassis.getPose().theta);
            pros::delay(100);
        } });
    
    // Start a task for auton selection using limit switch
    pros::Task auton_select_task([&]() {
        bool lastPressed = false;
        while (true) {
            bool pressed = autonLimitSwitch.get_value();
            if (pressed && !lastPressed) {
                competitionSelector.nextSelection();
                competitionSelector.displaySelectionBrain();
            }
            lastPressed = pressed;
            pros::delay(100);
        }
    });
}
void autonomous() {
    controller.clear();
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    
    // Start a background task to display controller info during autonomous
    pros::Task display_task([&]() {
        while(true) {
            controller.print(0, 0, "Heading: %.1f", fmod(chassis.getPose().theta, 360.0));
            controller.print(2, 0, "LemLib X:%.1f", chassis.getPose().x);
            controller.print(4, 0, "LemLib Y:%.1f", chassis.getPose().y);
            pros::delay(100);
        }
    });
    
    competitionSelector.runSelection();
    all_motors.brake();
    delay(2000);
    all_motors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 */

void competition_initialize() {
	

    // show current route on brain screen
    competitionSelector.displaySelectionBrain();

    // assign buttons to actions in auton selector
    lcd::register_btn1_cb(onCenter_button);
}

const double SMOOTHING_DENOMINATOR = 220; // Used to normalize the exponential curve
const double EXPONENTIAL_POWER = 2.2;       // Controls how aggressive the curve is
// Helper function that makes joystick input more precise for small movements
// while maintaining full power at maximum joystick
double logDriveJoystick(double joystickPCT)
{
    // Get the absolute value for calculation
    double magnitude = fabs(joystickPCT);

    // Calculate the smoothed value
    double smoothedValue = pow(magnitude, EXPONENTIAL_POWER) / SMOOTHING_DENOMINATOR;

    // Restore the original sign (positive or negative)
    return joystickPCT >= 0 ? smoothedValue : -smoothedValue;
}

// Precision mode: press Y to toggle slower, smoother driving for lining up
bool precisionMode = false;
const double PRECISION_SPEED_SCALE = 0.4; // Max speed in precision mode (40% of normal)
const double PRECISION_MAX_ACCEL = 15.0;  // Max rpm change per 20ms loop in precision mode

// Limits how fast a side's velocity can change so the robot doesn't jerk
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

// Lift limiting: automatically slows and smooths the drive as the lift goes up
// so the top heavy robot doesn't tip or jerk, set to false to turn it off
const bool LIFT_LIMIT_ENABLED = true;
const double LIFT_LIMIT_START = 200;      // Lift height (encoder deg) where limiting starts
const double LIFT_LIMIT_FULL = 650;       // Lift height where limiting is strongest
const double LIFT_MIN_SPEED_SCALE = 0.5;  // Max speed at full height (50% of normal)
const double LIFT_START_ACCEL = 60.0;     // Max rpm change per 20ms loop at start height
const double LIFT_MIN_ACCEL = 20.0;       // Max rpm change per 20ms loop at full height
const double LIFT_TURN_SCALE = 0.8;       // Turning strength at full height (60% of normal)

// Press DOWN to toggle lift limiting on and off during a match
bool liftLimitOn = true;

void handleLiftLimitToggle()
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        liftLimitOn = !liftLimitOn;
        controller.rumble(liftLimitOn ? ".." : "--"); // two short = on, two long = off
    }
}

// Returns 0 when the lift is low, 1 when it is at full height, and in between as it rises
double liftLimitAmount()
{
    if (!LIFT_LIMIT_ENABLED) return 0;
    if (!liftLimitOn) return 0;
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
        // lift is down, drive normally
        liftPrevLeft = left;
        liftPrevRight = right;
        return;
    }

    // reduce turning, sharp turns tip the robot sideways
    double forward = (left + right) / 2.0;
    double turn = (left - right) / 2.0;
    turn *= 1 - t * (1 - LIFT_TURN_SCALE);
    left = forward + turn;
    right = forward - turn;

    // lower top speed, skipped in precision mode since it is already slow
    if (!precisionMode) {
        double speedScale = 1 - t * (1 - LIFT_MIN_SPEED_SCALE);
        left *= speedScale;
        right *= speedScale;
    }

    // ramp speed changes, including stopping, so the weight up top doesn't throw the robot
    double maxStep = LIFT_START_ACCEL + t * (LIFT_MIN_ACCEL - LIFT_START_ACCEL);
    left = limitAccel(left, liftPrevLeft, maxStep);
    right = limitAccel(right, liftPrevRight, maxStep);
}

void handleDriveTrain()
{
    static double prevLeft = 0;
    static double prevRight = 0;

    // get left y and right y positions
    double leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // convert to pct
    leftY /= 1.27;
    rightY /= 1.27;

    leftY = logDriveJoystick(leftY);
    rightY = logDriveJoystick(rightY);

    // convert to gearset
    leftY *= 6;
    rightY *= 6;

    if (precisionMode) {
        // scale down top speed and smooth out acceleration
        leftY = limitAccel(leftY * PRECISION_SPEED_SCALE, prevLeft, PRECISION_MAX_ACCEL);
        rightY = limitAccel(rightY * PRECISION_SPEED_SCALE, prevRight, PRECISION_MAX_ACCEL);
    } else {
        prevLeft = leftY;
        prevRight = rightY;
    }

    applyLiftLimit(leftY, rightY);

    // skills change

    left_motors.move_velocity(leftY);
    right_motors.move_velocity(rightY);
}



void handleIntake()
{
	
	if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
		bottomStage.move(127);
		topStage.move(127);
	}


	else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
		bottomStage.move(-127);
		topStage.move(127);
	}


	else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
		bottomStage.move(127);
		topStage.move(60);
	}


	else{
		bottomStage.brake();
		topStage.brake();
	}

}
/*
void handleLift()
{
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        liftLeft.move_velocity(100);
        liftRight.move_velocity(100);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        liftLeft.move_velocity(-100);
        liftRight.move_velocity(-100);
    }
    else{
        liftLeft.brake();
        liftRight.brake();
     } // r2 down
}
*/
void handleHood(){
	if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
		hood.set_value(!hood.get_value());
	}
}
void handleMidDescore(){
	if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
		midDescore.set_value(!midDescore.get_value());
	}
}
void handleWing(){
	if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		wing.set_value(!wing.get_value());
	}
}
void handleClamp(){
	if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
		clamp.set_value(!clamp.get_value());
	}
}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.setPose(0, 0, 0);
	while (true) {
		handlePrecisionMode();
		handleLiftLimitToggle();
		handleDriveTrain();
		/*handleHood();
		handleMidDescore();
		handleWing();
		handleIntake();*/
        handleClamp();
        handleLift();
		pros::delay(20);                               // Run for 20 ms then update
    }
}