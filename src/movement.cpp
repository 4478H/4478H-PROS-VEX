#include "main.h"
#include "api.h"
#include "pros/adi.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton_routes.h"
#include "movement.h"
#include "liftControl.h"
#include <string>
#include <iostream>
#include <thread>
#include <algorithm>
#include <atomic>
#include <memory>
#include "pros/rtos.hpp"


void Intake(double direction)
{
    bottomStage.move(direction * 127);
    topStage.move(direction * 127);
}
void scoreMid(){
    bottomStage.move(127);
    topStage.move(-80);
}
void load(){
    bottomStage.move(80);
}
void stopIntake()
{
    bottomStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    topStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    bottomStage.brake();
    topStage.brake();
}
void outake(int time)
{
    bottomStage.move(-127);//intake out then in
    topStage.move(-80);

    pros::delay(time);

}
void outakeSkills(int time)
{
    bottomStage.move(-50);//intake out then in
    topStage .move(-10);

    pros::delay(time);

}
void scoreMidSkills(){
    bottomStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    topStage.set_brake_mode(MOTOR_BRAKE_HOLD);
    bottomStage.move(100);
    topStage.move(-30);
}
// Improved slew rate limiting for smoother control
double slewStep = 30.0;  // Increased for better responsiveness
double slewRate = 0.5; // Maximum change per iteration (15% of difference)

double slew(double val, double fwdVal)
{
    static double prevVal = 0;
    static double prevFwdVal = 0;

    // Reset slew if target direction changes significantly
    if ((fwdVal >= 0 && prevFwdVal < 0) || (fwdVal < 0 && prevFwdVal >= 0))
    {
        prevVal = 0;
    }
    prevFwdVal = fwdVal;

    // Calculate the difference between target and current
    double difference = val - prevVal;

    // If difference is small, just return the target
    if (fabs(difference) < 0.5)
    {
        prevVal = val;
        return val;
    }

    // Calculate slew rate based on difference (adaptive)
    double currentSlewStep = slewStep + (fabs(difference) * slewRate);

    // Apply slew rate limiting
    if (difference > 0)
    {
        // Positive direction
        if (prevVal + currentSlewStep < val)
        {
            prevVal += currentSlewStep;
        }
        else
        {
            prevVal = val;
        }
    }
    else
    {
        // Negative direction
        if (prevVal - currentSlewStep > val)
        {
            prevVal -= currentSlewStep;
        }
        else
        {
            prevVal = val;
        }
    }

    return prevVal;
}

// Drive PID gains (made global for monitoring)
double drive_kP = 0.17; 
double drive_kI = 0.00; 
double drive_kD = 0.45;

void drivePID(double fwdVal, double maxSpeedPercent, double timeout)
{
    
    double kP = drive_kP; 
    double kI = drive_kI; 
    double kD = drive_kD; 

    const double diameter = 3.25;
    const double pi = 3.14159;
    const double outputGear = 60;
    const double inputGear = 36;

    double num = fwdVal;                                       // leave this one alone
    double denom = (diameter * pi) * (inputGear / outputGear); // leave this one alone
    double target = (num / denom) * 360;                       // leave this one alone
    
    // Remember where each motor starts instead of taring, LemLib uses these
    // same encoders for odometry and taring them makes its x and y jump back
    const double startBackLeft = mBackLeft.get_position();
    const double startMidLeft = mMidLeft.get_position();
    const double startBackRight = mBackRight.get_position();
    const double startMidRight = mMidRight.get_position();

    double startTime = pros::millis();
    double error = 0;
    double prevError = 0;
    double integral = 0;
    
    // Dynamic goal tracking (proportional to distance)
    const double pollingRate = 20; // ms - consistent with sister team
    
    // Exit conditions: position error only (degrees)
    const double errorThreshold = 6.0; // degrees - position threshold
    int inGoal = 0;
    const int goalsNeeded = 1; // Only need 1 iteration when conditions are met for faster exit
    
    // Clamp and convert maxSpeedPercent (0-100) -> max motor units (0-127)
    if (maxSpeedPercent < 0)
        maxSpeedPercent = 0;
    if (maxSpeedPercent > 100)
        maxSpeedPercent = 100;
    double maxMotor = (maxSpeedPercent / 100.0) * 127.0;

    bool hasMoved = false; // Track if robot has started moving (prevent immediate exit)
    
    while (inGoal < goalsNeeded)
    {
        // Use the MEDIAN of all 4 drivetrain motor positions for robustness
        double positions[4] = {
            mBackLeft.get_position() - startBackLeft, mMidLeft.get_position() - startMidLeft,
            mBackRight.get_position() - startBackRight, mMidRight.get_position() - startMidRight
        };
        // sort to find middle two for even count (4): indices 1 and 2
        std::sort(positions, positions + 4);
        double medianPos = (positions[1] + positions[2]) / 2.0;
        double processVariable = medianPos * 360;
        
        error = target - processVariable;
        
        // Track if robot has moved significantly OR is close to target (prevents immediate exit at start)
        // For small movements, check if we're close to target instead of requiring large movement
        // Also check if enough time has passed (prevents false exits at start)
        if (fabs(processVariable) > 20 || fabs(error) < errorThreshold * 2 || (pros::millis() - startTime) > 100) {
            hasMoved = true;
        }

        // Proportional term
        double P = error * kP;
        
        // Integral term with anti-windup 
        integral += error;
        double I = integral * kI;
        I = std::clamp(I, -50.0, 50.0); // Clamp integral contribution
        
        // Derivative term 
        double D = ((error - prevError) / pollingRate) * kD;
        
        // Calculate total PID output
        double motorPower = P + I + D;
        
        // Apply slew only at start for smooth acceleration
        if ((pros::millis() - startTime) < 300) {
            motorPower = slew(motorPower, fwdVal);
        }
        
        // Clamp output
        motorPower = std::clamp(motorPower, -maxMotor, maxMotor);

        // Move motors
        left_motors.move(motorPower);
        right_motors.move(motorPower);

        // Fast exit condition: position error is small AND robot has moved
        // This allows immediate exit when robot has reached the target
        if (hasMoved && fabs(error) < errorThreshold)
        {
            inGoal++;
            // Exit immediately once condition is met (no need to wait for multiple iterations)
            if (inGoal >= goalsNeeded) {
                break;
            }
        }
        else
        {
            inGoal = 0; // Reset if conditions not met
        }

        // Timeout safety - break immediately if timeout reached
        if ((pros::millis() - startTime) >= timeout)
        {
            break;
        }

        prevError = error;
        
        // Consistent polling rate
        pros::delay(pollingRate);
    }
    
    // Stop motors once goal is reached
    left_motors.brake();
    right_motors.brake();
}
void turnToHeadingSmart(float theta, int timeout, TurnToHeadingParams params, bool async)
{
    // Get current heading
    float currentHeading = chassis.getPose().theta;
    
    // Calculate angle difference
    float angleDiff = theta - currentHeading;
    
    // Normalize to [-180, 180]
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    // For small turns, automatically apply minSpeed if not already set
    // This ensures the robot has enough power to overcome friction and motor deadband
    if (params.minSpeed == 0)
    {
        float absAngleDiff = fabs(angleDiff);
        
        // Very small turns (< 15 degrees) need higher minSpeed to overcome friction
        if (absAngleDiff < 15.0)
        {
            params.minSpeed = 40; // Higher minimum speed for very small turns
            params.earlyExitRange = 2; // Tighter exit range for precision
        }
        // Small to medium turns (15-30 degrees) need moderate minSpeed
        else if (absAngleDiff < 30.0)
        {
            params.minSpeed = 30; // Moderate minimum speed
            params.earlyExitRange = 3;
        }
    }
    
    // Call the actual turnToHeading function with async parameter
    chassis.turnToHeading(theta, timeout, params, async);
}

// ---------------- Override helpers ----------------
extern bool red; // Declared in auton_selector.cpp

// Clamp is open when HIGH and closed when LOW
// TUNE: how long the piston needs to fully open or close
const int CLAMP_DELAY = 150;
void grabPin(){
    clamp.set_value(LOW);
    pros::delay(CLAMP_DELAY);
}
void releasePin(){
    clamp.set_value(HIGH);
    pros::delay(CLAMP_DELAY);
}

// How many times the back roller has to bump the Toggle to reach each color
// TUNE: depends on which color the Toggle starts on and which way the roller turns it
const int TOGGLE_PRESSES_RED = 1;
const int TOGGLE_PRESSES_BLUE = 1;

// Back the roller into the Toggle to set it to our Alliance color,
// then pull away since a Toggle only counts when no Robot is touching it
void setToggle(double backupDist){
    int presses = red ? TOGGLE_PRESSES_RED : TOGGLE_PRESSES_BLUE;
    for(int p = 0; p < presses; p++){
        drivePID(-backupDist, 40, 1000);
        pros::delay(150);
        drivePID(backupDist, 100, 1000);
    }
}

// Raise the lift to a preset, drive the Pin over the Goal, drop it,
// back away, then lower the lift
void placePin(int preset, double approachDist){
    liftToPreset(preset, true, 1500, 127);
    Lift.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    Lift.brake();
    drivePID(approachDist, 60, 1500);
    releasePin();
    drivePID(-approachDist, 100, 1500);
    // lower in the background so the robot can head to the next target right away
    liftToPreset(0, false, 1500, 127);
}
