#include "liftControl.h"
#include "devices.h"
#include <cmath>
#include <atomic>

using namespace pros;

// Feedforward constant that counters gravity, tune this first
static double kG = 15;
// Small proportional correction on top of the feedforward
static double kP = 0.3;

// Wherever the lift lands when a button is released becomes the new hold point
static double targetPosition = 0;

// Full PID constants used only during autonomous, tune these separately
// from kG and kP above since precise positioning needs different gains
static double kP_auton = 0.8;
static double kI_auton = 0.0;
static double kD_auton = 0.04;

// How close counts as arrived, in encoder degrees
static const double ARRIVED_TOLERANCE = 5;

// Preset heights in encoder degrees, edit these to match your actual lift
// Index 0 is the bottom, add or remove entries as needed
static const double LIFT_PRESETS[] = {-12, 255, 400, 600, 700, 1000};
static const int LIFT_PRESET_COUNT = 6;

// Bumped every time a new auton lift move starts, a running move quits
// as soon as it sees a newer one so two moves never fight over the motors
static std::atomic<int> liftMoveId{0};

void homeLift() {
    // gentle power, not full 127, so it does not slam the hard stop
    liftLeft.move(-40);
    liftRight.move(-40);
pros::delay(300);
    int stableCount = 0;
    while (stableCount < 10) {
        double velocity = liftLeft.get_actual_velocity();

        if (fabs(velocity) < 2) {
            stableCount++;
        } else {
            stableCount = 0;
        }

        pros::delay(10);
    }

    liftLeft.brake();
    liftRight.brake();

    // zero the encoders right at this exact spot, whatever number they were at before
    liftLeft.tare_position();
    liftRight.tare_position();

    targetPosition = 0;
}

void handleLift() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        targetPosition = 0;
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        // held, run up at full power, no correction applied
        liftLeft.move(90);
        liftRight.move(90);
        targetPosition = (liftLeft.get_position() + liftRight.get_position()) / 2.0;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        // held, run down at full power, no lower limit
        liftLeft.move(-90);
        liftRight.move(-90);
        targetPosition = (liftLeft.get_position() + liftRight.get_position()) / 2.0;
    }
    else {
        // neither button held, hold position against gravity
        double current = (liftLeft.get_position() + liftRight.get_position()) / 2.0;
        double error = targetPosition - current;
        double output = kG + (kP * error);

        if (output > 95) output = 95;
        if (output < -105) output = -95;

        liftLeft.move(output);
        liftRight.move(output);
    }
}

void liftToPosition(double target, int timeoutMs, int maxSpeed) {
    int myId = ++liftMoveId;
    double integral = 0;
    double prevError = 0;
    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < (uint32_t)timeoutMs) {
        // a newer lift move took over, let it have the motors
        if (myId != liftMoveId) {
            return;
        }


        double current = (liftLeft.get_position() + liftRight.get_position()) / 2.0;
        double error = target - current;

        integral += error;
        double derivative = error - prevError;
        prevError = error;

        double output = (kP_auton * error) + (kI_auton * integral) + (kD_auton * derivative);

        if (output > maxSpeed) output = maxSpeed;
        if (output < -maxSpeed) output = -maxSpeed;

        liftLeft.move(output);
        liftRight.move(output);

        if (fabs(error) < ARRIVED_TOLERANCE) {
            break;
        }

        pros::delay(10);
    }

    // update the shared hold target so opcontrol picks up smoothly
    // if the driver takes over right after this finishes
    targetPosition = target;
}

void liftToPreset(int preset, bool waitUntilDone, int timeoutMs, int maxSpeed) {
    if (preset < 0 || preset >= LIFT_PRESET_COUNT) {
        // invalid preset number, ignore the call rather than crash
        return;
    }

    double target = LIFT_PRESETS[preset];

    if (waitUntilDone) {
        liftToPosition(target, timeoutMs, maxSpeed);
    } else {
        // run the move in its own task so the next lines of the route start right away
        pros::Task([target, timeoutMs, maxSpeed]() {
            liftToPosition(target, timeoutMs, maxSpeed);
        });
    }
}

void bumpLift(double amount) {
    // start from where the lift actually is so repeated bumps don't stack up
    targetPosition = getLiftPosition() + amount;
}

double getLiftPosition() {
    return (liftLeft.get_position() + liftRight.get_position()) / 2.0;
}
