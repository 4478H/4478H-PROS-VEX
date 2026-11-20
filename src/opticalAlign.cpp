#include "opticalAlign.h"
#include "devices.h"
#include "movement.h"
#include <algorithm>
#include <cmath>

// Distances in mm, angles in degrees, speeds out of 127

// Object size range the distance sensor reports for a goal
// Smaller is noise, bigger is a wall
const int MIN_GOAL_SIZE = 45;
const int MAX_GOAL_SIZE = 90;

const int SCAN_SPEED = 28;             // max speed while sweeping for the goal
const double BACKUP_SPEED = 50.0;      // speed for backing into the goal
const double BACKUP_SLOW_SPEED = 30.0; // speed once the backup slows down
const double HEADING_KP = 1.5;         // how hard to steer back onto the goal heading
const double MAX_CORRECTION = 30;      // cap on that steering power
const int LOST_FRAMES_LIMIT = 5;       // frames without seeing the goal before trying to find it again

// One side's numbers, the two long goals only differ in these
struct GoalSide {
    double scanStart;   // heading to start the sweep at
    double scanEnd;     // heading to sweep toward
    double backupTime;  // max time to back up, in ms
    double slowAfter;   // ms into the backup to slow down
};
const GoalSide NEAR_SIDE = {290.0, 360.0, 1300, 600};
const GoalSide FAR_SIDE  = {70.0, 150.0, 900, 650};

static bool detectsGoal() {
    int objSize = backDistance.get_object_size();
    return (objSize >= MIN_GOAL_SIZE && objSize <= MAX_GOAL_SIZE);
}

static void stopDrive() {
    left_motors.brake();
    right_motors.brake();
}

// Sweep from scanStart to scanEnd until the goal is seen
// Returns true and sets targetHeading to where it was seen, or false if it never was
static bool scanForGoal(const GoalSide& side, double& targetHeading) {
    // Snap to the start, then sweep slowly in the background while we watch the sensor
    turnToHeadingSmart(side.scanStart, 500, {.minSpeed = 100});
    turnToHeadingSmart(side.scanEnd, 1000, {.maxSpeed = SCAN_SPEED});

    while (chassis.isInMotion()) {
        double dist = backDistance.get();
        int objSize = backDistance.get_object_size();
        double currHeading = imu.get_heading();
        bool isDetected = detectsGoal();

        pros::lcd::clear_line(0);
        pros::lcd::print(0, "Detected: %s", isDetected ? "YES" : "NO");
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Size: %d  Dist: %.0fmm", objSize, dist);
        pros::lcd::clear_line(2);
        pros::lcd::print(2, "Heading: %.1f -> %.1f", currHeading, side.scanEnd);

        if (isDetected) {
            chassis.cancelAllMotions();
            targetHeading = currHeading;
            stopDrive();
            pros::delay(150);
            return true;
        }

        pros::delay(20);
    }

    stopDrive();
    return false;
}

// Back up into the goal, steering to hold targetHeading
// If the goal is lost for a few frames, pull forward and scan again
static void backIntoGoal(const GoalSide& side, double targetHeading) {
    double startTime = pros::millis();
    int lostFrames = 0;

    while (pros::millis() - startTime < side.backupTime) {
        if (backDistance.get_object_size() == 0) {
            lostFrames++;

            if (lostFrames >= LOST_FRAMES_LIMIT) {
                // Lost the goal, stop and pull forward to get a better view
                stopDrive();
                pros::delay(100);
                left_motors.move(70);
                right_motors.move(70);
                pros::delay(300);
                stopDrive();
                pros::delay(100);

                if (scanForGoal(side, targetHeading)) {
                    // Found it again, restart the backup
                    startTime = pros::millis();
                    lostFrames = 0;
                    continue;
                }
                return;
            }
        } else {
            lostFrames = 0;
        }

        // Heading error, the short way around (-180 to 180)
        double headingErr = targetHeading - imu.get_heading();
        while (headingErr > 180) headingErr -= 360;
        while (headingErr < -180) headingErr += 360;

        double correction = std::clamp(headingErr * HEADING_KP, -MAX_CORRECTION, MAX_CORRECTION);

        double elapsed = pros::millis() - startTime;
        double speed = (elapsed >= side.slowAfter) ? BACKUP_SLOW_SPEED : BACKUP_SPEED;

        // Driving backwards, so the correction is added to the left and taken from the right
        left_motors.move(-speed + correction);
        right_motors.move(-speed - correction);

        pros::delay(20);
    }

    stopDrive();
}

static void alignToGoal(const GoalSide& side, double targetHeading) {
    if (scanForGoal(side, targetHeading)) {
        backIntoGoal(side, targetHeading);
    } else {
        stopDrive();
    }
}

void alignToLongGoal() {
    alignToGoal(NEAR_SIDE, 0);
}

void alignToLongGoalOpposite() {
    alignToGoal(FAR_SIDE, 180.0);
}
