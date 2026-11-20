#include "colorSort.h"
#include "devices.h"
#include "auton_selector.h"
#include <cmath>

using namespace pros;

// Hue ranges in degrees, red wraps around 0 so it has two ranges
const double RED_HUE_MIN = 0.0;
const double RED_HUE_MAX = 30.0;
const double RED_HUE_WRAP_MIN = 330.0;
const double BLUE_HUE_MIN = 200.0;
const double BLUE_HUE_MAX = 260.0;
const double MIN_SATURATION = 50.0; // below this the color is too washed out to trust
const double MIN_BRIGHTNESS = 50.0; // above this counts as a ball being there
const double MIN_PROXIMITY = 50.0;  // above this counts as a ball being there

// Intake reversal settings
const int REVERSE_SPEED = -127;
const int REVERSE_TIME_MS = 300; // how long to reverse when the wrong color shows up

// State for the non-blocking reversal
static bool isReversing = false;
static uint32_t reverseStartTime = 0;
static bool lastBallDetected = false;

bool getSelectedColor() {
    return red;
}

void setSelectedColor(bool isRed) {
    red = isRed;
}

void toggleColorSelection() {
    red = !red;
    displayColorSelection();
}

void displayColorSelection() {
    pros::lcd::clear_line(0);
    pros::lcd::print(0, "Color Sort: %s", red ? "RED" : "BLUE");
    pros::lcd::clear_line(1);
    pros::lcd::print(1, "Press Center to Change");
}

bool isBallDetected() {
    double brightness = colorSens.get_brightness();
    double proximity = colorSens.get_proximity();
    return (brightness > MIN_BRIGHTNESS || proximity > MIN_PROXIMITY);
}

BallColor getBallColor() {
    if (!isBallDetected()) {
        return BallColor::NONE;
    }

    double hue = colorSens.get_hue();
    double saturation = colorSens.get_saturation();
    double brightness = colorSens.get_brightness();

    // Keep hue in the 0-360 range
    while (hue < 0) hue += 360;
    while (hue >= 360) hue -= 360;

    if (saturation < MIN_SATURATION || brightness < MIN_BRIGHTNESS) {
        return BallColor::NONE;
    }

    if ((hue >= RED_HUE_MIN && hue <= RED_HUE_MAX) || hue >= RED_HUE_WRAP_MIN) {
        return BallColor::RED;
    }

    if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
        return BallColor::BLUE;
    }

    return BallColor::NONE;
}

// Call this every loop in opcontrol. It never waits, so driving isn't interrupted
void runColorSort() {
    BallColor detectedColor = getBallColor();
    uint32_t currentTime = pros::millis();

    // Already spitting out a ball, keep going until the time is up
    if (isReversing) {
        if (currentTime - reverseStartTime >= REVERSE_TIME_MS) {
            intake.brake();
            isReversing = false;
        } else {
            intake.move(REVERSE_SPEED);
            return; // don't check for new balls while reversing
        }
    }

    if (detectedColor == BallColor::NONE) {
        lastBallDetected = false;
        return;
    }

    // Only act once per ball, when it first shows up
    if (lastBallDetected) {
        return;
    }
    lastBallDetected = true;

    bool isDetectedRed = (detectedColor == BallColor::RED);
    bool shouldKeep = (isDetectedRed == red);

    if (!shouldKeep) {
        // Wrong color, spit it out
        isReversing = true;
        reverseStartTime = currentTime;
        intake.move(REVERSE_SPEED);

        pros::lcd::clear_line(3);
        pros::lcd::print(3, "Rejected: %s", isDetectedRed ? "RED" : "BLUE");
    } else {
        // Right color, leave the intake to the driver
        pros::lcd::clear_line(3);
        pros::lcd::print(3, "Accepted: %s", isDetectedRed ? "RED" : "BLUE");
    }
}
