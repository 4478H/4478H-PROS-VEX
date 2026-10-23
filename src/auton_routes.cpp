#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "auton_routes.h"
#include "devices.h"
#include "movement.h"
#include "opticalAlign.h"
#include "liftControl.h"
#include "pros/rtos.hpp"
#include <cmath>

// ---------------- Override starting positions ----------------
// Each Alliance owns two Quadrants and each Robot must start in a different one
//   Q1 = the Quadrant on your own Alliance Station wall
//   Q2 = the Quadrant on the side wall (the other Quadrant with your colored Goal)
// The Field is the same for red and blue when rotated 180 degrees, so red and
// blue run identical code. Q2 is a mirror image of Q1, so every route takes
// side = 1 for Q1 and side = -1 for Q2, which flips every turn
//
// Start: centered on your Quadrant's Toggle, roller facing it, about 1" gap,
// facing straight into the Field, preload in the clamp
//
// Field map in "Quadrant frame" (inches): origin is the Field center, +y points
// from our Toggle toward the center, +x points toward our other Quadrant
// (positions are from the official Field layout, Q2 is the same map with x flipped)
struct FieldPoint { double x; double y; };
static const FieldPoint START = {0, -55.5};          // TUNE: measure center of robot to center of Field
static const FieldPoint ALLIANCE_GOAL = {24, -48};   // our colored Goal, empty at the start
static const FieldPoint NEUTRAL_GOAL = {-24, -48};   // already holds a yellow Pin, the Toggle makes it ours
static const FieldPoint YELLOW_PIN = {24, -24};      // yellow Pin in a Cup on the line between our two Quadrants,
                                                     // BOTH our robots can reach it, so only one may run a route that takes it
static const FieldPoint MID_PIN = {0, -24};          // red/blue Pin in a Cup on the Midfield corner straight ahead, one per Quadrant

static const double CLAMP_REACH = 10;      // TUNE: robot center to the center of a Pin in the clamp
static const double PLACE_APPROACH = 6;    // TUNE: how far placePin drives in over the Goal
static const double WALL_CLEARANCE = 6;    // drive this far off the Toggle before turning so the robot does not swing into it
// TUNE: lift presets from liftControl.cpp
static const int ALLIANCE_GOAL_PRESET = 1;   // Pin straight onto the empty short Alliance Goal
static const int ALLIANCE_STACK_PRESET = 2;  // Pin + Cup on top of the Pin already in the Alliance Goal
static const int NEUTRAL_STACK_PRESET = 3;   // Pin + Cup on top of the yellow Pin on the taller Neutral Goal

static const double DEG_TO_RAD = 3.14159265 / 180.0;

// Where the robot is in the Quadrant frame, tracked from our own moves since
// drivePID tares the drive encoders, which throws off LemLib's x and y
static FieldPoint robotAt;

// Drive straight at the current heading and keep robotAt up to date
static void driveStraight(double dist, int side, double speed = 80, double timeout = 2000){
    double heading = chassis.getPose().theta * side * DEG_TO_RAD;
    drivePID(dist, speed, timeout);
    robotAt.x += dist * sin(heading);
    robotAt.y += dist * cos(heading);
}

// Turn to face a point, then drive at it and stop stopShort inches before it
// Returns the distance driven so a route can back out the same way
static double driveToward(FieldPoint target, double stopShort, int side, double speed = 80, double timeout = 2000){
    double dx = target.x - robotAt.x;
    double dy = target.y - robotAt.y;
    double heading = atan2(dx, dy) / DEG_TO_RAD;
    double dist = hypot(dx, dy) - stopShort;
    chassis.turnToHeading(heading * side, 900, {}, false);
    driveStraight(dist, side, speed, timeout);
    return dist;
}

// Drive up to a Pin standing in a Cup and clamp it, the Cup comes with it
static void pickUp(FieldPoint pin, int side){
    driveToward(pin, CLAMP_REACH, side, 60);
    grabPin();                        // TUNE: check the Cup stays nested under the Pin
}

// Drive up to a Goal and drop whatever is in the clamp on it
// Returns the distance driven so a route can back out the same way
static double scoreOn(FieldPoint goal, int preset, int side){
    double dist = driveToward(goal, CLAMP_REACH + PLACE_APPROACH, side);
    placePin(preset, PLACE_APPROACH);
    return dist;
}

static void startRoute(int side){
    chassis.setPose(0, 0, 0);
    robotAt = START;
    grabPin();                        // hold the preload so it does not fall out
    setToggle(6);                     // TUNE: gap between roller and Toggle
    driveStraight(WALL_CLEARANCE, side, 60, 1000);  // a Toggle only counts if no Robot is touching it
}

// Preload into our empty Alliance Goal, then back out the way we came
// so the next path clears the Goal we just scored on
static void preloadToAllianceGoal(int side){
    double toGoal = scoreOn(ALLIANCE_GOAL, ALLIANCE_GOAL_PRESET, side);   // about an 86 degree turn
    driveStraight(-toGoal, side, 60, 1000);
}

// ---------------- Override routes ----------------
// Every value marked TUNE is a starting guess, measure them on a real field before competing
// Point totals assume the Toggle is set to our color, a Pin in the clamp keeps its
// Cup, and the Cup half that covers the Pin below it does not hide a scored half

// Safe fallback: just set our Quadrant's Toggle and stay out of the way
// 10 points, the yellow Pin already on our Neutral Goal becomes ours
void toggleOnly(int side){
startRoute(side);
}

// Toggle + Preload: place the preload in our colored Goal so its yellow half also scores for us
// 25 points: 10 for the Neutral Goal's yellow Pin + 5 red/blue half + 10 owned yellow half
void togglePreload(int side){
startRoute(side);
preloadToAllianceGoal(side);
}

// Toggle + Preload, then stack the red/blue Pin (with its Cup) from the
// Midfield corner on top of the preload. Only uses objects in our own
// Quadrant, so both robots on the Alliance can run it
// 30 points (25 + the Pin's alliance-colored half), 2 Pins in our Alliance Goal
void togglePreloadStack(int side){
startRoute(side);
preloadToAllianceGoal(side);
pickUp(MID_PIN, side);                          // straight ahead
scoreOn(ALLIANCE_GOAL, ALLIANCE_STACK_PRESET, side);
}

// Toggle + Preload, stack the shared yellow Pin on the preload, then stack the
// Midfield corner Pin on the yellow Pin already in our Neutral Goal
// 4 Pins and 2 Goals with 2 Pins, about 45 points. If the partner runs
// togglePreloadStack from the other Quadrant the Alliance has 7 Pins and
// 3 Goals with 2 Pins, which is the Autonomous Win Point (both robots must end off the wall)
// Only ONE robot on the Alliance may run this, it takes the yellow Pin both Quadrants share
// TUNE: four cycles is tight for 15 seconds, time it and raise speeds if needed
void awpStack(int side){
startRoute(side);
preloadToAllianceGoal(side);
pickUp(YELLOW_PIN, side);                       // about 43 degrees, passes 16" from our Goal
scoreOn(ALLIANCE_GOAL, ALLIANCE_STACK_PRESET, side);
pickUp(MID_PIN, side);
scoreOn(NEUTRAL_GOAL, NEUTRAL_STACK_PRESET, side);   // passes 20" from the Pins on the Autonomous Line
}

void redLoaderSideOneGoal(int i){
    chassis.setPose(0,0,0);
    drivePID(4, 100, 600);
    drivePID(-7, 100, 600);
    drivePID(4, 100, 600);
    drivePID(-7, 100, 600);
    liftToPreset(3,false, 1000, 127);
   // drivePID(12, 127, 1500);
    //chassis.turnToHeading(90, 500);
    //drivePID(16.3, 100, 1500);
    chassis.moveToPose(17.8, 14.5, 90 ,1800, {.lead=0.2}, false);
    drivePID(3.8, 100, 600);
    liftToPreset(2, true, 400, 80);
    //pros::delay(250);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 60, 600);
    liftToPreset(0, true, 500, 127);
    chassis.turnToHeading(138, 500);
    drivePID(13, 50, 1700);
    chassis.swingToHeading(180, DriveSide::RIGHT, 1300, {}, false);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPreset(3, false, 1000, 80);
    chassis.turnToHeading(120, 900);
    drivePID(12, 100, 500);
    drivePID(-15, 50, 4000);
    chassis.turnToHeading(90, 700, {.maxSpeed = 90});
    drivePID(9.5, 50, 1500);
    liftToPreset(2, false, 1000, 80);
    //liftToPreset(1, false, 1000, 80);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 100, 500);
    chassis.turnToHeading(-80, 500);
   // drivePID(24, 100, 1000);
   // chassis.turnToHeading(135, 500);
   // drivePID(90, 100, 1000);
    /*chassis.moveToPose(42.978, 6.38, 30 ,1500, {.lead=0.2,.maxSpeed = 100}, true);
    drivePID(3, 30, 500);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPreset(4, true, 1000, 40);
    chassis.moveToPose(30.884, 16.005, -90 ,1500, {.lead=0.3,.maxSpeed = 40}, true);
    clamp.set_value(LOW);
*/
}
void leftSideOneGoal(int i){
    chassis.setPose(0,0,0);
    drivePID(4, 100, 600);
    drivePID(-7, 100, 600);
    drivePID(4, 100, 600);
    drivePID(-7, 100, 600);
    liftToPreset(2,false, 1000, 127);
   // drivePID(12, 127, 1500);
    //chassis.turnToHeading(90, 500);
    //drivePID(16.3, 100, 1500);
    chassis.moveToPose(-18.1, 14.8, -90 ,1800, {.lead=0.2}, false);
    drivePID(3.8, 100, 600);
    liftToPreset(1, true, 400, 80);
    //pros::delay(250);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 60, 600);
    liftToPreset(0, true, 500, 127);
    chassis.turnToHeading(-140, 500);
    drivePID(15, 50, 1700);
    chassis.swingToHeading(180, DriveSide::RIGHT, 1300, {}, false);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPreset(3, false, 1000, 80);
    chassis.turnToHeading(-120, 900);
    drivePID(12, 100, 500);
    drivePID(-15, 50, 4000);
    chassis.turnToHeading(-90, 700, {.maxSpeed = 90});
    drivePID(9.5, 50, 1500);
    liftToPreset(2, false, 1000, 80);
    //liftToPreset(1, false, 1000, 80);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 100, 500);
    chassis.turnToHeading(-30, 500);
    drivePID(24, 100, 1000);
    chassis.turnToHeading(-135, 500);
    drivePID(90, 100, 1000);
}
void loaderSide2Goal(int i){
    chassis.setPose(0,0,0);
    drivePID(-3, 60, 500);
    liftToPreset(1, false, 1000, 127);
    drivePID(16, 100, 1000);
    chassis.turnToHeading(90, 500);
    drivePID(15.5, 100, 1000);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-5, 60, 600);
    liftToPreset(0, false, 1000, 100);
    chassis.turnToHeading(150, 500);
    chassis.moveToPose(4.541, -23.386, 180 ,1500, {.lead=0.8,.maxSpeed = 100}, true);
    drivePID(2, 20, 500);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPreset(2, true, 1000, 80);
    drivePID(-3, 20, 500);
    chassis.turnToHeading(0, 500, {.maxSpeed = 40});
    drivePID(5, 50, 800);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-5, 127, 500);
    liftToPreset(0, false, 1000, 100);
    chassis.moveToPose(-23.81, 1.138, 180 ,1500, {.forwards = false, .lead = 0.2, .maxSpeed = 100}, true);
    drivePID(4, 20, 500);
    clamp.set_value(HIGH);
    pros::delay(250);
    drivePID(-3, 20, 500);
    liftToPreset(3, true, 500, 40);
    chassis.turnToHeading(0, 1500,{.maxSpeed = 20});
    drivePID(2, 20, 400);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-3, 50, 500);
    liftToPreset(0, false, 1000, 127);
    chassis.moveToPose(-2.214, 1.123, 90 ,1500, {.forwards = false, .lead=0.2,.maxSpeed = 100}, true);
}
void threecup1goal(int i){
    chassis.setPose(0, 0, 0);
    drivePID(-2, 60, 500);
    liftToPreset(1, false, 1000, 127);
    chassis.moveToPose(-23.392, -53.723, 0, 3000, {.forwards = false, .lead = 0.6, .maxSpeed = 100});
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-2, 100, 1000);
    liftToPreset(0, false, 1000, 127);
    chassis.turnToHeading(180, 500);
    drivePID(2, 20, 500);
    clamp.set_value(HIGH);
    pros::delay(250);
    drivePID(-2, 50, 1000);
    chassis.turnToHeading(0, 500, {.maxSpeed = 30});
    liftToPreset(2, true, 1000, 70);
    drivePID(2, 30, 500);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-2, 90, 500);
    liftToPreset(0, false, 1000);
    chassis.moveToPose(-36.263, -53.387, 300, 3000, {.forwards = false, .lead = 0.6, .maxSpeed = 100});
    drivePID(6, 30, 1000);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPreset(3, false, 2000, 40);
    chassis.moveToPose(-29.995, -47.231, 90, 2000, {.lead = 0.2, .maxSpeed = 30});
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-5, 30, 500);
    liftToPreset(0, false, 1000, 127);
    chassis.turnToHeading(-90, 500);
    chassis.moveToPose(-33.353, -33.353, 45, 2000, {.lead = .02});
    drivePID(5, 20, 700);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPreset(3, false, 2000, 40);
    chassis.moveToPose(-23.28, -40.068, 180, 5000, {.forwards = true, .lead = 0.6, .maxSpeed = 40});
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-3, 10, 500);
    liftToPreset(0, false, 1000, 127);
}
void PIDTesting(int i){
    /*drivePID(24, 80, 2000);
    pros::delay(5000);
    chassis.turnToHeading(90, 800);
    pros::delay(3000);
    chassis.turnToHeading(180, 500);\
    */
    chassis.moveToPose(0, 24, 0, 2000);
    chassis.moveToPose(24, 24, 90, 2000);
    pros::delay(5000);
    chassis.moveToPose(0, 0, -90, 5000);

}
void left3Pin(int i){
    chassis.setPose(0,0,0);
    liftToPreset(3, false, 500, 127);
    chassis.moveToPose(15.5, 54, 0, 2100, {.lead=0.002, .minSpeed=120}, false);
    liftToPreset(2, true, 250, 127);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 100, 600);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(180, 550);
    chassis.moveToPose(6.5, 18, 160, 1700,{.lead=0.2, .minSpeed=120}, false);
    chassis.moveToPose(7, -10, 180, 1400,{.lead=0.002, .minSpeed=120}, false);
    clamp.set_value(HIGH);
    pros::delay(300);
    liftToPosition(5,200, 127);
    drivePID(-6, 100, 500);
    chassis.turnToHeading(0, 550);
    liftToPreset(3, false, 500, 127);
    chassis.moveToPose(17, 56.5, 0, 2300, {.lead=0.002, .minSpeed=120}, false);
    pros::delay(300);
    clamp.set_value(LOW);
    pros::delay(300);
    drivePID(-6, 100, 500);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(180, 800);
    chassis.moveToPose(6.5, -11, 180, 2200,{.lead=.002, .minSpeed=120}, false);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPosition(5,200, 127);
    drivePID(-6, 100, 500);
    chassis.turnToHeading(0, 550);
    liftToPreset(4, false, 500, 127);
    chassis.moveToPose(15, 60 , 0, 2300, {.lead=0.002, .minSpeed=120}, false);
    pros::delay(300);
    clamp.set_value(LOW);
    drivePID(-6, 100, 500);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(180, 550);
    chassis.moveToPose(6.5, 5, 180, 1800,{.lead=0.002, .minSpeed=120}, false);
    chassis.moveToPose(8, -10, 180, 1400,{.lead=0.002, .maxSpeed=90}, false);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPosition(5,200, 127);
    drivePID(-12, 100, 1000);
    liftToPreset(1, false, 500, 127);
    chassis.turnToHeading(90, 450);
    drivePID(28, 100, 2000);
    clamp.set_value(LOW);
}
void left3PinMidRush(int i){
    chassis.setPose(0,0,0);
    liftToPreset(3, false, 500, 127);
    chassis.moveToPose(15.5, 54, 0, 2100, {.lead=0.002, .minSpeed=120}, false);
    liftToPreset(2, true, 250, 127);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 100, 600);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(180, 550);
    chassis.moveToPose(6.5, 18, 160, 1700,{.lead=0.2, .minSpeed=120}, false);
    chassis.moveToPose(7, -10, 180, 1400,{.lead=0.002, .minSpeed=120}, false);
    clamp.set_value(HIGH);
    pros::delay(300);
    liftToPosition(8,200, 127);
    drivePID(-6, 100, 800);
    chassis.turnToHeading(50, 800);
    liftToPreset(4, false, 500, 127);
    chassis.moveToPose(68, 61, 55, 2000, {.lead=0.2, .minSpeed=110}, false);
    pros::delay(250);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 100, 2000);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(-125, 500);
    chassis.moveToPose(6, -10, 180, 2500,{.lead=0.002, .minSpeed=120}, false);
    pros::delay(250);
    clamp.set_value(HIGH);
    pros::delay(400);
    drivePID(-6, 100, 800);
    chassis.turnToHeading(0, 500);
    liftToPreset(3, false, 500, 127);
    chassis.moveToPose(16, 55, 0, 2000, {.lead=0.2, .minSpeed=110}, false);
    pros::delay(250);
    clamp.set_value(LOW);

}
// Mirror of left3Pin, every x and heading is flipped
void right3Pin(int i){
    chassis.setPose(0,0,0);
    liftToPreset(3, false, 500, 127);
    chassis.moveToPose(-15, 52, 0, 2100, {.lead=0.002, .minSpeed=120}, false);
    liftToPreset(2, true, 250, 127);
    clamp.set_value(LOW);
    pros::delay(250);
    drivePID(-6, 100, 600);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(180, 550);
    chassis.moveToPose(-6.5, 18, -160, 1700,{.lead=0.2, .minSpeed=120}, false);
    chassis.moveToPose(-6, -18, 180, 1400,{.lead=0.002, .minSpeed=120}, false);
    clamp.set_value(HIGH);
    pros::delay(300);
    liftToPosition(5,200, 127);
    drivePID(-6, 100, 500);
    chassis.turnToHeading(0, 550);
    liftToPreset(3, false, 500, 127);
    chassis.moveToPose(-15, 56.5, 0, 2300, {.lead=0.002, .minSpeed=120}, false);
    pros::delay(300);
    clamp.set_value(LOW);
    pros::delay(300);
    drivePID(-6, 100, 500);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(180, 800);
    chassis.moveToPose(-5, -18, 180, 2200,{.lead=.002, .minSpeed=120}, false);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPosition(5,200, 127);
    drivePID(-6, 100, 500);
    chassis.turnToHeading(0, 550);
    liftToPreset(4, false, 500, 127);
    chassis.moveToPose(-15, 60 , 0, 2300, {.lead=0.002, .minSpeed=120}, false);
    pros::delay(300);
    clamp.set_value(LOW);
    drivePID(-6, 100, 500);
    liftToPreset(0, false, 500, 127);
    chassis.turnToHeading(180, 550);
    chassis.moveToPose(-6.5, 5, 180, 1800,{.lead=0.002, .minSpeed=120}, false);
    chassis.moveToPose(-8, -10, 180, 1400,{.lead=0.002, .maxSpeed=90}, false);
    clamp.set_value(HIGH);
    pros::delay(250);
    liftToPosition(5,200, 127);
    drivePID(-12, 100, 1000);
    liftToPreset(1, false, 500, 127);
    chassis.turnToHeading(-90, 450);
    drivePID(28, 100, 2000);
    clamp.set_value(LOW);
}
