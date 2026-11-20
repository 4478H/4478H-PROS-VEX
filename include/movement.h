#ifndef __MOVEMENT__
#define __MOVEMENT__

#include "main.h"
#include "lemlib/chassis/chassis.hpp"

using namespace pros;

// ---------------- Intake ----------------
extern void Intake(double direction = 1); // 1 for intake, -1 for outtake
extern void scoreMid();
extern void load();
extern void stopIntake();
extern void outake(int time);             // runs the intake out for time ms
extern void outakeSkills(int time);       // slower outtake for skills
extern void scoreMidSkills();

// ---------------- Driving ----------------
// Slew rate limiter used at the start of drivePID
extern double slewStep;
extern double slew(double val, double fwdVal);

// Gains for drivePID
extern double drive_kP;
extern double drive_kI;
extern double drive_kD;

// Drive straight using the drive motor encoders
// fwdVal: distance in inches, negative drives backwards
// maxSpeedPercent: speed cap, 0-100 (100 = full power, 127)
// timeout: give up after this many ms
extern void drivePID(double fwdVal, double maxSpeedPercent = 100.0, double timeout = 3000);

// Same as drivePID, but you choose whether the code after it waits
// waitUntilDone: true = wait for the drive to finish (same as drivePID)
//                false = start the drive and move on to the next line right away
extern void drivePIDAsync(double fwdVal, double maxSpeedPercent = 100.0, double timeout = 3000, bool waitUntilDone = false);

// Pauses until a drivePIDAsync move running in the background has finished
extern void waitForDrive();

// LemLib turnToHeading that adds a minimum speed on small turns so they don't stall
extern void turnToHeadingSmart(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);

// ---------------- Override helpers ----------------
extern void grabPin();
extern void releasePin();
// backupDist: inches to back up into the Toggle
extern void setToggle(double backupDist = 6);
// preset: lift height from liftControl.cpp, approachDist: inches to drive in over the Goal
extern void placePin(int preset, double approachDist = 6);

#endif
