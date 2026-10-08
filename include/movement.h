#ifndef __MOVEMENT__
#define __MOVEMENT__

// required files for devices
#include "main.h" 
#include "lemlib/chassis/chassis.hpp"

// namespace for declarations
using namespace pros;

extern void Intake(double=1);//-1 for outake, 1 for intake
extern void scoreMid();
extern void load();
extern void descore();
extern void descoreLeft();
extern void stopIntake();
extern void outake(int time);
extern void outakeSkills(int time);
extern void scoreMidSkills();
extern double slewStep;
extern double slew(double, double);
extern double drive_kP;
extern double drive_kI;
extern double drive_kD;
// fwdVal: target distance (or degrees for turn)
// maxSpeedPercent: optional speed cap in percent (0-100). Default 100 = full power (127)
extern void drivePID(double, double maxSpeedPercent = 100.0, double timeout = 3000);
// Override helpers
extern void grabPin();
extern void releasePin();
// backupDist: inches to back up into the Toggle
extern void setToggle(double backupDist = 6);
// preset: lift height from liftControl.cpp, approachDist: inches to drive in over the Goal
extern void placePin(int preset, double approachDist = 6);
extern void turnToHeadingSmart(float theta, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);


#endif