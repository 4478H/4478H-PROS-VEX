#ifndef __LIFT_CONTROL__
#define __LIFT_CONTROL__

#include "devices.h"

// Runs once at the start of the program, drives the lift down until it
// stalls against the hard stop, then zeroes the encoders at that point
extern void homeLift();

// Call this every cycle in opcontrol, handles hold to move, gravity hold
// on release, the soft limit going down, and L2 return to zero
extern void handleLift();

// Blocking full PID move to an exact encoder position, for use in autonomous
// timeoutMs is a safety cutoff in case the lift never reaches target
// maxSpeed caps how fast it is allowed to move, out of 127
extern void liftToPosition(double target, int timeoutMs = 2000, int maxSpeed = 127);

// Moves to a preset height by number, edit the actual values in liftControl.cpp
// preset 0 is the bottom, higher numbers go up from there
// waitUntilDone true = wait for the lift before running the next line
// waitUntilDone false = start the lift and keep going while it moves
extern void liftToPreset(int preset, bool waitUntilDone = true, int timeoutMs = 2000, int maxSpeed = 127);


// Current lift height in encoder degrees, average of both lift motors
extern double getLiftPosition();

#endif