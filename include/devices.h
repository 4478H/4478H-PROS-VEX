#pragma once

// Every motor, sensor, and piston on the robot, plus the LemLib chassis setup
// Ports and settings live in devices.cpp

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"

using namespace pros;
using namespace lemlib;

extern Controller controller;

// ---------------- Motors ----------------
// Drivetrain
extern MotorGroup left_motors;
extern MotorGroup right_motors;
extern MotorGroup all_motors;
extern Motor mBackLeft;
extern Motor mMidLeft;
extern Motor mBackRight;
extern Motor mMidRight;
// Lift
extern Motor liftLeft;
extern Motor liftRight;
extern MotorGroup Lift;
// Intake
extern Motor bottomStage;
extern Motor topStage;
extern MotorGroup intake;
// Toggle
extern Motor toggleMotor;

// ---------------- Sensors ----------------
extern Imu imu;
extern Distance backDistance;
extern Optical colorSens;
// extern adi::DigitalIn autonLimitSwitch; // press to cycle through auton routes

// ---------------- Pneumatics ----------------
extern adi::Port toggle; // took over the hood's port (B)
extern adi::Port clamp; // HIGH = open, LOW = closed
extern adi::Port wing;
extern adi::Port midDescore;

// ---------------- LemLib ----------------
extern lemlib::TrackingWheel left_tracking_wheel;
extern lemlib::TrackingWheel right_tracking_wheel;
extern pros::Rotation horizontal_rotation;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::Drivetrain drivetrain;
extern OdomSensors sensors;
extern ControllerSettings lateral_controller;
extern ControllerSettings angular_controller;
extern ExpoDriveCurve throttle_curve;
extern lemlib::Chassis chassis;
