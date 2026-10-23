#pragma once

// required files for devices
#include "main.h"
#include "api.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include <string>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <numeric>

// namespace for declarations
using namespace pros;
using namespace lemlib;



extern Controller controller;
// Motor Declerations
extern MotorGroup left_motors;
extern MotorGroup right_motors;
extern MotorGroup all_motors;
extern Motor bottomStage;
extern Motor topStage;
extern Motor liftLeft;
extern Motor liftRight;
extern MotorGroup Lift;
extern MotorGroup intake;
extern Motor mBackLeft;
extern Motor mMidLeft;
extern Motor mBackRight;
extern Motor mMidRight;
// Sensor Declerations
extern Imu imu;
extern Distance backDistance;
extern Distance* backDistancePtr;
extern Optical colorSens;
// Phnematic Declerations
extern adi::Port hood;
extern adi::Port clamp;
extern adi::Port wing;
extern adi::Port midDescore;
extern adi::DigitalIn autonLimitSwitch;

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
