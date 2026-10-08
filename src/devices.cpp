#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "api.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "devices.h"
#include "auton_routes.h"
#include "movement.h"
#include <string>
#include <iostream>
#include <thread>
using namespace pros;
using namespace lemlib;




pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Motor Declerations and Port Configurations
pros::MotorGroup right_motors({7, 6}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({-9, -8}, pros::MotorGearset::blue);
pros::MotorGroup all_motors({-8, -9, 6, 7}, pros::MotorGearset::blue);
pros::Motor liftLeft{-20, pros::MotorGearset::green};
pros::Motor liftRight{10, pros::MotorGearset::green};
pros::MotorGroup Lift({10, -20}, pros::MotorGearset::green);
pros::Motor bottomStage{16, pros::MotorGearset::blue};
pros::Motor topStage{2, pros::MotorGearset::green};
pros::MotorGroup intake({16, 2});
pros::Motor mBackLeft(-9, pros::MotorGearset::blue);
pros::Motor mMidLeft(-8, pros::MotorGearset::blue);
pros::Motor mBackRight(7, pros::MotorGearset::blue);
pros::Motor mMidRight(6, pros::MotorGearset::blue);
// Sensor Declerations and Configurations
pros::Imu imu(18);
pros::Optical colorSens(8);
pros::Distance backDistance(17); 
Distance* backDistancePtr = &backDistance;
// Phnematic Declerations and Configurations
adi::Port hood('B', E_ADI_DIGITAL_OUT);
adi::Port clamp('C', E_ADI_DIGITAL_OUT);
adi::Port wing('H', E_ADI_DIGITAL_OUT);
adi::Port midDescore('A', E_ADI_DIGITAL_OUT);
adi::DigitalIn autonLimitSwitch('G');



// drivetrain settings
Drivetrain drivetrain(&left_motors,  // left motor group
  &right_motors, // right motor group
  11.375,          // 11 inch track width
  3.25,          // using new 2.75" omnis
  360,           // drivetrain rpm is 360
  1.5            // horizontal drift is 8 (center traction wheel drivebase)
);

lemlib::TrackingWheel left_tracking_wheel(&left_motors, drivetrain.wheelDiameter, -drivetrain.trackWidth / 2.0f, drivetrain.rpm);
lemlib::TrackingWheel right_tracking_wheel(&right_motors, drivetrain.wheelDiameter, drivetrain.trackWidth / 2.0f, drivetrain.rpm);

OdomSensors sensors(&left_tracking_wheel,  // vertical tracking wheel 1 (left drive)
                    &right_tracking_wheel, // vertical tracking wheel 2 (right drive)
                    nullptr,               // horizontal tracking wheel 1
                    nullptr,               // horizontal tracking wheel 2
                    &imu                   // inertial sensor
);

// lateral PID controller
ControllerSettings lateral_controller(10,  // proportional gain (kP)
                                      0,   // integral gain (kI)
                                      3,   // derivative gain (kD)
                                      3,   // anti windup
                                      0.1, // small error range, in inches
                                      100, // small error range timeout, in milliseconds
                                      0.5, // large error range, in inches
                                      500, // large error range timeout, in milliseconds
                                      20   // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(2.75, // proportional gain (kP)
                                      0, // integral gain (kI)
                                      19.5,  // derivative gain (kD)
                                      5,   // anti windup
                                      0.1, // small error range, in inches
                                      250, // small error range timeout, in milliseconds
                                      0.3, // large error range, in inches
                                      250, // large error range timeout, in milliseconds
                                      0    // maximum acceleration (slew)
);

// input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(3,    // joystick deadband out of 127
                              0,    // minimum output where drivetrain will move out of 127
                              1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
  lateral_controller, // lateral PID settings
  angular_controller, // angular PID settings
  sensors,            // odometry sensors
  &throttle_curve);