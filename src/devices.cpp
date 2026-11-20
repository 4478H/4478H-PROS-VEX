#include "devices.h"

// Negative port numbers mean the motor is reversed

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ---------------- Motors ----------------
// Drivetrain, 2 motors per side, blue (600 rpm) cartridges
pros::MotorGroup right_motors({7, 6}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({-9, -8}, pros::MotorGearset::blue);
pros::MotorGroup all_motors({-8, -9, 6, 7}, pros::MotorGearset::blue);
// Single drive motors, drivePID reads each one to take the median
pros::Motor mBackLeft(-9, pros::MotorGearset::blue);
pros::Motor mMidLeft(-8, pros::MotorGearset::blue);
pros::Motor mBackRight(7, pros::MotorGearset::blue);
pros::Motor mMidRight(6, pros::MotorGearset::blue);

// Lift, green (200 rpm) cartridges
pros::Motor liftLeft{-18, pros::MotorGearset::green};
pros::Motor liftRight{10, pros::MotorGearset::green};
pros::MotorGroup Lift({10, -18}, pros::MotorGearset::green);

// Intake
pros::Motor bottomStage{16, pros::MotorGearset::blue};
pros::Motor topStage{2, pros::MotorGearset::green};
pros::MotorGroup intake({16, 2});
// Toggle
pros::Motor toggleMotor(-17, pros::MotorGearset::green);

// ---------------- Sensors ----------------
pros::Imu imu(21);
// WARNING: port 8 is also the left middle drive motor, move this before using color sort
pros::Optical colorSens(8);
pros::Distance backDistance(16);
// adi::DigitalIn autonLimitSwitch('G'); // not on the robot right now  

// ---------------- Pneumatics ----------------
adi::Port toggle('B', E_ADI_DIGITAL_OUT);
adi::Port clamp('C', E_ADI_DIGITAL_OUT);
adi::Port wing('H', E_ADI_DIGITAL_OUT);
adi::Port midDescore('A', E_ADI_DIGITAL_OUT);

// ---------------- LemLib setup ----------------
Drivetrain drivetrain(&left_motors,  // left motor group
                      &right_motors, // right motor group
                      11,            // track width, in inches
                      3.25,          // wheel diameter, in inches
                      360,           // drivetrain rpm
                      1.5            // horizontal drift
);

// The drive motors double as vertical tracking wheels
lemlib::TrackingWheel left_tracking_wheel(&left_motors, drivetrain.wheelDiameter, -drivetrain.trackWidth / 2.0f, drivetrain.rpm);
lemlib::TrackingWheel right_tracking_wheel(&right_motors, drivetrain.wheelDiameter, drivetrain.trackWidth / 2.0f, drivetrain.rpm);

// Horizontal tracking wheel on a rotation sensor, 4" behind the center
pros::Rotation horizontal_rotation(13);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, -4);

OdomSensors sensors(&left_tracking_wheel,       // vertical tracking wheel 1 (left drive)
                    &right_tracking_wheel,      // vertical tracking wheel 2 (right drive)
                    &horizontal_tracking_wheel, // horizontal tracking wheel 1
                    nullptr,                    // horizontal tracking wheel 2
                    &imu                        // inertial sensor
);

// Lateral (driving straight) PID controller
ControllerSettings lateral_controller(11,  // proportional gain (kP)
                                      0,   // integral gain (kI)
                                      3,   // derivative gain (kD)
                                      3,   // anti windup
                                      0.1, // small error range, in inches
                                      100, // small error range timeout, in milliseconds
                                      0.5, // large error range, in inches
                                      500, // large error range timeout, in milliseconds
                                      20   // maximum acceleration (slew)
);

// Angular (turning) PID controller
ControllerSettings angular_controller(3.5,  // proportional gain (kP)
                                      0,    // integral gain (kI)
                                      19.2, // derivative gain (kD)
                                      5,    // anti windup
                                      0.1,  // small error range, in degrees
                                      250,  // small error range timeout, in milliseconds
                                      0.3,  // large error range, in degrees
                                      250,  // large error range timeout, in milliseconds
                                      0     // maximum acceleration (slew)
);

// Input curve for throttle during driver control
ExpoDriveCurve throttle_curve(3,    // joystick deadband out of 127
                              0,    // minimum output where drivetrain will move out of 127
                              1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,            // odometry sensors
                        &throttle_curve);
