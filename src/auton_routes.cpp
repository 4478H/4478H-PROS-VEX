#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "auton_routes.h"
#include "devices.h"
#include "movement.h"
#include "opticalAlign.h"
#include "liftControl.h"





void fullAWPLeft(int i){
liftToPreset(3, 2000, 127);
}
void fullAWPRight(int i){
chassis.setPose(0,0,180);
clamp.set_value(HIGH);
Intake(1);
drivePID(30.1, 100, 3000);
chassis.turnToHeading(270, 900);
drivePID(10.2, 100, 1000);
drivePID(-2, 50, 500);
drivePID(2,50, 500);
pros::delay(200);
drivePID(-28.064, 100, 3000);
hood.set_value(HIGH);
clamp.set_value(LOW);
pros::delay(500);
hood.set_value(LOW);
chassis.turnToHeading(18.954, 700);
drivePID(23.13, 70, 3000);
chassis.turnToHeading(0, 1000);
drivePID(48.837, 70, 3000);
chassis.turnToHeading(316.685, 700);
drivePID(-15.628, 100, 2000);
stopIntake();
scoreMid();
pros::delay(50);
Intake(1);
drivePID(15.782, 100, 2000);
chassis.moveToPose(25.854, 40.659, 316, 5000);
chassis.moveToPose(3.977, 63.861, 270, 5000);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(14.142, 100, 2000);
drivePID(-2, 50, 500);
drivePID(2,50, 500);
pros::delay(200);
drivePID(-28.726, 100, 3000);
hood.set_value(HIGH);
}
void odomAWP(int i){
    
}
void odomAWPHigh(int i){
drivePID(4, 20,3000);
}
void skills(int i){
chassis.setPose(0,0,180);
//Go To MatchLoader
wing.set_value(HIGH);
Intake(1);
drivePID(30.5, 50, 3000);
chassis.turnToHeading(270, 1500);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(8, 50, 2500);
drivePID(-2, 50, 500);
drivePID(2,50, 500);
chassis.setPose(26.9,-4.2,chassis.getPose().theta);
pros::delay(500);
drivePID(-10, 50, 1000);
//Go To Other Side
/*
drivePID(-19.423, 100, 3000);
clamp.set_value(LOW);
chassis.turnToHeading(180, 1500);
drivePID(14.5, 50, 5000);
chassis.turnToHeading(90, 1500);
*/
chassis.moveToPose(43.6, -18, 273, 3500, {.forwards = false}, false);
drivePID(-82, 50, 6000);
//Go To Score
/*
chassis.turnToHeading(180, 1500);
drivePID(-11, 50, 3500);
chassis.turnToHeading(90, 1500);
drivePID(-16, 50 ,3000);
*/
chassis.moveToPose(90, -3.8, 90, 4000, {.forwards = false}, false);
outake(200);
Intake(1);
hood.set_value(HIGH);
pros::delay(1200);
hood.set_value(LOW);
chassis.turnToHeading(90, 500);
//Go To Loader 2
clamp.set_value(HIGH);
pros::delay(200);
chassis.turnToHeading(90, 1500);
drivePID(29, 50, 3000);
drivePID(-2, 50, 500);
drivePID(2,50, 500);
pros::delay(500);
chassis.turnToHeading(90, 1500);
//Go To Score 2
//drivePID(-30, 50, 5000);
chassis.moveToPose(80, -1, 90, 4000, {.forwards = false}, false);
outake(200);
Intake(1);
clamp.set_value(LOW);
hood.set_value(HIGH);
pros::delay(1200);
hood.set_value(LOW);
chassis.setPose(80, -1, chassis.getPose().theta);
//Get Mid Balls
load();
chassis.turnToHeading(-8, 1000);
drivePID(30, 50, 2000);
chassis.turnToHeading(0, 500);
drivePID(40, 50, 3500);
drivePID(-4.7, 50, 2000);
chassis.turnToHeading(230, 1000);
drivePID(20, 50, 3000);
outake(1000);
chassis.moveToPose(200, 30, 90, 4000, {.forwards = false}, false);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(6, 100, 1500);
pros::delay(200);
clamp.set_value(LOW);
hood.set_value(HIGH);
pros::delay(700);
hood.set_value(LOW);
chassis.moveToPose(67.838, 77.119, 270, 5000);
chassis.turnToHeading(270, 1000);
drivePID(61.872, 100, 4000);
chassis.turnToHeading(180, 1000);
drivePID(14.142, 100, 2000);
chassis.turnToHeading(270, 1000);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(17.679, 100, 3000);
drivePID(-2, 50, 500);
drivePID(2,50, 500);
pros::delay(200);
drivePID(-29.39, 100, 3000);
clamp.set_value(LOW);
hood.set_value(HIGH);
chassis.moveToPose(-12.153, 43.31, 188, 5000);
chassis.turnToHeading(188, 1000);
drivePID(12.938, 100, 2000);
Intake(-1);
chassis.turnToHeading(180, 100);
drivePID(19.004, 100, 2000);
}
void skillsNew(int i){
chassis.setPose(-47.226,-17.298,0);
Intake(1);
clamp.set_value(HIGH);
chassis.moveToPose(-47.226,-47.129,0,3000,{.minSpeed = 127}, false);
chassis.turnToHeading(270, 1000);
drivePID(10, 100, 2000);
pros::delay(500);
chassis.moveToPose(-26.234, -60.166, 270, 3000, {.forwards = false}, false);
clamp.set_value(LOW);
chassis.moveToPose(30.114,-47.129,90,3000,{.forwards = false,.lead=0.4,.minSpeed = 127}, false);
hood.set_value(HIGH);
pros::delay(500);
hood.set_value(LOW);
chassis.setPose(30.114,-47.129,chassis.getPose().theta);
clamp.set_value(HIGH);
chassis.moveToPose(58.177,-47.129,90,3000,{.forwards = false,.lead=0.4,.minSpeed = 127}, false);
pros::delay(500);
chassis.moveToPose(30.114,-47.129,90,3000,{.forwards = false,.lead=0.4,.minSpeed = 127}, false);
clamp.set_value(LOW);
hood.set_value(HIGH);
pros::delay(500);
hood.set_value(LOW);
chassis.turnToHeading(-15, 800,{.maxSpeed = 70});
chassis.moveToPose(23.043,-23.485,0,3000,{.forwards = false,.lead=0.4,.minSpeed = 127}, false);
chassis.moveToPose(23.706, 23.14, 0, 3000, {.forwards = false,.lead=0.4,.minSpeed = 60}, true);
clamp.set_value(HIGH);
pros::delay(500);
clamp.set_value(LOW);
chassis.turnToHeading(220, 800);
chassis.moveToPose(23.706,23.14, 220, 3000);
outake(800);
chassis.moveToPose(46.908, 46.784,90, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
chassis.turnToHeading(90, 1000);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(11, 100, 1500);
pros::delay(500);
chassis.setPose(58.84, 46.784, chassis.getPose().theta);
chassis.moveToPose(26.137,60.484,90, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
clamp.set_value(LOW);
chassis.moveToPose(-32.642,60.705,90, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
chassis.moveToPose(-29.501,47.107,270, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
hood.set_value(HIGH);
pros::delay(800);
hood.set_value(LOW);
clamp.set_value(HIGH);
chassis.setPose(-29.501,47.107,chassis.getPose().theta);
chassis.moveToPose(-58.104,46.486,270, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
pros::delay(500);
chassis.moveToPose(-29.501,47.107,270, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
clamp.set_value(LOW);
hood.set_value(HIGH);
pros::delay(800);
hood.set_value(LOW);
chassis.setPose(-29.501,47.107,chassis.getPose().theta);   
chassis.moveToPose(-53.268,29.814,210, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
chassis.moveToPose(-61.835,15.706,180, 3000, {.forwards = false,.lead=0.4,.minSpeed = 127}, false);
clamp.set_value(HIGH);
drivePID(20, 100, 2000);

}
void leftPush(int i){
wing.set_value(HIGH);
chassis.setPose(0,0,0);
Intake(1);
drivePID(29.3, 100, 3000);
chassis.turnToHeading(270, 1000);
clamp.set_value(HIGH);
drivePID(11.8, 100, 2500);
//alignToLongGoal();
drivePID(-33, 100, 1800);
clamp.set_value(LOW);
outake(200);
hood.set_value(HIGH);
Intake(1);
pros::delay(900);
hood.set_value(LOW);
chassis.turnToHeading(160, 1500);
drivePID(29.5,40, 3000);
chassis.turnToHeading(180, 1000);
drivePID(-12.5, 80, 3000);
chassis.turnToHeading(-38, 1000);
drivePID(-31, 70, 2000);
stopIntake();
scoreMid();
pros::delay(300);
stopIntake();
chassis.turnToHeading(-60, 1000);
drivePID(30.552, 80, 2000);
chassis.moveToPose(31.5, 13.4, 90, 5000);
}
void rightPush(int i){
chassis.setPose(0,0,180);
Intake(1);
drivePID(31, 70, 3000);
chassis.turnToHeading(270, 1000);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(9, 100, 2000);
chassis.turnToHeading(270, 1000);
drivePID(-33.5, 100, 2000);
//alignToLongGoal();
outake(200);
Intake(1);
clamp.set_value(LOW);
hood.set_value(HIGH);
pros::delay(1700);
hood.set_value(LOW);
chassis.turnToHeading(8,1000,{},false);
load();
drivePID(26.064, 70, 2000);
drivePID(-7, 80, 500);
chassis.turnToHeading(42.614, 1000);
Intake(1);
drivePID(17.013, 100, 3000);
pros::delay(100);
drivePID(4.552, 100, 3000);

}
void PIDTesting(int i){

chassis.setPose(0,0,0);
/*
chassis.turnToHeading(90,1000);
pros::delay(1000);
chassis.turnToHeading(180, 1000);
pros::delay(1000);
chassis.turnToHeading(270, 1000);
pros::delay(1000);
chassis.turnToHeading(0, 1000);
pros::delay(1500);
*/
//drivePID(24, 100, 3000);
//drivePID(70, 100, 10000);

alignToLongGoal();
}
void left4_3Long(int i){
chassis.setPose(-48.331,17.077,70);
load();
chassis.moveToPose(-22.698,23.043,110,3000,{.minSpeed = 127}, true);
pros::delay(500);
clamp.set_value(HIGH);
pros::delay(200);
clamp.set_value(LOW);
chassis.turnToHeading(132, 700);
chassis.moveToPose(-11.309,11.396,132,3000,{.minSpeed = 127}, true);
outake(600);
Intake(1);
chassis.moveToPose(-44.795,47.129,270,3000,{.forwards = false,.lead=0.4,.minSpeed = 127}, false);
turnToHeadingSmart(270, 500);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(25, 100, 3000);
pros::delay(200);
chassis.moveToPose(-29.548, 47.129, 270, 3000, {.forwards = false, .lead = 0.4, .minSpeed = 127}, false);
hood.set_value(HIGH);
pros::delay(500);   
hood.set_value(LOW);
}
void right4_3Long(int i){
chassis.setPose(-48.331,-17.077,110);
load();
chassis.moveToPose(-22.698,-23.043,70,3000,{.minSpeed = 127}, true);
pros::delay(500);
clamp.set_value(HIGH);
pros::delay(200);
clamp.set_value(LOW);
chassis.turnToHeading(48, 700);
chassis.moveToPose(-11.309,-11.396,48,3000,{.minSpeed = 127}, true);
outake(400);
Intake(1);
chassis.moveToPose(-44.795,-47.129,270,3000,{.forwards = false,.lead=0.4,.minSpeed = 127}, false);
turnToHeadingSmart(270, 500);
clamp.set_value(HIGH);
pros::delay(200);
drivePID(25, 100, 3000);
pros::delay(200);
chassis.moveToPose(-29.548, -47.129, 270, 3000, {.forwards = false, .lead = 0.4, .minSpeed = 127}, false);
hood.set_value(HIGH);
pros::delay(500);   
hood.set_value(LOW);
descore();
}

void right7Push(int i){
chassis.setPose(-45.458, -14.867, 90);
Intake(1);
wing.set_value(HIGH);
chassis.moveToPose(-21.388, -21.69, 110,1100, {.lead=0.4,.minSpeed = 127}, true);
pros::delay(500);
clamp.set_value(HIGH);
pros::delay(200);
chassis.moveToPose(-47.889, -47.35, 270, 1000, {.lead=0.2,.minSpeed = 100}, false);
Intake(-1);
pros::delay(700);
Intake(1);
chassis.turnToHeading(270, 1000);
drivePID(30, 100, 3000);
pros::delay(200);
//drivePID(-30, 100, 2000);
chassis.moveToPose(13, 0, 270, 3000, {.forwards = false, .lead = 0.4, .minSpeed = 127}, false);
hood.set_value(HIGH);
pros::delay(2000);
drivePID(10, 100, 2000);
hood.set_value(LOW);
drivePID(-10, 100, 2000);	
//descore();
}
void left7Push(int i){
chassis.setPose(-45.458, 14.743, 90);
Intake(1);
wing.set_value(HIGH);
chassis.moveToPose(-22.27, 22.531, 70 ,1100, {.lead=0.4,.minSpeed = 127}, true);
pros::delay(500);
clamp.set_value(HIGH);
pros::delay(200);
chassis.moveToPose(-47.149,51, 272, 1000, {.lead=0.2,.minSpeed = 100}, false);
chassis.turnToHeading(270,500);
drivePID(-6.5, 100, 1000);
hood.set_value(HIGH);
outake(200);
Intake(1);
pros::delay(1200);
hood.set_value(LOW);
/*turnToHeadingSmart(273, 500);
drivePID(32, 50, 2500);
pros::delay(200); 
turnToHeadingSmart(269,500);

chassis.moveToPose(-30, 16.3, 270, 3000, {.forwards = true, .lead = 0.4, .minSpeed = 127}, false);
pros::delay(700);
//drivePID(-29, 100, 1500);
//chassis.moveToPose(-10.6,16.3,270,3000,{.forwards = false,.lead=0.4,.minSpeed = 127}, false);
outake(500);
hood.set_value(HIGH);
Intake(1);
pros::delay(400);
hood.set_value(LOW);
//descoreLeft();
*/
}
void soloTap(int i){
chassis.setPose(-46, 0, 0);
Intake(1);
wing.set_value(HIGH);
drivePID(10, 100, 500);
chassis.moveToPose(-46,-47.129,0,1800,{.forwards = false,.lead=0.0},false);
clamp.set_value(HIGH);
turnToHeadingSmart(270,700);
drivePID(13, 100, 1000);
pros::delay(200);
drivePID(-30, 100, 2000);
clamp.set_value(LOW);
hood.set_value(HIGH);
pros::delay(300);
hood.set_value(LOW);
chassis.setPose(-30.432,-47.129,chassis.getPose().theta);
chassis.turnToPoint(-23,24,700,{},false);
drivePID(25, 100, 1500);
chassis.moveToPose(-22.27,24.793,0,2000,{.lead=0.4,.minSpeed = 127}, false);
clamp.set_value(HIGH);
pros::delay(300);
clamp.set_value(LOW);
chassis.turnToHeading(-42,800);
drivePID(-20, 100, 2000);
scoreMid();
pros::delay(200);
stopIntake();
chassis.setPose(-12.179,13.31,chassis.getPose().theta);
chassis.moveToPose(-43.691,46.784,270,2000,{.lead=0.4,.minSpeed = 127}, false);
Intake(1);
clamp.set_value(HIGH);
drivePID(16, 100, 2000);
pros::delay(200);
drivePID(-30,100,2000);
hood.set_value(HIGH);
}
