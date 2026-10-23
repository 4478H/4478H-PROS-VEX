#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

extern int autonSection;


// side = 1 for Quadrant 1 (Alliance Station wall), -1 for Quadrant 2 (side wall)
void toggleOnly(int side);
void togglePreload(int side);
void togglePreloadStack(int side);
void awpStack(int side);
void redLoaderSideOneGoal(int i);
void loaderSide2Goal(int i);
void PIDTesting(int i);
void leftSideOneGoal(int i);
void left3Pin(int i);
void left3PinMidRush(int i);
void right3Pin(int i);

#endif // AUTON_ROUTES_H