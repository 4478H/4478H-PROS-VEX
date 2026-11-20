#ifndef AUTON_ROUTES_H
#define AUTON_ROUTES_H

// Every route takes one int so the selector can call them all the same way
// Override routes: side = 1 for Quadrant 1 (Alliance Station wall), -1 for Quadrant 2 (side wall)
void toggleOnly(int side);
void togglePreload(int side);
void togglePreloadStack(int side);
void awpStack(int side);

// Coordinate routes, the parameter is unused
void redLoaderSideOneGoal(int i);
void leftSideOneGoal(int i);
void loaderSide2Goal(int i);
void threecup1goal(int i);
void left3Pin(int i);
void left3PinMidRush(int i);
void right3Pin(int i);
void PIDTesting(int i);
void skills(int i);
void leftAlliance(int i);

#endif // AUTON_ROUTES_H
