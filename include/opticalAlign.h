#ifndef __OPTICAL_ALIGN__
#define __OPTICAL_ALIGN__

// Align to a long goal using the back distance sensor
// Sweeps a turn until it sees the goal, then backs into it while holding that heading
extern void alignToLongGoal();

// Same thing for the long goal on the opposite side of the field (scans around 70-150 degrees)
extern void alignToLongGoalOpposite();

#endif
