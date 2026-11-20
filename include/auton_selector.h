#ifndef AUTON_SELECTOR_H
#define AUTON_SELECTOR_H

#include <string>
#include <vector>
#include <functional>

// A route name, the function that runs it, and the int passed to it
// Defined in auton_selector.cpp
struct AutonRoutine;

// Holds a list of routes and which one is picked
// Selections are numbered from 1, not 0
class AutonSelector
{
private:
    std::vector<AutonRoutine> routines;
    int currentSelection;

public:
    // Copies the routes in, and the extra ones too if combineTesting is true
    AutonSelector(const AutonRoutine *routinesArray, size_t routineCount, bool combineTesting = false,
                  const AutonRoutine *extraRoutinesArray = nullptr, size_t extraCount = 0);

    // Show the alliance color and selected route on the brain screen
    void displaySelectionBrain();

    // Step through the list, wrapping around at either end
    void prevSelection();
    void nextSelection();

    // Run the selected route
    void runSelection();

    int getRoutineCount() const;
};

extern const AutonRoutine COMPETITION_ROUTINES[];
extern AutonSelector competitionSelector;

// Alliance color, true = red, toggled with the brain's center button
// The brain's left/right buttons step through the routes
extern bool red;

#endif // AUTON_SELECTOR_H
