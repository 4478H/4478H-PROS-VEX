#include "auton_selector.h"
#include "auton_routes.h"
#include "devices.h"

struct AutonRoutine {
    std::string displayName;
    std::function<void(int)> func;
    int parameter = 1; // passed to func, the side for Override routes
};

// Alliance color, true = red
bool red = false;

AutonSelector::AutonSelector(const AutonRoutine* routinesArray, size_t routineCount, bool combineTesting,
                             const AutonRoutine* extraRoutinesArray, size_t extraCount) {
    // Route the robot starts on, numbered from 1 (4 = the 4th route in the list)
    currentSelection = 1;

    for (size_t i = 0; i < routineCount; i++) {
        if (routinesArray[i].func == nullptr) {
            pros::lcd::clear_line(3);
            pros::lcd::print(3, "Routine %d has null function", i);
        }
        routines.push_back(routinesArray[i]);
    }

    if (combineTesting && extraRoutinesArray != nullptr) {
        for (size_t i = 0; i < extraCount; i++) {
            if (extraRoutinesArray[i].func == nullptr) {
                pros::lcd::clear_line(3);
                pros::lcd::print(3, "Extra routine %d has null function", i);
            }
            routines.push_back(extraRoutinesArray[i]);
        }
    }
}

void AutonSelector::displaySelectionBrain() {
    if (currentSelection < 1 || currentSelection > routines.size()) {
        pros::lcd::clear_line(7);
        pros::lcd::print(7, "Invalid selection: %i", currentSelection);
        return;
    }
    pros::lcd::clear_line(7);
    pros::lcd::print(7, "Alliance: %s", red ? "Red" : "Blue");
    pros::lcd::clear_line(6);
    pros::lcd::print(6, "%s", routines[currentSelection - 1].displayName.c_str());
}

void AutonSelector::prevSelection() {
    currentSelection = (currentSelection - 2 + routines.size()) % routines.size() + 1;
}

void AutonSelector::nextSelection() {
    currentSelection = currentSelection % routines.size() + 1;
}

void AutonSelector::runSelection() {
    if (currentSelection < 1 || currentSelection > routines.size()) {
        pros::lcd::clear_line(7);
        pros::lcd::print(7, "Invalid selection: %d", currentSelection);
        return;
    }

    const AutonRoutine& selectedRoutine = routines[currentSelection - 1];
    if (selectedRoutine.func) {
        selectedRoutine.func(selectedRoutine.parameter);
    }
}

int AutonSelector::getRoutineCount() const {
    return routines.size();
}

// Routes that show up in the selector, in order
// Format: {name shown on the brain, route function, parameter}
const AutonRoutine COMPETITION_ROUTINES[] = {
    // Override routes: 1 = start on your Alliance Station wall (Q1), -1 = start on the side wall (Q2)
    // Red and blue use the same routes
    /*
    {"Red Loader Side", redLoaderSideOneGoal, 1},
    {"Loader Side 2 Goal", loaderSide2Goal, 1},
    {"Q1 Toggle + Preload", togglePreload, 1},
    {"Q2 Toggle + Preload", togglePreload, -1},
    {"Q1 Preload + Stack", togglePreloadStack, 1},
    {"Q2 Preload + Stack", togglePreloadStack, -1},
    // only one robot per Alliance can run AWP Stack, see auton_routes.cpp
    {"Q1 AWP Stack", awpStack, 1},
    {"Q2 AWP Stack", awpStack, -1},
    {"Toggle Only", toggleOnly, 1},
    {"left side", leftSideOneGoal, 1},
    */
    {"PID Testing", PIDTesting, 1},
    {"left 3 pin", left3Pin, 1},
    {"right 3 pin", right3Pin, 1},
    {"left 3 pin mid rush", left3PinMidRush, 1},
    {"Skills", skills, 1},
    {"left alliance", leftAlliance, 1},
};

AutonSelector competitionSelector(COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));
