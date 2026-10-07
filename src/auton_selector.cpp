#include "auton_selector.h"
#include "auton_routes.h"
#include "devices.h"
#include <iostream>

// Define the AutonRoutine structure
struct AutonRoutine {
    std::string displayName;
    std::function<void(int)> func;
    int parameter = 1;
};

// Global variable for color/alliance selection
bool red = false;

// Constructor
AutonSelector::AutonSelector(const AutonRoutine* routinesArray, size_t routineCount, bool combineTesting, const AutonRoutine* extraRoutinesArray, size_t extraCount) {
    currentSelection =3;  // Initialize to first selection

    // Add main routines
    for (size_t i = 0; i < routineCount; i++) {
        if (routinesArray[i].func == nullptr) {
            pros::lcd::clear_line(3);
            pros::lcd::print(3, "Routine %d has null function", i);
        }
        routines.push_back(routinesArray[i]);
    }

    // Add extra routines if required
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

// Method implementations
void AutonSelector::displaySelectionBrain() {
    if (currentSelection < 1 || currentSelection > routines.size()) {
        pros::lcd::clear_line(7);
        pros::lcd::print(7, "Invalid selection: %i", currentSelection);
        return;
    }
    pros::lcd::clear_line(7);
    pros::lcd::print(7, "Alliance: %s", red ? "Red" : "Blue");
    pros::lcd::clear_line(6);
    pros::lcd::print(6, "%s",routines[currentSelection - 1].displayName.c_str());
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

// Global object definitions
const AutonRoutine COMPETITION_ROUTINES[] = {                        
    {"Left 7 Ball Push", left7Push, 1},
    {"Right 7 Ball Push", right7Push, 1},
    {"Full AWP RIGHT SIDE", fullAWPLeft,1},
    {"Odom AWP Right", odomAWPHigh,1},
    {"skills", skills,1},
    {"PID Testing", PIDTesting,1},
    {"skillsNew", skillsNew,1}

};


const bool isTestingCombined = false;

AutonSelector competitionSelector(COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));