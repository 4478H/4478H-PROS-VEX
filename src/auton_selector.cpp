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
    currentSelection =1;  // Initialize to first selection

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
    // Q1 = start on your Alliance Station wall, Q2 = start on the side wall
    // Red and blue use the same routes
    {"Red Loader Side", redLoaderSideOneGoal, 1},
    {"Loader Side 2 Goal", loaderSide2Goal, 1},
    {"Q1 Toggle + Preload", togglePreload, 1},
    {"Q2 Toggle + Preload", togglePreload, -1},
    {"Q1 Preload + Stack", togglePreloadStack, 1},
    {"Q2 Preload + Stack", togglePreloadStack, -1},
    // only one robot per Alliance can run AWP Stack, see auton_routes.cpp
    {"Q1 AWP Stack", awpStack, 1},
    {"Q2 AWP Stack", awpStack, -1},
    {"Toggle Only", toggleOnly, 1}

};


const bool isTestingCombined = false;

AutonSelector competitionSelector(COMPETITION_ROUTINES, sizeof(COMPETITION_ROUTINES) / sizeof(COMPETITION_ROUTINES[0]));