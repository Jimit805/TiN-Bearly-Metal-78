#include "Auto.h"
#include "Elevator.h"
#include "constants.h"

const unsigned long AUTO_DURATION = 15000; // 15 seconds

void handleAuto(NoU_Drivetrain& drivetrain, NoU_Motor& elevator, RobotState& state) {
    unsigned long currentTime = millis();

    // End auto after time limit
    if (currentTime - state.autoStartTime >= AUTO_DURATION) {
        state.autoStep = 5;
        state.robotMode = teleOP;
        return;
    }

    switch (state.autoStep) {
        case 0: // Drive forward briefly
            state.wristAngle = 0;
            drivetrain.holonomicDrive(0, -0.4, 0);
            if (currentTime - state.autoStartTime > 750) {
                drivetrain.holonomicDrive(0, 0, 0);
                state.autoStep = 1;
                state.stepStart = currentTime;
            }
            break;

        case 1: // Raise arm to scoring position
            state.armAngle = 110;
            if (currentTime - state.stepStart > 1000) {
                state.autoStep = 2;
                state.stepStart = currentTime;
            }
            break;

        case 2: // Raise elevator to L4
            handleElevator(elevator, state);
            state.elevatorTarget = CORAL_B_L4;
            state.elevatorUseSetpoint = true;
            if (currentTime - state.stepStart > 1000) {
                state.autoStep = 3;
                state.stepStart = currentTime;
            }
            break;

        case 3: // Drive into reef
            drivetrain.holonomicDrive(0, -0.4, 0);
            if (currentTime - state.stepStart > 400) {
                drivetrain.holonomicDrive(0, 0, 0);
                state.autoStep = 4;
                state.stepStart = currentTime;
            }
            break;

        case 4: // Eject coral
            state.coralIntakeThrottle = -1.0f;
            state.middleIntakeThrottle = -1.0f;
            if (currentTime - state.stepStart > 1000) {
                state.coralIntakeThrottle = 0.0f;
                state.middleIntakeThrottle = 0.0f;
                state.autoStep = 5;
            }
            break;

        case 5: // Stop everything
        default:
            drivetrain.holonomicDrive(0, 0, 0);
            state.coralIntakeThrottle = 0;
            state.middleIntakeThrottle = 0;
            break;
    }
}
