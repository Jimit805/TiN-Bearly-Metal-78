#include "Elevator.h"
#include "constants.h"

void initElevator(NoU_Motor& elevator) {
    elevator.beginEncoder();
    elevator.setInverted(false);
    elevator.setBrakeMode(true);
}

void handleElevator(NoU_Motor& elevator, RobotState& state) {
    // Stow
    if (PestoLink.buttonHeld(RIGHT_BUMPER)) {
        state.elevatorTarget      = 0;
        state.elevatorUseSetpoint = true;
    }

    // Setpoints
    if (state.gamePiece == coral) { // Coral
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            state.elevatorTarget      = CORAL_INTAKE;
            state.elevatorUseSetpoint = true;
        }
        if (state.scoreSide == front) { // Front
            if      (PestoLink.buttonHeld(BUTTON_TOP))    { state.elevatorTarget = CORAL_F_L4; state.elevatorUseSetpoint = true; } // L4
            else if (PestoLink.buttonHeld(BUTTON_LEFT))   { state.elevatorTarget = CORAL_F_L3; state.elevatorUseSetpoint = true; } // L3
            else if (PestoLink.buttonHeld(BUTTON_RIGHT))  { state.elevatorTarget = CORAL_F_L2; state.elevatorUseSetpoint = true; } // L2
            else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) { state.elevatorTarget = CORAL_F_L1; state.elevatorUseSetpoint = true; } // L1
        } else { // Back
            if      (PestoLink.buttonHeld(BUTTON_TOP))    { state.elevatorTarget = CORAL_B_L4; state.elevatorUseSetpoint = true; } // L4
            else if (PestoLink.buttonHeld(BUTTON_LEFT))   { state.elevatorTarget = CORAL_B_L3; state.elevatorUseSetpoint = true; } // L3
            else if (PestoLink.buttonHeld(BUTTON_RIGHT))  { state.elevatorTarget = CORAL_B_L2; state.elevatorUseSetpoint = true; } // L2
            else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) { state.elevatorTarget = CORAL_B_L1; state.elevatorUseSetpoint = true; } // L1
        }
    } else { // Algae
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            state.elevatorTarget      = ALGAE_INTAKE;
            state.elevatorUseSetpoint = true;
        }
        if      (PestoLink.buttonHeld(BUTTON_TOP))    { state.elevatorTarget = ALGAE_NET;       state.elevatorUseSetpoint = true; } // Net
        else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) { state.elevatorTarget = ALGAE_PROCESSOR; state.elevatorUseSetpoint = true; } // Processor
        else if (PestoLink.buttonHeld(BUTTON_RIGHT))  { state.elevatorTarget = ALGAE_LOW;       state.elevatorUseSetpoint = true; } // Low Intake
        else if (PestoLink.buttonHeld(BUTTON_LEFT))   { state.elevatorTarget = ALGAE_HIGH;      state.elevatorUseSetpoint = true; } // High Intake
    }

    // --- PID control ---
    if (state.elevatorUseSetpoint) {
        long  currentPos = elevator.getPosition();
        long  error = state.elevatorTarget - currentPos;

        state.elevatorIntegral += error;
        float derivative = error - state.elevatorPrevError;

        state.elevatorPower = -1.0f * ((kP * error) + (kI * state.elevatorIntegral) + (kD * derivative));
        state.elevatorPower = constrain(state.elevatorPower, -0.9f, 0.9f);

        // Stop integrating and driving when close enough
        if (abs(error) < 40) {
            state.elevatorPower = 0;
            state.elevatorIntegral = 0;
        }

        elevator.set(state.elevatorPower);
        state.elevatorPrevError = error;
    } else {
        elevator.set(state.elevatorThrottle);
        state.elevatorIntegral = 0;
        state.elevatorPrevError = 0;
    }

    // Encoder reset
    if (PestoLink.buttonHeld(R_PRESS)) {
        elevator.resetPosition();
    }
}
