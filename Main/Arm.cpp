#include "Arm.h"
#include "constants.h"

void handleArm(RobotState& state) {
    // Stow
    if (PestoLink.buttonHeld(RIGHT_BUMPER)) {
        state.armAngle = 35;
        state.wristAngle = 0;
    }

    if (state.gamePiece == coral) { // coral
        // Coral intake position
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            state.armAngle = 0;
            state.wristAngle = 110;
        }
        // Coral scoring positions
        if (state.scoreSide == back) {
            if      (PestoLink.buttonHeld(BUTTON_TOP))    { state.armAngle = 105; state.wristAngle = 0; }  // L4
            else if (PestoLink.buttonHeld(BUTTON_LEFT))   { state.armAngle = 110; state.wristAngle = 0; }  // L3
            else if (PestoLink.buttonHeld(BUTTON_RIGHT))  { state.armAngle = 135; state.wristAngle = 0; }  // L2
        } else { // front
            if      (PestoLink.buttonHeld(BUTTON_TOP))    { state.armAngle = 60; state.wristAngle = 0;   } // L4
            else if (PestoLink.buttonHeld(BUTTON_LEFT))   { state.armAngle = 48; state.wristAngle = 0;   } // L3
            else if (PestoLink.buttonHeld(BUTTON_RIGHT))  { state.armAngle = 30; state.wristAngle = 0;   } // L2
            else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) { state.armAngle = 45; state.wristAngle = 135; } // L1
        }
    } else { // algae
        // Algae ground intake
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            state.armAngle   = 18;
            state.wristAngle = 190;
        }
        // Algae reef intake
        if (state.scoreSide == back) {
            if      (PestoLink.buttonHeld(BUTTON_RIGHT)) { state.armAngle = 125; state.wristAngle = 0;  }  // Low
            else if (PestoLink.buttonHeld(BUTTON_LEFT))  { state.armAngle = 105; state.wristAngle = 0;  }  // High
        } else { // front
            if      (PestoLink.buttonHeld(BUTTON_RIGHT)) { state.armAngle = 45; state.wristAngle = 155; }  // Low
            else if (PestoLink.buttonHeld(BUTTON_LEFT))  { state.armAngle = 60; state.wristAngle = 165; }  // High
        }
        // Algae scoring
        if      (PestoLink.buttonHeld(BUTTON_TOP))     { state.armAngle = 70; state.wristAngle = 130; }    // Net
        else if (PestoLink.buttonHeld(BUTTON_BOTTOM))  { state.armAngle = 0;  state.wristAngle = 120; }    // Processor
    }
}

void applyArm(NoU_Servo& armServo, NoU_Servo& wristServo, RobotState& state) {
    // Only write arm servo when angle has changed (reduces jitter)
    if (state.armAngle != state.lastArmAngle) {
        armServo.write(ARM_SERVO_OFFSET - state.armAngle);
        state.lastArmAngle = state.armAngle;
    }
    wristServo.write(state.wristAngle);
}
