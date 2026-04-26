#include "Intake.h"
#include "constants.h"

void initIntake(NoU_Motor& coralIntake, NoU_Motor& middleIntake, NoU_Motor& algaeIntake) {
    coralIntake.setInverted(true);
    coralIntake.setBrakeMode(true);
    middleIntake.setBrakeMode(true);
    algaeIntake.setBrakeMode(true);
}

void handleCoralIntake(RobotState& state) {
    if (state.gamePiece != coral) return;

    if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
        state.coralIntakeThrottle  = 0.25;
        state.middleIntakeThrottle = 0.4;
    } else if (PestoLink.buttonHeld(RIGHT_TRIGGER)) {
        // Eject direction depends on scoring side
        float dir = (state.scoreSide == back) ? -1.0f : 1.0f;
        state.coralIntakeThrottle = dir;
        state.middleIntakeThrottle = dir;
    } else {
        state.coralIntakeThrottle = 0;
        state.middleIntakeThrottle = 0;
        state.algaeIntakeThrottle = 0;
    }
}

void handleAlgaeIntake(RobotState& state) {
    if (state.gamePiece != algae) return;

    static bool intakeToggled = false;
    static bool leftTriggerPrev = false;

    bool leftTriggerCurr = PestoLink.buttonHeld(LEFT_TRIGGER);
    bool rightTriggerCurr = PestoLink.buttonHeld(RIGHT_TRIGGER);

    // Toggle intake on left-trigger press
    if (leftTriggerCurr && !leftTriggerPrev) {
        intakeToggled = !intakeToggled;
    }
    leftTriggerPrev = leftTriggerCurr;

    if (rightTriggerCurr) {
        state.algaeIntakeThrottle = -1.0f;
        state.middleIntakeThrottle = 1.0f;
    } else if (intakeToggled) {
        state.algaeIntakeThrottle = 0.3f;
        state.middleIntakeThrottle = -0.3f;
    } else {
        state.algaeIntakeThrottle = 0;
        state.middleIntakeThrottle = 0;
    }
}

void applyIntake(NoU_Motor& coralIntake, NoU_Motor& middleIntake, NoU_Motor& algaeIntake, RobotState& state) {
    coralIntake.set(state.coralIntakeThrottle);
    middleIntake.set(state.middleIntakeThrottle);
    algaeIntake.set(state.algaeIntakeThrottle);
}
