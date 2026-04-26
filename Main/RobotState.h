#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

// Robot Modes
enum State { teleOP, autoOne };
enum Side { front, back };
enum Piece { coral, algae };

struct RobotState {
    // Mode
    State robotMode = teleOP;
    Side scoreSide = back;
    Piece gamePiece = coral;

    // Arm / Wrist
    int armAngle = 35;
    int lastArmAngle  = 0;
    int wristAngle = 10;

    // Intake throttles (written by Intake, applied in main loop)
    float coralIntakeThrottle = 0;
    float middleIntakeThrottle = 0;
    float algaeIntakeThrottle = 0;

    // Elevator
    bool elevatorUseSetpoint = false;
    long elevatorTarget = 0;
    float elevatorPower = 0;
    float elevatorThrottle = 0;

    // Elevator PID state
    float elevatorIntegral = 0;
    float elevatorPrevError = 0;

    // Auto timing
    long autoStartTime = 0;
    int autoStep = 0;
    unsigned long stepStart = 0;

    // Debug / print throttle
    unsigned long lastPrintTime = 0;
};

#endif
