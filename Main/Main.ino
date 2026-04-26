#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>

#include "constants.h"
#include "RobotState.h"
#include "Drivetrain.h"
#include "Elevator.h"
#include "Arm.h"
#include "Intake.h"
#include "Auto.h"

// Hardware
NoU_Motor frontLeftMotor(5);
NoU_Motor frontRightMotor(4);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(1);

NoU_Servo armServo(3); // 50g servo
NoU_Servo wristServo(4); // 9g servo

NoU_Motor elevator(6); // Encoder-integrated
NoU_Motor coralIntake(2); // Double motor
NoU_Motor middleIntake(3); // Middle roller
NoU_Motor algaeIntake(7); // Top roller

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor,  &rearRightMotor);

// Shared robot state
RobotState state;


void setup() {
    Serial.begin(115200);
    PestoLink.begin("TIN");
    NoU3.begin();
    NoU3.calibrateIMUs();

    initDrivetrain(drivetrain, frontLeftMotor, frontRightMotor, rearLeftMotor,  rearRightMotor);
    initElevator(elevator);
    initIntake(coralIntake, middleIntake, algaeIntake);
}


void loop() {
    // Mode switching (edge-detected)
    static bool midLeftPrev = false;
    bool midLeftCurr = PestoLink.buttonHeld(MID_LEFT);
    if (state.robotMode == teleOP && midLeftCurr && !midLeftPrev) {
        state.robotMode = autoOne;
        state.autoStartTime = millis();
        state.autoStep = 0;
    }
    midLeftPrev = midLeftCurr;

    // Run the active mode
    if (PestoLink.update()) {
        if (state.robotMode == autoOne) {
            handleAuto(drivetrain, elevator, state);
        } else {
            handleTeleOP();
        }
    }

    // Apply outputs
    applyArm(armServo, wristServo, state);
    applyIntake(coralIntake, middleIntake, algaeIntake, state);

    // Battery / status light
    PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());
    NoU3.setServiceLight(PestoLink.isConnected() ? LIGHT_ENABLED : LIGHT_DISABLED);
}

// Tele OP
void handleTeleOP() {
    handleDrivetrain(drivetrain, state);
    handleArm(state);
    handleElevator(elevator, state);
    handleCoralIntake(state);
    handleAlgaeIntake(state);

    // Toggle scoring side (D-Left)
    static bool dLeftPrev = false;
    bool dLeftCurr = PestoLink.buttonHeld(D_LEFT);
    if (dLeftCurr && !dLeftPrev) {
        state.scoreSide = (state.scoreSide == back) ? front : back;
        PestoLink.printfTerminal(state.scoreSide == front ? "Front Side" : "Back Side");
    }
    dLeftPrev = dLeftCurr;

    // Toggle game piece (D-Right)
    static bool dRightPrev = false;
    bool dRightCurr = PestoLink.buttonHeld(D_RIGHT);
    if (dRightCurr && !dRightPrev) {
        state.gamePiece = (state.gamePiece == coral) ? algae : coral;
        PestoLink.printfTerminal(state.gamePiece == algae ? "Algae" : "Coral");
    }
    dRightPrev = dRightCurr;

    // Debug telemetry (every 100 ms)
    if (millis() - state.lastPrintTime > 100) {
        PestoLink.printfTerminal(
            "Heading: %.2f | Elev: %ld -> %ld | %s | %s | A: %ld | W: %ld \r\n",
            NoU3.yaw,
            elevator.getPosition(),
            state.elevatorTarget,
            (state.gamePiece == coral ? "Coral" : "Algae"),
            (state.scoreSide == back ? "Back"  : "Front"),
            state.armAngle,
            state.wristAngle
        );
        state.lastPrintTime = millis();
    }
}
