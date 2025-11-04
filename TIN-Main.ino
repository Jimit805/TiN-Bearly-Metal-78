#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>


// Buttons
#define BUTTON_BOTTOM 0
#define BUTTON_RIGHT 1
#define BUTTON_LEFT 2
#define BUTTON_TOP 3
#define LEFT_BUMPER 4
#define RIGHT_BUMPER 5
#define LEFT_TRIGGER 6
#define RIGHT_TRIGGER 7
#define MID_LEFT 8
#define MID_RIGHT 9
#define L_PRESS 10
#define R_PRESS 11
#define D_UP 12
#define D_DOWN 13
#define D_LEFT 14
#define D_RIGHT 15

// Motors and Servos
NoU_Motor frontLeftMotor(5);
NoU_Motor frontRightMotor(4);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(1);
NoU_Servo armServo(1);
NoU_Servo wristServo(2);
NoU_Motor elevator(7); // Integrated encodered support
NoU_Motor coralIntake(6);
NoU_Motor middleIntake(2);
NoU_Motor algaeIntake(3);

// Variables
int armAngle = 0;
int wristAngle = 0;
float coralIntakeThrottle = 0;
float middleIntakeThrottle = 0;
float algaeIntakeThrottle = 0;
bool elevatorUseSetpoint = false;
long elevatorTarget = 0;
float elevatorPower = 0;
float elevatorThrottle = 0;

// Elevator PID
float kP = 0.0005;
float kI = 0.0000005;
float kD = 0.0004;

// PID state variables
float elevatorIntegral = 0;
float elevatorPrevError = 0;

// Elevator setpoints
const long CORAL_INTAKE = 0;
const long CORAL_F_L1 = 0;
const long CORAL_F_L2 = 500;
const long CORAL_F_L3 = 750;
const long CORAL_F_L4 = 1000;
const long CORAL_B_L1 = 0;
const long CORAL_B_L2 = 500;
const long CORAL_B_L3 = 750;
const long CORAL_B_L4 = 1000;
const long ALGAE_INTAKE = 100;
const long ALGAE_NET = 1000;
const long ALGAE_PROCESSOR = 0;

// Robot Modes
enum State{teleOP, autoOne};
State robotMode = teleOP;

// Robot scoring side mode
enum Side{front, back};
Side scoreSide = back;

// Game piece mode
enum Piece{coral, algae};
Piece gamePiece = coral;

// Timings for auto
long autoStartTime = 0;
const unsigned long AUTO_DURATION = 15000; // 15 seconds

// Initialize drivetrain
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

void setup() {
    PestoLink.begin("TIN");
    Serial.begin(115200);

    NoU3.begin();
    
    NoU3.calibrateIMUs();

    drivetrain.setMinimumOutput(0.2);
    drivetrain.setMaximumOutput(1.0);
    drivetrain.setDeadband(0.08);
    drivetrain.setExponent(1.375);

    // Inverted Motors
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);
    //frontLeftMotor.setInverted(true);
    //rearLeftMotor.setInverted(true);

    // Brake Modes
    frontLeftMotor.setBrakeMode(true);
    frontRightMotor.setBrakeMode(true);
    rearLeftMotor.setBrakeMode(true);
    rearRightMotor.setBrakeMode(true);
    elevator.setBrakeMode(true);
    coralIntake.setBrakeMode(true);
    middleIntake.setBrakeMode(true);
    algaeIntake.setBrakeMode(true);

}

unsigned long lastPrintTime = 0;

void loop() {
    
    // Auto timing
    unsigned long currentTime = millis();

    // Set robot mode
    if (robotMode == teleOP && PestoLink.buttonHeld(MID_LEFT)) {
        robotMode = autoOne;
        autoStartTime = millis(); // record start time
    }

    // Run the robot mode
    if (PestoLink.update()) {
        if (robotMode == autoOne) {
            handleAuto();
        } else {
            handleTeleOP();
        }
    }

    // Battery Voltage stuff
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if (PestoLink.isConnected()) {
        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.setServiceLight(LIGHT_DISABLED);
    }

    // Set motor speeds and servo angles
    armServo.write(armAngle);
    wristServo.write(wristAngle);
    coralIntake.set(coralIntakeThrottle);
    middleIntake.set(middleIntakeThrottle);
    algaeIntake.set(algaeIntakeThrottle);

}

void handleTeleOP() {
    Drivetrain();
    Arm();
    Elevator();
    ManualArm();
    CoralIntake();
    AlgaeIntake();

    // Toggle Scoring side
    if (PestoLink.buttonHeld(D_LEFT) && scoreSide == back) {
        scoreSide = front;
    } else if (PestoLink.buttonHeld(D_LEFT) && scoreSide == front) {
        scoreSide = back;
    }

    // Toggle game piece
    if (PestoLink.buttonHeld(D_RIGHT) && gamePiece == coral) {
        gamePiece = algae;
    } else if (PestoLink.buttonHeld(D_RIGHT) && gamePiece == algae) {
        gamePiece = coral;
    }

}

void Drivetrain() {
    
    // Tuning procedure: 
    // Rotate the robot in place 5 times, set the measured angle from output console to "measured_angle"
    float measured_angle = 27.5;
    float angular_scale = (5.0*2.0*PI) / measured_angle;

    if (lastPrintTime + 100 < millis()) {
        PestoLink.printfTerminal("Gyro Yaw: %.3f\r\n",  NoU3.yaw);
        lastPrintTime = millis();
    }

    float fieldPowerX = PestoLink.getAxis(0);
    float fieldPowerY = -PestoLink.getAxis(1);
    float rotationPower = PestoLink.getAxis(2);

    // Get robot heading (in radians) from the gyro
    float heading = NoU3.yaw * angular_scale;

    // Rotate joystick vector to be robot-centric
    float cosA = cos(heading);
    float sinA = sin(heading);

    float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
    float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

    //set motor power
    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

    if (PestoLink.buttonHeld(MID_RIGHT)) {
        NoU3.calibrateIMUs();
        NoU3.yaw = 0;
        return;
    }

}

void Elevator() {
    
    // Manual Elevator Control
    if (PestoLink.buttonHeld(LEFT_BUMPER)) {
        elevatorThrottle = 0.5;
    } else if (PestoLink.buttonHeld(RIGHT_BUMPER)){
        elevatorThrottle = -0.5;
    } else {
        elevatorThrottle = 0;
    }

    // Setpoints
    if (gamePiece == coral) { // Coral Setpoints
        
        // Coral Intake
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            elevatorTarget = CORAL_INTAKE;
            elevatorUseSetpoint = true;
        }

        // Scoring Setpoints
        if (scoreSide == front) {
            if (PestoLink.buttonHeld(BUTTON_TOP)) {
                elevatorTarget = CORAL_F_L4;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                elevatorTarget = CORAL_F_L3;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                elevatorTarget = CORAL_F_L2;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
                elevatorTarget = CORAL_F_L1;
                elevatorUseSetpoint = true;
            }
        } else if (scoreSide == back) {
            if (PestoLink.buttonHeld(BUTTON_TOP)) {
                elevatorTarget = CORAL_B_L4;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                elevatorTarget = CORAL_B_L3;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                elevatorTarget = CORAL_B_L2;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
                elevatorTarget = CORAL_B_L1;
                elevatorUseSetpoint = true;
            }
        }

    } else if (gamePiece == algae) { // Algae Setpoints
        
        // Algae Intake
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            elevatorTarget = ALGAE_INTAKE;
            elevatorUseSetpoint = true;
        }
        
        if (PestoLink.buttonHeld(BUTTON_TOP)) {
            elevatorTarget = ALGAE_NET;
            elevatorUseSetpoint = true;
        } else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
            elevatorTarget = ALGAE_PROCESSOR;
            elevatorUseSetpoint = true;
        }
    }

    // PID Control
    if (elevatorUseSetpoint) {
        long currentPos = elevator.getPosition();
        long error = elevatorTarget - currentPos;

        elevatorIntegral += error;
        float derivative = error - elevatorPrevError;

        elevatorPower = (kP * error) + (kI * elevatorIntegral) + (kD * derivative);
        elevatorPower = constrain(elevatorPower, -0.9, 0.9);

        // Stop when near target
        if (abs(error) < 50) {
            elevatorPower = 0;
            elevatorIntegral = 0;
        }

        elevator.set(elevatorPower);
        elevatorPrevError = error;

        PestoLink.printfTerminal("Elevator | Pos:%ld Target:%ld Power:%.2f\r\n",
                                 currentPos, elevatorTarget, elevatorPower);
    } else {
        elevator.set(elevatorThrottle);
        elevatorIntegral = 0;
        elevatorPrevError = 0;
    }

    // Reset Encoder
    if (PestoLink.buttonHeld(R_PRESS)) {
        elevator.resetPosition();
    }
}


void Arm() {

    // Setpoints
    if (gamePiece == coral) { // Coral Setpoints

        // Coral Intake
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            armAngle = 0;
            wristAngle = 0;
        } else {
            armAngle = 60;
            wristAngle = 90;
        }
        
        // Coral Scoring
        if (scoreSide == back) { // Score coral back
            if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 105; // L4
                wristAngle = 90;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 110; // L3
                wristAngle = 90;
            } else if (PestoLink.buttonHeld(BUTTON_TOP)) {
                armAngle = 135; // L2
                wristAngle = 90;
            }
        } else if (scoreSide == front) { // Score coral front
            if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 60; // L4
                wristAngle = 100;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 48; // L3
                wristAngle = 100;
            } else if (PestoLink.buttonHeld(BUTTON_TOP)) {
                armAngle = 30; // L2
                wristAngle = 100;
            } else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
                armAngle = 45; // L1
                wristAngle = -60;
            }
        }


    } else if (gamePiece == algae) { // Algae Setpoints

        // Algae Intake
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            armAngle = 25;
            wristAngle = -40;
        } else {
            armAngle = 60;
            wristAngle = 90;
        }

        if (scoreSide == back) { // Back Algae Intake
            if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 95; // Low Algae
                wristAngle = 105;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 95; // High Algae
                wristAngle = 105;
            }
        } else if (scoreSide == front) { // Front Algae Intake
            if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 45; // Low Algae
                wristAngle = -35;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 60; // High Algae
                wristAngle = -45;
            }
        }

        // Algae Scoring
        if (PestoLink.buttonHeld(BUTTON_TOP)) {
            armAngle = 70; // Net 
            wristAngle = -15;
        } else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
            armAngle = 0; // Processor 
            wristAngle = 20;
        }

    }

}

void ManualArm() {

    /*
    // Manual Arm control
    if (PestoLink.buttonHeld(D_DOWN) && armAngle > 0) {
        armAngle--;
        delay(50);
    }
  
    if (PestoLink.buttonHeld(D_UP) && armAngle < 105) {
        armAngle++;
        delay(50);
    }

    // Manual Wrist Control
    if (PestoLink.buttonHeld(D_RIGHT) && wristAngle > 0) {
        wristAngle--;
        delay(50);
    }
  
    if (PestoLink.buttonHeld(D_LEFT) && wristAngle < 95) {
        wristAngle++;
        delay(50);
    }
    */
}

void CoralIntake() {
    
    // Control the rollers for intaking and scoring coral
    if (gamePiece = coral) {
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            coralIntakeThrottle = 1;
            middleIntakeThrottle = 1;
        } else if (PestoLink.buttonHeld(RIGHT_TRIGGER)) {
            coralIntakeThrottle = -1;
            middleIntakeThrottle = -1;
        } else {
            coralIntakeThrottle = 0;
            middleIntakeThrottle = 0;
        }
    }
}

void AlgaeIntake() {

    // Control the algae rollers for intaking and scoring algae
    if (gamePiece = algae) {
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            algaeIntakeThrottle = 1;
            middleIntakeThrottle = -1;
        } else if (PestoLink.buttonHeld(RIGHT_TRIGGER)) {
            algaeIntakeThrottle = -1;
            middleIntakeThrottle = 1;
        } else {
            algaeIntakeThrottle = 0;
            middleIntakeThrottle = 0;
        }
    }
}

void handleAuto() {

    // Handle the timing and robot mode
    unsigned long currentTime = millis();

    // If 15 seconds have passed since auto started, switch to teleop
    if (currentTime - autoStartTime >= AUTO_DURATION) {
        robotMode = teleOP;
        PestoLink.printfTerminal("Robot Mode: %s", robotMode);
        return;
    }

    // Actual auto code


}
