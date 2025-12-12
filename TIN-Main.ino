#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <constants.h>

// Motors and Servos
NoU_Motor frontLeftMotor(5);
NoU_Motor frontRightMotor(4);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(1);
NoU_Servo armServo(3); // 50g Servo
NoU_Servo wristServo(4); // 9g Servo
NoU_Motor elevator(6); // Integrated encodered support
NoU_Motor coralIntake(2); // Double Motor
NoU_Motor middleIntake(3); // Middle Roller
NoU_Motor algaeIntake(7); // Top Roller

// Variables
int armAngle = 35;
int lastArmAngle = 0;
int wristAngle = 10;
float coralIntakeThrottle = 0;
float middleIntakeThrottle = 0;
float algaeIntakeThrottle = 0;
bool elevatorUseSetpoint = false;
long elevatorTarget = 0;
float elevatorPower = 0;
float elevatorThrottle = 0;

// PID state variables
float elevatorIntegral = 0;
float elevatorPrevError = 0;

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

    elevator.beginEncoder();

    drivetrain.setMinimumOutput(0.2);
    drivetrain.setMaximumOutput(1.0);
    drivetrain.setDeadband(0.1);
    drivetrain.setExponent(1.375);

    // Inverted Motors
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);
    //frontLeftMotor.setInverted(true);
    //rearLeftMotor.setInverted(true);
    elevator.setInverted(false);
    coralIntake.setInverted(true);


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
    if (armAngle != lastArmAngle) {
        armServo.write(ARM_SERVO_OFFSET - armAngle);
        lastArmAngle = armAngle;
    }
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

    // Previous button states for edge detection
    static bool dLeftPrev = false;
    static bool dRightPrev = false;

    // Current button states
    bool dLeftCurr = PestoLink.buttonHeld(D_LEFT);
    bool dRightCurr = PestoLink.buttonHeld(D_RIGHT);

    // Toggle scoring side  on button press
    if (dLeftCurr && !dLeftPrev) {
        if (scoreSide == back) {
            scoreSide = front;
            PestoLink.printfTerminal("Front Side");
        } else {
            scoreSide = back;
            PestoLink.printfTerminal("Back Side");
        }
    }
    dLeftPrev = dLeftCurr;

    // Toggle game piece on button press
    if (dRightCurr && !dRightPrev) {
        if (gamePiece == coral) {
            gamePiece = algae;
            PestoLink.printfTerminal("Algae");
        } else {
            gamePiece = coral;
            PestoLink.printfTerminal("Coral");
        }
    }
    dRightPrev = dRightCurr;

    if (lastPrintTime + 100 < millis()) {
        PestoLink.printfTerminal("DT: %.2f | Elev: %ld -> %ld | %s | %s | A: %ld | W: %ld \r\n",
            NoU3.yaw,
            elevator.getPosition(),
            elevatorTarget,
            (gamePiece == coral ? "Coral" : "Algae"),
            (scoreSide == back ? "Back" : "Front"),
            armAngle,
            wristAngle
        );
        
        lastPrintTime = millis();
    }

}

void Drivetrain() {
    
    // Tuning procedure: 
    // Rotate the robot in place 5 times, set the measured angle from output console to "measured_angle"
    float measured_angle = 27.5;
    float angular_scale = (5.0*2.0*PI) / measured_angle;

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
    /* if (PestoLink.buttonHeld(LEFT_BUMPER)) {
        elevatorThrottle = 0.5;
    } else if (PestoLink.buttonHeld(RIGHT_BUMPER)){
        elevatorThrottle = -0.5;
    } else {
        elevatorThrottle = 0;
    }
    */

    // Stow
    if (PestoLink.buttonHeld(RIGHT_BUMPER)) {
        elevatorTarget = 0;
        elevatorUseSetpoint = true;
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
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                elevatorTarget = CORAL_F_L3;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
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
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                elevatorTarget = CORAL_B_L3;
                elevatorUseSetpoint = true;
            } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
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
        } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
            elevatorTarget = ALGAE_LOW;
            elevatorUseSetpoint = true;
        } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
            elevatorTarget = ALGAE_HIGH;
            elevatorUseSetpoint = true;
        }

    }

    // PID Control
    if (elevatorUseSetpoint) {
        long currentPos = elevator.getPosition();
        long error = elevatorTarget - currentPos;

        elevatorIntegral += error;
        float derivative = error - elevatorPrevError;

        elevatorPower =  -1 * ((kP * error) + (kI * elevatorIntegral) + (kD * derivative));
        elevatorPower = constrain(elevatorPower, -0.9, 0.9);

        // Stop when near target
        if (abs(error) < 40) {
            elevatorPower = 0;
            elevatorIntegral = 0;
        }

        elevator.set(elevatorPower);
        elevatorPrevError = error;

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

    // Stow
    if (PestoLink.buttonHeld(RIGHT_BUMPER)) {
        armAngle = 35;
        wristAngle = 0;
    }

    // Setpoints
    if (gamePiece == coral) { // Coral Setpoints

        // Coral Intake
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            armAngle = 0;
            wristAngle = 110;
        }

        // Coral Scoring
        if (scoreSide == back) { // Score coral back
            if (PestoLink.buttonHeld(BUTTON_TOP)) {
                armAngle = 105; // L4
                wristAngle = 0;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 110; // L3
                wristAngle = 0;
            } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 135; // L2
                wristAngle = 0;
            }          
        } else if (scoreSide == front) { // Score coral front
            if (PestoLink.buttonHeld(BUTTON_TOP)) {
                armAngle = 60; // L4
                wristAngle = 0;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 48; // L3
                wristAngle = 0;
            } else if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 30; // L2
                wristAngle = 0;
            } else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
                armAngle = 45; // L1
                wristAngle = 135;
            }
        }


    } else if (gamePiece == algae) { // Algae Setpoints

        // Algae Ground Intake
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            armAngle = 18;
            wristAngle = 190;
        }

        // Algae Reef Intake
        if (scoreSide == back) { // Back Algae Intake
            if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 125; // Low Algae
                wristAngle = 0;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 105; // High Algae
                wristAngle = 0;
            }
        } else if (scoreSide == front) { // Front Algae Intake
            if (PestoLink.buttonHeld(BUTTON_RIGHT)) {
                armAngle = 45; // Low Algae
                wristAngle = 155;
            } else if (PestoLink.buttonHeld(BUTTON_LEFT)) {
                armAngle = 60; // High Algae
                wristAngle = 165;
            }
        }

        // Algae Scoring
        if (PestoLink.buttonHeld(BUTTON_TOP)) {
            armAngle = 70; // Net 
            wristAngle = 130;
        } else if (PestoLink.buttonHeld(BUTTON_BOTTOM)) {
            armAngle = 0; // Processor 
            wristAngle = 120;
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
    
    if (gamePiece == coral) {
        if (PestoLink.buttonHeld(LEFT_TRIGGER)) {
            coralIntakeThrottle = 0.25;
            middleIntakeThrottle = 0.4;
        } else if (PestoLink.buttonHeld(RIGHT_TRIGGER)) {
            if (scoreSide == back) {
                coralIntakeThrottle = -1;
                middleIntakeThrottle = -1;
            } else if (scoreSide == front) {
                coralIntakeThrottle = 1;
                middleIntakeThrottle = 1;
            }
        } else {
            coralIntakeThrottle = 0;
            middleIntakeThrottle = 0;
        }
    }
}

void AlgaeIntake() {

    if (gamePiece == algae) {
        static bool intakeToggled = false;
        static bool leftTriggerPrev = false;

        bool leftTriggerCurr = PestoLink.buttonHeld(LEFT_TRIGGER);
        bool rightTriggerCurr = PestoLink.buttonHeld(RIGHT_TRIGGER);

        if (leftTriggerCurr && !leftTriggerPrev) {
            intakeToggled = !intakeToggled;
        }
        leftTriggerPrev = leftTriggerCurr;

        // Determine motor speeds
        if (rightTriggerCurr) {
            algaeIntakeThrottle = -1;
            middleIntakeThrottle = 1;
        } else if (intakeToggled) {
            algaeIntakeThrottle = 0.3;
            middleIntakeThrottle = -0.3;
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
