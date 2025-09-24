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
NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);
NoU_Servo armServo(1);
NoU_Servo wristServo(2);
NoU_Motor elevator(5);
NoU_Motor coralIntake(6);
NoU_Motor middleIntake(7);
NoU_Motor algaeIntake(8);

// Brake Modes
frontLeftMotor.setBrakeMode(true);
frontRightMotor.setBrakeMode(true);
rearLeftMotor.setBrakeMode(true);
rearRightMotor.setBrakeMode(true);
elevator.setBrakeMode(true);
coralIntake.setBrakeMode(true);
middleIntake.setBrakeMode(true);
algaeIntake.setBrakeMode(true);

// Variables
int armAngle = 0;
int wristAngle = 0;
float elevatorThrottle = 0;
float coralIntakeThrottle = 0;
float algaeIntakeThrottle = 0;

// Robot Modes
enum State{teleOP, autoOne};
State robotMode = teleOP;

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

    drivetrain.setDeadband(0.08);
    drivetrain.setExponent(1.7);

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
}

void handleTeleOP() {
    Drivetrain();
    Arm();
    Elevator();
    ManualArm();
    CoralIntake();
    AlgaeIntake();
}

void Drivetrain() {
    
    // Tuning procedure: 
    // Rotate the robot in place 5 times, set the measured angle from output console to "measured_angle"
    float measured_angle = 31.416;
    float angular_scale = (5.0*2.0*PI) / measured_angle;

    if (lastPrintTime + 100 < millis()) {
        PestoLink.printfTerminal("Gyro Yaw: %.3f\r\n",  NoU3.yaw);
        lastPrintTime = millis();
    }

    float fieldPowerX = -PestoLink.getAxis(1);
    float fieldPowerY = PestoLink.getAxis(0);
    float rotationPower = -PestoLink.getAxis(2);

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

}

void Arm() {

}

void ManualArm() {

}

void CoralIntake() {

}

void AlgaeIntake() {

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