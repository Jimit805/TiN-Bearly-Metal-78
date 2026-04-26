#include "Drivetrain.h"
#include "constants.h"
#include <math.h>

void initDrivetrain(NoU_Drivetrain& drivetrain, NoU_Motor& frontLeftMotor, NoU_Motor& frontRightMotor, NoU_Motor& rearLeftMotor, NoU_Motor& rearRightMotor) {
    // Invert
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    // Brake modes
    frontLeftMotor.setBrakeMode(true);
    frontRightMotor.setBrakeMode(true);
    rearLeftMotor.setBrakeMode(true);
    rearRightMotor.setBrakeMode(true);

    // Drivetrain tuning
    drivetrain.setMinimumOutput(0.2);
    drivetrain.setMaximumOutput(1.0);
    drivetrain.setDeadband(0.1);
    drivetrain.setExponent(1.375);
}

void handleDrivetrain(NoU_Drivetrain& drivetrain, RobotState& state) {
    // Recalibrate IMU and zero yaw on mid-right button press
    if (PestoLink.buttonHeld(MID_RIGHT)) {
        NoU3.calibrateIMUs();
        NoU3.yaw = 0;
        return;
    }

    // Tuning: rotate robot in place 5 times, enter the measured yaw angle below
    const float measured_angle = 27.5;
    const float angular_scale = (5.0 * 2.0 * PI) / measured_angle;

    float fieldPowerX = PestoLink.getAxis(0);
    float fieldPowerY = -PestoLink.getAxis(1);
    float rotationPower = PestoLink.getAxis(2);

    // Convert gyro heading to field-centric robot frame
    float heading = NoU3.yaw * angular_scale;
    float cosA = cos(heading);
    float sinA = sin(heading);
    float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
    float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
}
