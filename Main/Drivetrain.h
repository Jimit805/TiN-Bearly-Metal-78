#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include "RobotState.h"

void initDrivetrain(NoU_Drivetrain& drivetrain,
                    NoU_Motor& frontLeftMotor, NoU_Motor& frontRightMotor,
                    NoU_Motor& rearLeftMotor, NoU_Motor& rearRightMotor);

void handleDrivetrain(NoU_Drivetrain& drivetrain, RobotState& state);

#endif
