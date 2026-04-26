#ifndef INTAKE_H
#define INTAKE_H

#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include "RobotState.h"

void initIntake(NoU_Motor& coralIntake, NoU_Motor& middleIntake, NoU_Motor& algaeIntake);

// Writes throttle values into state
void handleCoralIntake(RobotState& state);
void handleAlgaeIntake(RobotState& state);

// Applies the current throttle values in state to the motors
void applyIntake(NoU_Motor& coralIntake, NoU_Motor& middleIntake, NoU_Motor& algaeIntake, RobotState& state);

#endif
