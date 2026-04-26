#ifndef ARM_H
#define ARM_H

#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include "RobotState.h"

// Sets armAngle and wristAngle in state
void handleArm(RobotState& state);

// Applies the current angles to the servo objects
void applyArm(NoU_Servo& armServo, NoU_Servo& wristServo, RobotState& state);

#endif
