#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include "RobotState.h"

void initElevator(NoU_Motor& elevator);

void handleElevator(NoU_Motor& elevator, RobotState& state);

#endif
