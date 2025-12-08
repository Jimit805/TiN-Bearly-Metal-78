#ifndef CONSTANTS
#define CONSTANTS

// Buttons
constexpr int BUTTON_BOTTOM = 0;
constexpr int BUTTON_RIGHT = 1; 
constexpr int BUTTON_LEFT = 2;
constexpr int BUTTON_TOP = 3;
constexpr int LEFT_BUMPER = 4;
constexpr int RIGHT_BUMPER = 5;
constexpr int LEFT_TRIGGER = 6;
constexpr int RIGHT_TRIGGER = 7;
constexpr int MID_RIGHT = 9;
constexpr int MID_LEFT = 8;
constexpr int L_PRESS = 10;
constexpr int R_PRESS = 11;
constexpr int D_UP = 12;
constexpr int D_DOWN = 13;
constexpr int D_LEFT = 14;
constexpr int D_RIGHT = 15;

// Elevator Setpoints
constexpr long CORAL_INTAKE = 0;
constexpr long CORAL_F_L1 = 0;
constexpr long CORAL_F_L2 = -855;
constexpr long CORAL_F_L3 = -2000;
constexpr long CORAL_F_L4 = -2750;
constexpr long CORAL_B_L1 = 0;
constexpr long CORAL_B_L2 = 0;
constexpr long CORAL_B_L3 = -830;
constexpr long CORAL_B_L4 = -2665;
constexpr long ALGAE_INTAKE = 0;
constexpr long ALGAE_NET = -2700;
constexpr long ALGAE_PROCESSOR = 0;
constexpr long ALGAE_HIGH = -1800;
constexpr long ALGAE_LOW = -600;


// Elevator PID
constexpr float kP = 0.008;
constexpr float kI = 0;
constexpr float kD = 0.003;

// Arm
const int ARM_SERVO_OFFSET = 180 + 22;

#endif
