#ifndef AUTO_CONTROL_H
#define AUTO_CONTROL_H

#include "vex.h"

// PID Defintions
#define KP 0.025
#define KI 0.01
#define KD 0.007
#define LR_KP 0.05
#define WHEELSIZE 2.75    // Inches Diameter
#define TILEDISTANCE (2 * 12) // 2 feet
#define MANUAL_OFFSET 1.3
#define TILEREVOLUTIONS(offset) (TILEDISTANCE / (M_PI * WHEELSIZE)) + offset // Revolutions per Tile (S / (PI)*Diameter = Revolutions )
#define TIMEOUT_TIME 2500 // Time in milliseconds to wait for a command to complete
#define MINVOLTAGE 0.25
#define MAXVOLTAGE 6
#define ROTATE90 600
#define ROTATE45 ROTATE90 / 2.0
#define ROTATE180 ROTATE90 * 2.0

// PID Control
double PIDControl(double target, double position);

// Function prototypes
void rotateTo(double target);
void driveForward(float tiles);

// External declarations
extern vex::motor left_motor_front;
extern vex::motor left_motor_mid1;
extern vex::motor left_motor_mid2;
extern vex::motor left_motor_back;
extern vex::motor right_motor_front;
extern vex::motor right_motor_mid1;
extern vex::motor right_motor_mid2;
extern vex::motor right_motor_back;
extern vex::motor_group left_motor_group;
extern vex::motor_group right_motor_group;
extern vex::brain Brain;


#endif // AUTO_CONTROL_H