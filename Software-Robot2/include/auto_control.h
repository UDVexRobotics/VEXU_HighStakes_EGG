#ifndef AUTO_CONTROL_H
#define AUTO_CONTROL_H

#include "vex.h"

// PID Defintions
#define KP 0.01
#define LR_KP 0.05
#define WHEELSIZE 2.75    // Inches Diameter
#define TILEDISTANCE (2 * 12) // 2 feet
#define MANUAL_OFFSET 1.3
#define TILEREVOLUTIONS(offset) (TILEDISTANCE / (M_PI * WHEELSIZE)) + offset // Revolutions per Tile (S / (PI)*Diameter = Revolutions )
#define TIMEOUT_TIME 2000 // Time in milliseconds to wait for a command to complete
#define MINVOLTAGE 2.5
#define MAXVOLTAGE 8
#define ROTATE90 600

// PID Control
double PIDControl(double target, double position);

// Function prototypes
void rotateTo(double target);
void driveForward(int tiles);

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