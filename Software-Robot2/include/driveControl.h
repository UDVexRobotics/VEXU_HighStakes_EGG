#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include "vex.h"

// External definitions
extern vex::controller primary_controller;
extern vex::motor_group left_motor_group;
extern vex::motor_group right_motor_group;

// Function prototypes
void dual_stick_drive(void);
void tank_drive(void);

#endif // DRIVECONTROL_H