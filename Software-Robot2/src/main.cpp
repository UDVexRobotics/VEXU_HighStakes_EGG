/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       University of Delaware Team 2                             */
/*    Created:      X/XX/2024                                                 */
/*    Description:  Software for Robot 2 of UD's VEXU Varsity Team            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "vex.h"

using namespace vex;

// Global Constants
// Make sure to define a motor with the right gear ratio (motor gear color)
const gearSetting RED_GEAR = ratio36_1;     // 100 RPM - high torque & low speed (e.g. lifting arms & moving claws,)
const gearSetting GREEN_GEAR = ratio18_1;   // 200 RPM - standard gear ratio for drivetrain applications 
const gearSetting BLUE_GEAR = ratio6_1;     // 600 RPM - low torque & high speed (e.g.  intake rollers & flywheels))

// A global instance of vex::brain
brain Brain;

// Global instance of vex::competition
competition compete;

// Global instance of vex::controller
controller primary_controller = controller(primary);

// define your global instances of vex::motors and other devices here
    /* Example Code
        motor left_motor_front = motor(PORT1, GREEN_GEAR, false); 
        motor left_motor_mid = motor(PORT2, GREEN_GEAR, false);
        motor left_motor_back = motor(PORT5, GREEN_GEAR, false);
        motor_group left_motor_group = motor_group(left_motor_front, left_motor_mid, left_motor_back);
    */

// Code block for Pre-Autonomous 
void pre_auton(void) {
    Brain.Screen.print("Pre-Autonomous start!");
    Brain.Screen.newLine();


    // All activities that occur before the competition starts
    Brain.Screen.print("Pre-Autonomous complete.");
    Brain.Screen.newLine();
}

// Code block for Autonomous
void autonomous(void) {
    Brain.Screen.print("Autonomous start!");
    Brain.Screen.newLine();

    // Autonomous loop
    while(true){
        // Indefinite Autonomous code here

    }
}

// Code block for User Control
void usercontrol(void) {
    Brain.Screen.print("User Control start!");
    Brain.Screen.newLine();
    
    // User Control loop
    while(true){
        // Indefinite User Control code here

    }
}

int main() {
    Brain.Screen.print("Main start!");
    Brain.Screen.newLine();

    // Set up callbacks for autonomous and driver control periods.
    compete.autonomous(autonomous);
    compete.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true) {
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
    
    Brain.Screen.print("Main complete.");
    Brain.Screen.newLine();
}