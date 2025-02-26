/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       University of Delaware Team 2                             */
/*    Created:      2/13/2025                                                 */
/*    Description:  Software for Robot EGG of UD's VEXU Team                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "main.h"

// Actuator Control
volatile bool actuatorToggle = false;
void actuator_thread(void){
    while(true){
        if(ACTUATOR_TOGGLE_BUTTON){
            (actuatorToggle) ? Actuator.set(false) : Actuator.set(true);
            actuatorToggle = !actuatorToggle;
            this_thread::sleep_for(250);
        }
        this_thread::sleep_for(20);
    }
}

int delayed_actuator_toggle(void *params){
    uint32_t* delay = static_cast<uint32_t*>(params);
    this_thread::sleep_for(*delay);
    (actuatorToggle) ? Actuator.set(false) : Actuator.set(true);
    actuatorToggle = !actuatorToggle;
    return 0;
}


// Code block for Pre-Autonomous 
void pre_auton(void) { 
    std::cout<<"Pre-Autonomous start!"<<std::endl;
    
}

void skills_auton(void){
    highstake_motor.setVelocity(100, vex::percentUnits::pct);
    highstake_motor.spin(forward);
    belt_motor.setVelocity(15, vex::percentUnits::pct);
    belt_motor.spin(forward);   
    this_thread::sleep_for(700);
    //belt_motor.stop(coast);
    highstake_motor.stop(brake);

    driveForward(-1.25);
    rotateTo(-ROTATE45);
    //std::thread(delayed_actuator_toggle,(1000));
    uint32_t delay = 500;
    task delayTask(delayed_actuator_toggle, (void *)&delay); // Grab Mobile Goal
    driveForward(-1.0);
    rotateTo((ROTATE45/10) + ROTATE45);
    
    // Turn on Intake
    intake_motor.setVelocity(-100, vex::percentUnits::pct);
    intake_motor.spin(forward);

    vex::thread auto_belt = thread(auto_belt_thread);
    

    // Grab Ring One
    driveForward(0.65); // Grab Ring One
    rotateTo(-ROTATE90 - ROTATE45); // Turn Around
    driveForward(1); // Grab Ring 2 & approach next rings

    rotateTo(-ROTATE45/2); // closer towards rings
    driveForward(1); // Grab Ring 3
    driveForward(-0.5); // Back up
    rotateTo(ROTATE45/2); // Turn Around

    driveForward(1); // Grab Ring 4
    this_thread::sleep_for(500);
    driveForward(-0.5); // Back up
    rotateTo(-ROTATE90); // Turn Around

    driveForward(1); // Grab Ring 5
    this_thread::sleep_for(500);
    driveForward(-2.5); // Back up
    rotateTo(ROTATE45); // Turn Around
    rotateTo(ROTATE180);
    
    driveForward(2); // Grab Ring 6
    



}

// Code block for Autonomous
void autonomous(void) {
    //Brain.Screen.print("Autonomous start!");
    //Brain.Screen.newLine();
    //PathFollowing::driveForward(10, localizer, odometry_constants, 
    //left_motor_group, right_motor_group);
    skills_auton();
}

// Code block for User Control
void usercontrol(void) { 
    // Usercontrol loop
    while(true){
        if(INTAKE_FORWARD_BUTTON && !INTAKE_REVERSE_BUTTON){
            intake_motor.setVelocity(-100, vex::percentUnits::pct);
            intake_motor.spin(forward);
        }
        else if(INTAKE_REVERSE_BUTTON && !INTAKE_FORWARD_BUTTON){
            intake_motor.setVelocity(100, vex::percentUnits::pct);
            intake_motor.spin(forward); 
        }
        else{
            intake_motor.stop(brake);
        }

        //TODO: MAY NEED TO BE REVERSED IDK MANE
        if(HIGHSTAKES_FORWARD_MOTOR_BUTTON){
            highstake_motor.setVelocity(100, vex::percentUnits::pct);
            highstake_motor.spin(forward);
        }
        else if(HIGHSTAKES_BACKWARD_MOTOR_BUTTON){
            highstake_motor.setVelocity(-100, vex::percentUnits::pct);
            highstake_motor.spin(forward);
        }
        else{
            highstake_motor.stop(brake);
        }

        /*
        if(BELT_A){
            belt_motor.setVelocity(100, vex::percentUnits::pct);
            belt_motor.spin(forward);
        }
        else if(REVERSE_BELT_BUTTON)
        {
            belt_motor.setVelocity(-100, vex::percentUnits::pct);
            belt_motor.spin(forward);
        }
        else{
            belt_motor.stop(brake);
        }*/

        if(abs(BELT_CONTROL) > 0){
            belt_motor.setVelocity(BELT_CONTROL, vex::percentUnits::pct);
            belt_motor.spin(forward);
        }
        else{
            belt_motor.stop(brake);
        }


        if(SWITCH_DRIVE_TANK)
            currentDriveMode = TANK;
        else if(SWITCH_DRIVE_DUAL)
            currentDriveMode = DUAL_STICK;

        switch (currentDriveMode){
            case TANK:
                tank_drive();
                break;
            case DUAL_STICK:
                dual_stick_drive();
                break;
        }

        this_thread::sleep_for(10);
    }
}

void coutLog(){

    while(true){
        //std::cout<<"Left: "<<left_motor_group.position(vex::degrees)<<" Right: "<<right_motor_group.position(vex::degrees)<<std::endl;
        this_thread::sleep_for(250);
    }
}

int main() {
    // Set up callbacks for autonomous and driver control periods.
    compete.autonomous(autonomous);
    compete.drivercontrol(usercontrol);

    primary_controller.ButtonR1.pressed(intake_toggle);
    //primary_controller.ButtonL1.pressed(belt_toggle_on);
    //primary_controller.ButtonL2.pressed(belt_toggle_off);

    thread display_Status = thread(displayStatus);
    thread actuatorThread = thread(actuator_thread);
    //thread beltThread = thread(belt_control);
    thread visionThread = thread(vision_sensor_thread);
    thread logThread = thread(coutLog);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true) {
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}