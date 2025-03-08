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
    // Print Out Motor Temperatures
    std::cout<<"Left Motor Group Temp: "<<left_motor_group.temperature()<<std::endl;
    std::cout<<"Right Motor Group Temp: "<<right_motor_group.temperature()<<std::endl;
    std::cout<<"Intake Motor Temp: "<<intake_motor.temperature()<<std::endl;
    std::cout<<"Belt Motor Temp: "<<belt_motor.temperature()<<std::endl;
    std::cout<<"High Stakes Motor Temp: "<<highstake_motor.temperature()<<std::endl;
    
    uint32_t start_time = Brain.Timer.time();
    // Freeing intake and expanding
    highstake_motor.setVelocity(100, vex::percentUnits::pct);
    highstake_motor.spin(forward);
    belt_motor.setVelocity(15, vex::percentUnits::pct);
    belt_motor.spin(forward);   
    this_thread::sleep_for(700);
    //belt_motor.stop(coast);
    highstake_motor.stop(hold);



    // Going for Mobile Goal
    driveForward(-1.25,MAXVOLTAGE-2);
    rotateTo(-ROTATE45,MAXVOLTAGE-3);
    //std::thread(delayed_actuator_toggle,(1000));
    uint32_t delay = 500;
    task delayTask1(delayed_actuator_toggle, (void *)&delay); // Grab Mobile Goal
    driveForward(-0.45,MAXVOLTAGE-2);
    rotateTo((ROTATE45/10) + ROTATE45,MAXVOLTAGE-3);
    
    // Turn on Intake
    intake_motor.setVelocity(-100, vex::percentUnits::pct);
    intake_motor.spin(forward);

    vex::thread auto_belt = thread(auto_belt_thread);
    

    // Grab Ring One
    driveForward(1,MAXVOLTAGE-2); // Grab Ring One
    this_thread::sleep_for(250);
    driveForward(-0.2,MAXVOLTAGE-2); // Back up
    //rotateTo(-ROTATE90 - (ROTATE45/1.5)); // Turn Around
    rotateTo(-ROTATE90 - (ROTATE45));
    //driveForward(1); // Grab Ring 2 & approach group of rings in mid
    driveForward(2.5); // Grab Ring 2 & approach group of rings in mid
    this_thread::sleep_for(200);

    //rotateTo(-ROTATE45/2,MAXVOLTAGE-2); // closer towards rings
    //driveForward(1.5,MAXVOLTAGE-2); // Grab Ring 3 (at big group)
    this_thread::sleep_for(100);
    driveForward(-.75,MAXVOLTAGE-2); // Back up
    rotateTo(ROTATE45/2,MAXVOLTAGE-2); // Turn Around

    driveForward(0.85,MAXVOLTAGE-2); // Grab Ring 4
    this_thread::sleep_for(250);
    driveForward(-0.85,MAXVOLTAGE-2); // Back up
    rotateTo(-ROTATE45,MAXVOLTAGE-2); // Turn Around

    driveForward(0.85,MAXVOLTAGE-2); // Grab Ring 5
    this_thread::sleep_for(250);
    driveForward(-0.85,MAXVOLTAGE-2); // Back up
    rotateTo(ROTATE45/2,MAXVOLTAGE-2); // Readjust

    driveForward(-2.5,MAXVOLTAGE-2); // Back up
    //rotateTo(-ROTATE45/2,MAXVOLTAGE-2); // Readjust

    // Turn towards ring 6 (corner)
    rotateTo(-ROTATE180,MAXVOLTAGE-2);
    driveForward(2); // Go to ring 6
    this_thread::sleep_for(250);
    driveForward(-0.75); // Back up
    rotateTo(ROTATE180,MAXVOLTAGE-2); // Turn Around
    delay = 250;
    task delayTask2(delayed_actuator_toggle, (void *)&delay);  // Drop Mobile Goal
    driveForward(-1); // Go to deposit spot
    this_thread::sleep_for(250);
    driveForward(0.5); // drive away from mobile goal

    intake_motor.stop(coast);

    // Realign Against wall
    rotateTo(-ROTATE45,MAXVOLTAGE-2);
    driveForward(-0.75); // Go to ring 7

    driveForward(3); // Go to Mobile Goal
    rotateTo(ROTATE180 + ROTATE45); // Turn towards goal
    intake_motor.spin(forward);
    delay = 700;
    task delayTask3(delayed_actuator_toggle, (void *)&delay); // Grab Mobile Goal
    driveForward(-1.5); // Grab goal

    // Turn towards rings
    rotateTo(ROTATE90);
    driveForward(1); // Go to rings
    rotateTo(-ROTATE90); // Turn towards rings
    driveForward(1); // Go to rings

    // Turn towards corner (top right)
    rotateTo(ROTATE45);
    driveForward(1); // Go to ring 1
    this_thread::sleep_for(250);
    driveForward(-0.75); // Back up
    rotateTo(ROTATE180); // Turn Around
    task delayTask4(delayed_actuator_toggle, (void *)&delay); // Drop Mobile Goal
    driveForward(-1); // Go to ring 2


    // rotateTo(ROTATE180); // Turn climb
    // highstake_motor.spinToPosition(-270, degrees); // Climb
    // driveForward(-2);
    
    std::cout<<"Time: "<<Brain.Timer.time()-start_time<<std::endl;



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
    highstake_motor.setMaxTorque(100, percentUnits::pct);
    belt_motor.setMaxTorque(100, percentUnits::pct);
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
        // if(HIGHSTAKES_FORWARD_MOTOR_BUTTON){
        //     highstake_motor.setVelocity(100, vex::percentUnits::pct);
        //     highstake_motor.spin(forward);
        // }
        // else if(HIGHSTAKES_BACKWARD_MOTOR_BUTTON){
        //     highstake_motor.setVelocity(-100, vex::percentUnits::pct);
        //     highstake_motor.spin(forward);
        // }
        // else if(HIGHSTAKES_POSTION_BUTTON){
        //     highstake_motor.spinToPosition(-49.6, degrees);
        // }
        // else{
        //     highstake_motor.stop(hold);
        // }

        if(fabs(HIGHSTAKES_JOYSTICK) > 0){
            highstake_motor.setVelocity(-pow(HIGHSTAKES_JOYSTICK/100.0,3)*100, vex::percentUnits::pct);
            highstake_motor.spin(forward);
        }
        else if(HIGHSTAKES_POSTION_BUTTON){
            //highstake_motor.spinToPosition(HIGHSTAKES_POSITION, degrees);
            highstake_motor.setVelocity(100, vex::percentUnits::pct);
            highstake_motor.spinToPosition(HIGHSTAKES_POSITION, degrees,false);
        }   
        else{
            highstake_motor.stop(hold);
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

        if(fabs(BELT_CONTROL) > 0){
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
        //std::cout<<"High Stakes"<<highstake_motor.position(vex::degrees)<<std::endl;
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