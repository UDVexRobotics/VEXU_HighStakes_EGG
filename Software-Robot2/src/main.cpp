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
bool actuatorToggle = false;
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

// PID Control
double PIDControl(double target, double position){
    return (target - position) * KP;
}

void rotateTo(double target) {
    vex::timer timer = vex::timer();

    bool is_negative = (target < 0);
    double magnitude = fabs(target);
    double avg_pos = 0;
    
    left_motor_group.setPosition(0, vex::rotationUnits::deg);
    right_motor_group.setPosition(0, vex::rotationUnits::deg);
    timer.reset();
    double start_time = timer.value();

    // PID Loop
    while (!(avg_pos <= target && avg_pos >= target - 1)) {
        double left_pos = abs(left_motor_group.position(vex::rotationUnits::deg));
        double right_pos = abs(right_motor_group.position(vex::rotationUnits::deg));
        avg_pos = (left_pos + right_pos) / 2;
        double drive = PIDControl(target, avg_pos);
        
        // Clamp voltage to min and max 
        drive = (drive < MINVOLTAGE && drive > 0) ? MINVOLTAGE : drive;
        drive = (drive > -MINVOLTAGE && drive < 0) ? -MINVOLTAGE : drive;

        if (drive < MAXVOLTAGE) {
            drive = MAXVOLTAGE;
        } else if (drive < -MAXVOLTAGE) {
            drive = -MAXVOLTAGE;
        }
        
        double left_drive = drive;
        double right_drive = drive;
        if (left_pos > right_pos) {
            left_drive += (right_pos - left_pos) * LR_KP;
        } else {
            right_drive += (left_pos - right_pos) * LR_KP;
        }

        if (is_negative) { 
            left_motor_group.spin(reverse, left_drive, vex::voltageUnits::volt);
            right_motor_group.spin(forward, right_drive, vex::voltageUnits::volt);
        } else {
            left_motor_group.spin(forward, left_drive, vex::voltageUnits::volt);
            right_motor_group.spin(reverse, right_drive, vex::voltageUnits::volt);
        } 
        if ((timer.value() - start_time) > TIMEOUT_TIME) {
            break;
        }
    }

    left_motor_group.stop();
    right_motor_group.stop();
}

void driveForward(int tiles){
    int t = (int)(tiles * (TILEREVOLUTIONS(MANUAL_OFFSET)) * 360.0);
    std::cout<<((TILEREVOLUTIONS(MANUAL_OFFSET)) * 360.0)<<std::endl;
    std::cout<<TILEREVOLUTIONS(MANUAL_OFFSET)<<std::endl;
    std::cout<<"Target: "<<t<<std::endl;
    float avg_position = 0;

    left_motor_group.setPosition(0, degrees);
    right_motor_group.setPosition(0, degrees);

    uint32_t start_time = Brain.Timer.time();
    
    while(!(t+1 > avg_position && avg_position > t-1)){
        double left_position = left_motor_group.position(degrees);
        double right_position = right_motor_group.position(degrees);
        avg_position = (left_position + right_position) / 2;
        double drive = PIDControl(t, avg_position); // Calculate the drive value

    // Don't allow drive to go below minimum voltage (speed)
    drive = (drive < MINVOLTAGE && drive > 0) ? MINVOLTAGE : drive;
    drive = (drive > -MINVOLTAGE && drive < 0) ? -MINVOLTAGE : drive;

    if (drive > MAXVOLTAGE) {
        drive = MAXVOLTAGE;
    } else if (drive < -MAXVOLTAGE) {
        drive = -MAXVOLTAGE;
    }

    // P-Control between left and right motors
    double left_voltage_drive = drive;
    double right_voltage_drive = drive;

    // Set the motor voltages
    left_motor_group.spin(forward, left_voltage_drive, voltageUnits::volt);
    right_motor_group.spin(forward, right_voltage_drive, voltageUnits::volt);
    
    uint32_t elapsed_time = Brain.Timer.time() - start_time;
    if(elapsed_time > TIMEOUT_TIME)
        break; // Break if the command takes too long

    }

    // Stop the motors
    left_motor_group.stop(brakeType::brake);
    right_motor_group.stop(brakeType::brake);

    return;
}



// Code block for Pre-Autonomous 
void pre_auton(void) {
    //Brain.Screen.print("Pre-Autonomous start!");
    //Brain.Screen.newLine();


    // All activities that occur before the competition starts
    //Brain.Screen.print("Pre-Autonomous complete.");
    //Brain.Screen.newLine();
}

// Code block for Autonomous
void autonomous(void) {
    //Brain.Screen.print("Autonomous start!");
    //Brain.Screen.newLine();
    //PathFollowing::driveForward(10, localizer, odometry_constants, 
    //left_motor_group, right_motor_group);
    
    driveForward(1);
    wait(2, sec);
    driveForward(-1);
    wait(2, sec);
    driveForward(2);
    wait(2, sec);
    rotateTo(90);
    wait(2, sec);
    driveForward(2);
}

// Code block for User Control

void dual_stick_drive(void){

    // Controls for Up-Down and Left-Right movement
    float leftStick = (float)(primary_controller.Axis3.position() / 100.0);             // Vertical Movement
    float rightStick = primary_controller.Axis1.position() / (float)-100.0;            // Horizontal Movement
   
    
    // Motor speed percentage based on cubed function
    float ySpeed = pow(leftStick, 3);
    float xSpeed = pow(rightStick /*TURN_SPEED_RATIO*/, 3);

    //if(controller.buttonL2.pressing()):
        //xSpeed = (rightAxis_LR * SHIFT_TURN_SPEED_RATIO) ** 3

    
    if(fabs(ySpeed) > 0.05 or fabs(xSpeed) > 0.05){
        //printf("Left Stick: %f, Right Stick: %f\n", ySpeed, xSpeed);

        //Set the velocity depending on the axis position
        left_motor_group.setVelocity((ySpeed + xSpeed) * 100,vex::percentUnits::pct);
        right_motor_group.setVelocity((ySpeed - xSpeed) * 100,vex::percentUnits::pct); 

        left_motor_group.spin(forward);
        right_motor_group.spin(forward);

    }
    else {
        left_motor_group.stop(coast);
        right_motor_group.stop(coast);
    }

    return;
}

void tank_drive(void){
    // Controls for Up-Down and Left-Right movement
    float rightStick = (primary_controller.Axis3.position() / (float)100.0);             // Vertical Movement
    float leftStick = (primary_controller.Axis2.position() / (float)100.0);            // Horizontal Movement
   
    // Motor speed percentage based on cubed function
    float leftSpeed = pow(leftStick, 3);
    float rightSpeed = pow(rightStick, 3);

    if(fabs(leftSpeed) > 0.05){
        //Set the velocity depending on the axis position
        left_motor_group.setVelocity(leftSpeed * 100,vex::percentUnits::pct);
        left_motor_group.spin(forward);
    }
    else{
        left_motor_group.stop(coast);
    }

    if(fabs(rightSpeed) > 0.05){
        right_motor_group.setVelocity(rightSpeed * 100,vex::percentUnits::pct); 
        right_motor_group.spin(forward);
    }
    else{
        right_motor_group.stop(coast);

    }  

}


void usercontrol(void) { 
    // Usercontrol loop
    while(true){
        //bool buttonR1 = primary_controller.ButtonR1.pressing();
        //bool buttonR2 = primary_controller.ButtonR2.pressing();
        //bool buttonL1 = primary_controller.ButtonL1.pressing();
        //bool buttonL2 = primary_controller.ButtonL2.pressing();
        //reverse_belt = REVERSE_BELT_BUTTON;
       
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

        if(abs(BELT_B) > 0){
            belt_motor.setVelocity(BELT_B, vex::percentUnits::pct);
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

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true) {
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}