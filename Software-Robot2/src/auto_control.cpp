#include "auto_control.h"
#include <iostream>



// PID Control
double PIDControl(double target, double position){
    return (target - position) * KP;
}

void rotateTo(double target, float max_volts) {
    target = target * -1; // Reverse the target
    bool is_negative = (target < 0);  // Check if the target is negative (counter-clockwise vs clockwise)
    target = fabs(target);
    double avg_pos = 0;
    
    left_motor_group.setPosition(0, vex::degrees);
    right_motor_group.setPosition(0, vex::degrees);

    uint32_t start_time = Brain.Timer.time();
    double error_Integral = 0;
    double lastError = 0;

    // PID Loop
    while (!(target+1 > avg_pos && avg_pos > target-1)) {
        vex::this_thread::sleep_for(25); // Sleep for 20 milliseconds

        double left_pos = fabs(left_motor_group.position(vex::degrees));
        double right_pos = fabs(right_motor_group.position(vex::degrees));
        avg_pos = (left_pos + right_pos) / 2.0;
        double drive = PIDControl(target, avg_pos);
        error_Integral += (target - avg_pos) * KI;
        double error_Derivative = ((target - avg_pos) - lastError) * KD;
        if (error_Integral > 1)
            error_Integral = 1;
        else if (error_Integral < -1)
            error_Integral = -1;

        drive = drive + error_Integral + error_Derivative;
        
        //std::cout<<"Avg: "<<avg_pos<<" target: "<<target<<" drive: "<<drive<<std::endl;

        //Clamp voltage to min and max 
        // drive = (drive < MINVOLTAGE && drive > 0) ? MINVOLTAGE : drive; // Don't allow drive to go below minimum voltage (+speed)
        // drive = (drive > -MINVOLTAGE && drive < 0) ? -MINVOLTAGE : drive; // Don't allow drive to go below minimum voltage (-speed)

        // drive = (drive > MAXVOLTAGE) ? MAXVOLTAGE : drive; // Don't allow drive to go above maximum voltage (+speed)
        // drive = (drive < -MAXVOLTAGE) ? -MAXVOLTAGE : drive; // Don't allow drive to go below maximum voltage (-speed)

        double left_drive = drive;
        double right_drive = drive;

        

        // // Correct the drive values between the left and right motors (Fix Off-balanced drive)
        // if ((right_pos - left_pos) > 10) { // If the right motor is ahead of the left motor by X degrees
        //     left_drive += (right_pos - left_pos) * LR_KP;
        // } else if ((left_pos - right_pos) > 10){
        //     right_drive += (left_pos - right_pos) * LR_KP;
        // }

        // Set the motor voltages
        // Don't allow drive to go below minimum voltage (speed)

        left_drive = (left_drive < MINVOLTAGE && left_drive > 0) ? MINVOLTAGE : left_drive; // Don't allow left_drive to go below minimum voltage (+speed)
        left_drive = (left_drive > -MINVOLTAGE && left_drive < 0) ? -MINVOLTAGE : left_drive; // Don't allow left_drive to go below minimum voltage (-speed)

        left_drive = (left_drive > MAXVOLTAGE) ? MAXVOLTAGE : left_drive; // Don't allow left_drive to go above maximum voltage (+speed)
        left_drive = (left_drive < -MAXVOLTAGE) ? -MAXVOLTAGE : left_drive; // Don't allow left_drive to go below maximum voltage (-speed)

        right_drive = (right_drive < MINVOLTAGE && right_drive > 0) ? MINVOLTAGE : right_drive; // Don't allow right_drive to go below minimum voltage (+speed)
        right_drive = (right_drive > -MINVOLTAGE && right_drive < 0) ? -MINVOLTAGE : right_drive; // Don't allow right_drive to go below minimum voltage (-speed)

        right_drive = (right_drive > MAXVOLTAGE) ? MAXVOLTAGE : right_drive; // Don't allow right_drive to go above maximum voltage (+speed)
        right_drive = (right_drive < -MAXVOLTAGE) ? -MAXVOLTAGE : right_drive; // Don't allow right_drive to go below maximum voltage (-speed)

        
        
        // If the target is negative, turn counter-clockwise
        if (is_negative) { 
            left_motor_group.spin(vex::forward, -left_drive, vex::voltageUnits::volt); // Reversed
            right_motor_group.spin(vex::forward, right_drive, vex::voltageUnits::volt);
        }else {
            //std::cout<<"Left: "<<left_drive<<" Right: "<<right_drive<<std::endl;
            left_motor_group.spin(vex::forward, left_drive, vex::voltageUnits::volt);
            right_motor_group.spin(vex::forward, -right_drive, vex::voltageUnits::volt); // Reversed
        }
        uint32_t elapsed_time = Brain.Timer.time() - start_time;
        if (elapsed_time > TIMEOUT_TIME) {
            //std::cout<<"Timeout"<<std::endl;
            break;
        }
        lastError = (target - avg_pos) * KD;
    }
    //std::cout<<"Done"<<std::endl;
    left_motor_group.stop(vex::brakeType::brake);
    right_motor_group.stop(vex::brakeType::brake);
}

void driveForward(float tiles, float max_volts){
    int t = (int)(tiles * (TILEREVOLUTIONS(MANUAL_OFFSET)) * 360.0); // Convert tiles to degrees
    //std::cout<<((TILEREVOLUTIONS(MANUAL_OFFSET)) * 360.0)<<std::endl;
    //std::cout<<TILEREVOLUTIONS(MANUAL_OFFSET)<<std::endl;
    //std::cout<<"Target: "<<t<<std::endl;
    float avg_position = 0;

    // Reset the motor positions
    left_motor_group.setPosition(0, vex::degrees);
    right_motor_group.setPosition(0, vex::degrees);

    uint32_t start_time = Brain.Timer.time();

    double lastError = 0;
    double error_Integral = 0;
    
    while(!(t+2 > avg_position && avg_position > t-2)){
        vex::this_thread::sleep_for(25);
        double left_position = left_motor_group.position(vex::degrees);
        double right_position = right_motor_group.position(vex::degrees);
        avg_position = (left_position + right_position) / 2;
        double drive = PIDControl(t, avg_position); // Calculate the drive value
        error_Integral += (t - avg_position) * KI;
        double error_Derivative = ((t - avg_position) - lastError) * KD;

        if (error_Integral > 1)
            error_Integral = 1;
        else if (error_Integral < -1)
            error_Integral = -1;

        drive = drive + error_Integral + error_Derivative;


        //std::cout<<"drive: "<<drive<<std::endl;
        // P-Control between left and right motors
        double left_voltage_drive = drive;
        double right_voltage_drive = drive;

        // Don't allow left_voltage_drive to go below minimum voltage (speed)
        left_voltage_drive = (left_voltage_drive < MINVOLTAGE && left_voltage_drive > 0) ? MINVOLTAGE : left_voltage_drive;
        left_voltage_drive = (left_voltage_drive > -MINVOLTAGE && left_voltage_drive < 0) ? -MINVOLTAGE : left_voltage_drive;

        // Don't allow left_voltage_drive to go above maximum voltage (speed)
        left_voltage_drive = (left_voltage_drive > max_volts) ? max_volts : left_voltage_drive;
        left_voltage_drive = (left_voltage_drive < -max_volts) ? -max_volts : left_voltage_drive;

        // Don't allow right_voltage_drive to go below minimum voltage (speed)
        right_voltage_drive = (right_voltage_drive < MINVOLTAGE && right_voltage_drive > 0) ? MINVOLTAGE : right_voltage_drive;
        right_voltage_drive = (right_voltage_drive > -MINVOLTAGE && right_voltage_drive < 0) ? -MINVOLTAGE : right_voltage_drive;

        // Don't allow right_voltage_drive to go above maximum voltage (speed)
        right_voltage_drive = (right_voltage_drive > max_volts) ? max_volts : right_voltage_drive;
        right_voltage_drive = (right_voltage_drive < -max_volts) ? -max_volts : right_voltage_drive;
        
        // Correct the drive values between the left and right motors (Fix Off-balanced drive)
        if ((right_position - left_position) > 10) { // If the right motor is ahead of the left motor by X degrees
            left_voltage_drive += (right_position - left_position) * LR_KP;
        } else if ((left_position - right_position) > 10){
            right_voltage_drive += (left_position - right_position) * LR_KP;
        }

        left_motor_group.spin(vex::forward, left_voltage_drive, vex::voltageUnits::volt);
        right_motor_group.spin(vex::forward, right_voltage_drive, vex::voltageUnits::volt);
        
        //std::cout<<"Left: "<<left_voltage_drive<<" Right: "<<right_voltage_drive<<" drive: "<<drive<<std::endl;


        uint32_t elapsed_time = Brain.Timer.time() - start_time;
        if(elapsed_time > TIMEOUT_TIME){
            //std::cout<<"Timeout"<<std::endl;
            break; // Break if the command takes too long
        }

        lastError = (t - avg_position) * KD;
    }

    //std::cout<<"Done"<<std::endl;
    //std::cout<<t<<" "<<avg_position<<std::endl;
    //std::cout<<!(t+2 > avg_position && avg_position > t-2)<<std::endl;
    // Stop the motors
    left_motor_group.stop(vex::brakeType::brake);
    right_motor_group.stop(vex::brakeType::brake);

    return;
}
