#include "belt_control.h"

void intake_toggle(void){
    //std::cout<<"Intake Toggle"<<std::endl;
}

void belt_toggle_on(void){
    //std::cout<<"Belt Toggle On"<<std::endl;
    belt_toggle_state = true;

}

void belt_toggle_off(void){
    //std::cout<<"Belt Toggle Off"<<std::endl;
    belt_toggle_state = false;
}

void auto_belt_thread(void){
    belt_motor.setVelocity(-100, vex::percentUnits::pct);
    belt_motor.spin(forward);
    double lastPosition = belt_motor.position(vex::rotationUnits::deg);
    
    while(true){
        this_thread::sleep_for(250);
        double position = belt_motor.position(vex::rotationUnits::deg);
        
        // Prevent Jams
        if(position > (lastPosition - 2) && position < (lastPosition + 2)){
            // Spit out blockage
            belt_motor.setVelocity(100, vex::percentUnits::pct);
            //intake_motor.setVelocity(100, vex::percentUnits::pct);
            //intake_motor.spin(forward);
            belt_motor.spin(forward);

            // Wait for blockage to clear
            this_thread::sleep_for(300);

            // Resume Intake
            //intake_motor.setVelocity(-100, vex::percentUnits::pct);
            //intake_motor.spin(forward);
            belt_motor.setVelocity(-100, vex::percentUnits::pct);
            belt_motor.spin(forward);
        }

        lastPosition = position;
    }
}

void belt_control(void){
    while(true){
        //int belt_position = abs((((int)belt_motor.position(vex::rotationUnits::deg)) % BELT_THROW_POSITION));
        //Brain.Screen.printAt(1, 150, "Belt Position: %6d", belt_position);
        belt_motor.position(vex::rotationUnits::deg);

       if(color_detected){
            wait(0.15, sec); // Wait until at peak
            //std::cout<<"Ejecting Ring!"<<std::endl;
            
            belt_motor.stop(vex::brakeType::brake); // Briefly stop
            wait(0.45, sec);
            //belt_motor.setVelocity(BELTSPEED, vex::percentUnits::pct);
            belt_motor.spin(forward);
            wait(0.7, sec);
       }

        if(BELT_TOGGLE_BUTTON){
            belt_toggle_state = !belt_toggle_state;
            this_thread::sleep_for(250);
        }

        if(belt_toggle_state){
            if(reverse_belt)
                belt_motor.setVelocity(-BELTSPEED, vex::percentUnits::pct);
            else
                belt_motor.setVelocity(BELTSPEED, vex::percentUnits::pct);
            
            belt_motor.spin(forward);  
        }
        else{
            belt_motor.stop(brake);
        }

        this_thread::sleep_for(20);


    }
}