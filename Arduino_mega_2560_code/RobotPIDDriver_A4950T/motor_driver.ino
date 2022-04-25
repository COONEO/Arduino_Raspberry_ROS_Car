#include "Make4e2ndChassis.h"

/* Create the motor driver object */
Make4e2ndChassis drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
 // Serial.println("init Make4e2ndChassis");
}


// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {

  drive.setSpeeds(leftSpeed,rightSpeed);
  
}
