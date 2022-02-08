/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

/* Include the Pololu library */
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
