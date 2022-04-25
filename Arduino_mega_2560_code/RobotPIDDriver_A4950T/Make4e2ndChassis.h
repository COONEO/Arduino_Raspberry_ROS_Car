#ifndef Make4e2ndChassis_h
#define Make4e2ndChassis_h

#include <Arduino.h>

class Make4e2ndChassis
{
  public:  
    // CONSTRUCTORS
    Make4e2ndChassis(); // Default pin selection.
    
    // PUBLIC METHODS
    void init(); 
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
  private:

    unsigned char Back_Left_D1;
    unsigned char Back_Left_D1_B;

    unsigned char Back_Right_D1;
    unsigned char Back_Right_D1_B;
};

#endif
