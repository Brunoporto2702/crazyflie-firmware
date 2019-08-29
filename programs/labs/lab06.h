#include "mbed.h"
#include "crazyflie.h"
#include <cmath>

//Declaring MIXER object mixer
MIXER mixer;

// Main program
int main()
{
    // Arm motor
    mixer.arm();

    //Actuate motor with 70% mg total trust force (N) and zero torques   
    mixer.actuate(0.6*m*g,0,0,0);
    wait(2);

    mixer.actuate(0.6*m*g,0,0,0.001);
    wait(1);

    mixer.actuate(0.6*m*g,0,0,-0.001);
    wait(1);

    //turn off all motors
    mixer.disarm();
}
