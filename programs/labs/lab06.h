#include "mbed.h"
#include "crazyflie.h"
#include <cmath>
#include "mixer.h"

//Declaring MIXER object mixer
MIXER Mixer;

// Main program
int main()
{
    // Arm motor
    Mixer.arm();

    //Actuate motor with 70% mg total trust force (N) and zero torques   
    Mixer.actuate(0.6*m*g,0,0,0);
    wait(2);

    Mixer.actuate(0.6*m*g,0,0,0.001);
    wait(1);

    Mixer.actuate(0.6*m*g,0,0,-0.001);
    wait(1);

    //turn off all motors
    Mixer.disarm();
}
