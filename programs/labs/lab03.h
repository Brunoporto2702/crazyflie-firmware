#include "mbed.h"
#include "crazyflie.h"
#include <cmath>

// Define all LEDs as digital output objects
DigitalOut led_blue(LED_BLUE_L,!false);
DigitalOut led_red(LED_RED_L,!false);
DigitalOut led_green(LED_GREEN_L,!false);

// Define all motors as PWM objects
PwmOut motor1(MOTOR1);
PwmOut motor2(MOTOR2);
PwmOut motor3(MOTOR3);
PwmOut motor4(MOTOR4);


//Converts desired angular velocity to PWM signal
float control_motor(/*float omega_rpm*/float omega_rad)
{
    //float omega_rad;
    //omega_rad = omega_rpm*2*pi*1/60;
    float PWM = powf(omega_rad,2)*a2+omega_rad*a1;

    return PWM;    
}

//desired velocity
float vel_rad;

// Main program
int main()
{
    // Blink blue LED indicating inicialization (5 seconds)
    led_blue = 1;
    wait(2);
    led_blue = false;
    // Turn on red LEDs indicating motors are armed
    led_red = !true;

    //Trun motors ON for 10s for data aquisition and turn OFF    

    vel_rad = 2200.0;
    motor1 = control_motor(vel_rad);
    motor2 = control_motor(vel_rad);
    motor3 = control_motor(vel_rad);
    motor4 = control_motor(vel_rad);
    wait(10);
    motor1 = 0.0;
    motor2 = 0.0;
    motor3 = 0.0;
    motor4 = 0.0;

    // Turn off red LEDs indicating motors are disarmed
    
    led_red = true;
    // Blink green LEDs indicating end of program
    while(true)
    {
        led_green = true;
        wait(0.5);
        led_green = !led_green;
    }
}
