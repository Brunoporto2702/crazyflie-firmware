#include "mbed.h"
#include "crazyflie.h"

// Define all LEDs as digital output objects
DigitalOut led_blue(LED_BLUE_L,!false);
DigitalOut led_red(LED_RED_L,!false);
// Define all motors as PWM objects
PwmOut motor1(MOTOR1);
PwmOut motor2(MOTOR2);
PwmOut motor3(MOTOR3);
PwmOut motor4(MOTOR4);



// Main program
int main()
{
    // Blink blue LED indicating inicialization (5 seconds)
    led_blue = 1;
    wait(5);
    led_blue = false;
    // Turn on red LEDs indicating motors are armed
    led_red = !true;
    
    // Test all motors with different frequencies (to make different noises)
    motor1.period(1.0/500.0);
    motor1 = 0.2;
    wait(0.5);
    motor1 = 0.0;
    motor2.period(1.0/1000.0);
    motor2 = 0.2;
    wait(0.5);
    motor2 = 0.0;
    

    // Turn off red LEDs indicating motors are disarmed
    
    // Blink green LEDs indicating end of program
    while(true)
    {
        
    }
}
