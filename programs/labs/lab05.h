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

//Define angular velocities (rad/s)
float omega_r_1;
float omega_r_2;
float omega_r_3;
float omega_r_4;

//Converts desired angular velocity to PWM signal
float control_motor(float omega_rad)
{
    float PWM = powf(omega_rad,2)*a2+omega_rad*a1;

    return PWM;    
}

// Converts the desired trust force (N) and torques (N.m) to angular velocities (rad/s)
void mixer(float f_t, float tau_phi, float tau_theta, float tau_psi)
{

    float w1_sqrd = 1/(4*kl)*f_t - 1/(4*kl*l)*tau_phi - 1/(4*kl*l)*tau_theta - 1/(4*kd)*tau_psi;
    float w2_sqrd = 1/(4*kl)*f_t - 1/(4*kl*l)*tau_phi + 1/(4*kl*l)*tau_theta + 1/(4*kd)*tau_psi;
    float w3_sqrd = 1/(4*kl)*f_t + 1/(4*kl*l)*tau_phi + 1/(4*kl*l)*tau_theta - 1/(4*kd)*tau_psi;
    float w4_sqrd = 1/(4*kl)*f_t + 1/(4*kl*l)*tau_phi - 1/(4*kl*l)*tau_theta + 1/(4*kd)*tau_psi;

    if (w1_sqrd < 0)
    {
        omega_r_1 = 0;
    }
    else 
    {
        omega_r_1 = sqrt(w1_sqrd);
    }

    if (w2_sqrd < 0)
    {
        omega_r_2 = 0;
    }
    else 
    {
        omega_r_2 = sqrt(w2_sqrd);
    }

    if (w3_sqrd < 0)
    {
        omega_r_3 = 0;
    }
    else 
    {
        omega_r_3 = sqrt(w3_sqrd);
    }

    if (w4_sqrd < 0)
    {
        omega_r_4 = 0;
    }
    else 
    {
        omega_r_4 = sqrt(w4_sqrd);
    }
    return;
}

void actuate(float f_t, float tau_phi, float tau_theta, float tau_psi)
{
    mixer(f_t, tau_phi, tau_theta, tau_psi);
    motor1 = control_motor(omega_r_1);
    motor2 = control_motor(omega_r_2);
    motor3 = control_motor(omega_r_3);
    motor4 = control_motor(omega_r_4);
}

//desired velocity
float vel_rad1_3;
float vel_rad2_4;

// Main program
int main()
{
    // Blink blue LED indicating inicialization (5 seconds)
    led_blue = 1;
    wait(2);
    led_blue = false;
    // Turn on red LEDs indicating motors are armed
    led_red = !true;

    //Actuate motor with 70% mg total trust force (N) and zero torques   
    actuate(0.6*m*g,0,0,0);
    wait(2);

    actuate(0.6*m*g,0,0,0.001);
    wait(1);

    actuate(0.6*m*g,0,0,-0.001);
    wait(1);

    //turn off all motors
    actuate(0,0,0,0);

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
