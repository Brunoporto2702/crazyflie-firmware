#include "crazyflie.h"
#include "mbed.h"
#include <cmath>

#ifndef mixer_h
#define mixer_h

class MIXER {

public:
  MIXER();
  void arm();
  void disarm();
  void actuate(float f_t, float tau_phi, float tau_theta, float tau_psi);

private:
  bool armed;

  DigitalOut led_red;
  DigitalOut led_green;

  PwmOut motor1;
  PwmOut motor2;
  PwmOut motor3;
  PwmOut motor4;

  float omega_r_1;
  float omega_r_2;
  float omega_r_3;
  float omega_r_4;

  float control_motor(float omega_rad);

  void mixer(float f_t, float tau_phi, float tau_theta, float tau_psi);
};

#endif