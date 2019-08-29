#include "crazyflie.h"
#include "mbed.h"
#include <cmath>

class MIXER {

public:
  MIXER()
      : motor1(MOTOR1), motor2(MOTOR2), motor3(MOTOR3), motor4(MOTOR4),
        led_red(LED_RED_L, !false), led_green(LED_GREEN_L, !false) {}

  void arm() {
    led_green = true;
    led_red = !true;
    wait(1);
    motor1.period(1.0 / 500.0);
    motor1 = 0.1;
    wait(0.1);
    motor1 = 0.0;
    motor1.period(1.0 / 500.0);
    motor2.period(1.0 / 1000.0);
    motor2 = 0.1;
    wait(0.1);
    motor2 = 0.0;
    motor2.period(1.0 / 500.0);
    motor3.period(1.0 / 1500.0);
    motor3 = 0.1;
    wait(0.1);
    motor3 = 0.0;
    motor3.period(1.0 / 500.0);
    motor4.period(1.0 / 2000.0);
    motor4 = 0.1;
    wait(0.1);
    motor4 = 0.0;
    motor4.period(1.0 / 500.0);
    wait(1);
    armed = true;
  }

  void disarm() {
    actuate(0, 0, 0, 0);
    led_red = true;
    led_green = !true;
    armed = false;
  }

  void actuate(float f_t, float tau_phi, float tau_theta, float tau_psi) {
    mixer(f_t, tau_phi, tau_theta, tau_psi);
    if (armed) {
      motor1 = control_motor(omega_r_1);
      motor2 = control_motor(omega_r_2);
      motor3 = control_motor(omega_r_3);
      motor4 = control_motor(omega_r_4);
    }
  }

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

  float control_motor(float omega_rad) {
    float PWM = powf(omega_rad, 2) * a2 + omega_rad * a1;

    return PWM;
  }

  void mixer(float f_t, float tau_phi, float tau_theta, float tau_psi) {
    float w1_sqrd = 1 / (4 * kl) * f_t - 1 / (4 * kl * l) * tau_phi -
                    1 / (4 * kl * l) * tau_theta - 1 / (4 * kd) * tau_psi;
    float w2_sqrd = 1 / (4 * kl) * f_t - 1 / (4 * kl * l) * tau_phi +
                    1 / (4 * kl * l) * tau_theta + 1 / (4 * kd) * tau_psi;
    float w3_sqrd = 1 / (4 * kl) * f_t + 1 / (4 * kl * l) * tau_phi +
                    1 / (4 * kl * l) * tau_theta - 1 / (4 * kd) * tau_psi;
    float w4_sqrd = 1 / (4 * kl) * f_t + 1 / (4 * kl * l) * tau_phi -
                    1 / (4 * kl * l) * tau_theta + 1 / (4 * kd) * tau_psi;

    if (w1_sqrd < 0) {
      omega_r_1 = 0;
    } else {
      omega_r_1 = sqrt(w1_sqrd);
    }

    if (w2_sqrd < 0) {
      omega_r_2 = 0;
    } else {
      omega_r_2 = sqrt(w2_sqrd);
    }

    if (w3_sqrd < 0) {
      omega_r_3 = 0;
    } else {
      omega_r_3 = sqrt(w3_sqrd);
    }

    if (w4_sqrd < 0) {
      omega_r_4 = 0;
    } else {
      omega_r_4 = sqrt(w4_sqrd);
    }
    return;
  }
};