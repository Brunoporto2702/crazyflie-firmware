#include "crazyflie.h"
#include "mbed.h"
#include <cmath>
// #include "USBSerial.h"

// USB serial object
// USBSerial serial;

// Declaring MIXER object mixer
MIXER Mixer;

// Declaring attittude estimantor object
AttitudeEstimator att_est;
// Declaring vertical estimator object
VerticalEstimator ver_est;

// Declaring attitude controler
AttitudeController att_cont;
// Declaring vertical Controller
VerticalController ver_cont;

Timer tim;

Ticker tic;
Ticker tic_range;

bool flag;
bool flag_range = false;

void callback() { flag = true; }

void callback_range() { flag_range = true; }

// Main program
int main() {
  float phi_r = 0.0f;
  float theta_r = 0.0f;
  float psi_r = 0.0f;
  float r_r = 0.0f;
  float q_r = 0.0f;
  float p_r = 0.0f;

  // vert controller params
  float r = 0.0;

  Mixer.arm();
  att_est.init();
  ver_est.init();
  tic.attach(&callback, dt);
  tic_range.attach(&callback_range, dt_vert);

  wait(5.0);
  tim.start();
  float t = 0;
  while (t < 10.0 && (att_est.p < 2 * 3.14) && (att_est.q < 3.14 * 4) &&
         (att_est.r < 3.14 * 4) && (abs(att_est.theta) < 3.14 / 4) &&
         (abs(att_est.phi) < 3.14 / 4) && (abs(att_est.psi) < 3.14 / 4)) {
    if (flag) {
      flag = false;
      att_est.estimate();
      ver_est.predict();

      att_cont.control(phi_r, theta_r, psi_r, att_est.phi, att_est.theta,
                       att_est.psi, att_est.p, att_est.q, att_est.r);

      if (t * 0.1 < 0.5) {
        r = t * 0.1;
      } else {
        r = 0.5;
      }

      // vertical controler
      ver_cont.control(ver_est.z, ver_est.w, r);

      // Actuate motor with 70% mg total trust force (N) and zero torques
      Mixer.actuate(ver_cont.Ft / (cos(att_est.theta) * cos(att_est.phi)),
                    att_cont.tau_phi, att_cont.tau_theta, att_cont.tau_psi);
    }

    if (flag_range) {
      flag_range = false;
      ver_est.correct(att_est.theta, att_est.phi);
    }

    t = tim.read();
  }
  Mixer.disarm();
}