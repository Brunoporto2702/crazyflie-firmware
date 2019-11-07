#include "mbed.h"
#include "crazyflie.h"
#include "USBSerial.h"

// USB serial object
USBSerial serial;
// Crazyflie controller objects
AttitudeEstimator att_est;
VerticalEstimator ver_est;
HorizontalEstimator hor_est;

HorizontalController hor_cont;

// Define ticker
Ticker tic;
Ticker tic_range;

// Define interrupt flags
bool flag;
bool flag_range = false;

// Define callback functions
void callback() { flag = true; }
void callback_range() { flag_range = true; }

// Main program
int main() {
  // Initialize estimator objects
  att_est.init();
  ver_est.init();
  hor_est.init();
  // Initialize interrupts
  tic.attach(&callback, dt);
  tic_range.attach(&callback_range, dt_vert);
  while (true) {
    // Estimate attitude and predict vertical position
    if (flag) {
      flag = false;
      att_est.estimate();
      ver_est.predict();
      hor_est.correct(ver_est.z, att_est.theta, att_est.phi, att_est.q, att_est.p);
      hor_cont.control(hor_est.x, hor_est.y, 0, 0, hor_est.v_x, hor_est.v_y);
      serial.printf("px [m ]:%6.2f | py [m/s ]:%6.2f vx [m/s ]:%6.2f | vy [m/s ]:%6.2f    theta [rad]: %6.2f    phi[rad]: %6.2f \n", hor_est.x, hor_est.y, hor_est.v_x, hor_est.v_y, hor_cont.theta_r, hor_cont.phi_r);
    }
    // Correct vertical position and print values
    if (flag_range) {
      flag_range = false;
      ver_est.correct(att_est.theta, att_est.phi);
    }
  }
}