#include "mbed.h"
#include "crazyflie.h"
#include "USBSerial.h"

// USB serial object
USBSerial serial;
// Crazyflie controller objects
AttitudeEstimator att_est;
VerticalEstimator ver_est;
VerticalController ver_cont;

// Define ticker
Ticker tic;
Ticker tic_range;

// Define interrupt flags
bool flag;
bool flag_range = false;

// Define callback functions
void callback() { flag = true; }
void callback_range() { flag_range = true; }


float r = 1.0;
// Main program
int main() {
  // Initialize estimator objects
  att_est.init();
  ver_est.init();
  // Initialize interrupts
  tic.attach(&callback, dt);
  tic_range.attach(&callback_range, dt_vert);
  while (true) {
    // Estimate attitude and predict vertical position
    if (flag) {
      flag = false;
      att_est.estimate();
      ver_est.predict();
    }
    // Correct vertical position and print values
    if (flag_range) {
      flag_range = false;
      ver_est.correct(att_est.theta, att_est.phi);
      ver_cont.control(ver_est.z, ver_est.w, r);
      serial.printf("z [m ]:%6.2f | w [m/s ]:%6.2f | Ft[N]: %6.2f\n", ver_est.z, ver_est.w, ver_cont.Ft);
    }
  }
}