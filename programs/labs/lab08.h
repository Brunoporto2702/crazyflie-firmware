#include "crazyflie.h"
#include "mbed.h"
#include "mixer.h"
#include <cmath>


// Declaring MIXER object mixer
MIXER Mixer;

// Declaring attittude estimantor object
AttitudeEstimator att_est;

Timer tim;

// Main program
int main() {
    Mixer.arm();
    att_est.init();
    wait(5.0);
    tim.start();
    float t = 0;
    while(t<4.0 && (att_est.p<2*3.14) && (att_est.q<3.14*4) && (att_est.r<3.14*4) && (abs(att_est.theta) < 3.14/4) && (abs(att_est.phi)<3.14/4) && (abs(att_est.psi)<3.14/4))
    {
        att_est.estimate();

        float Torque_theta = -att_est.theta * 0.00071*2 - att_est.q * 0.005;

        float Torque_phi = -att_est.phi * 0.00071*2 - att_est.p * 0.005;

        float Torque_psi = -att_est.psi * 0.00071*2 - att_est.r * 0.005;

        // Actuate motor with 70% mg total trust force (N) and zero torques
        Mixer.actuate(0.7 * m * g, Torque_phi, Torque_theta, Torque_psi);


        t = tim.read();
    }
    Mixer.disarm();
}
