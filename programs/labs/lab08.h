#include "crazyflie.h"
#include "mbed.h"
#include <cmath>


// Declaring MIXER object mixer
MIXER Mixer;

// Declaring attittude estimantor object
AttitudeEstimator att_est;

// Declaring attitude controler
AttitudeController att_cont;

Timer tim;

// Main program
int main() {
    float phi_r = 0.0f;
    float theta_r = 0.0f;
    float psi_r = 0.0f;
    float r_r = 0.0f;
    float q_r = 0.0f;
    float p_r = 0.0f;
    Mixer.arm();
    att_est.init();
    wait(5.0);
    tim.start();
    float t = 0;
    while(t<4.0 && (att_est.p<2*3.14) && (att_est.q<3.14*4) && (att_est.r<3.14*4) && (abs(att_est.theta) < 3.14/4) && (abs(att_est.phi)<3.14/4) && (abs(att_est.psi)<3.14/4))
    {
        att_est.estimate();

        att_cont.control(phi_r, theta_r, psi_r, att_est.phi, att_est.theta, att_est.psi, att_est.p, att_est.q, att_est.r);

        // Actuate motor with 70% mg total trust force (N) and zero torques
        Mixer.actuate(0.53 * m * g, att_cont.tau_phi, att_cont.tau_theta, att_cont.tau_psi);


        t = tim.read();
    }
    Mixer.disarm();
}
