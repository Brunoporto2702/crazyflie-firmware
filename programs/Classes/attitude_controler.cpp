# include "mbed.h"
# include "attitude_controler.h"

// Class constructor
AttitudeController :: AttitudeController ()
{
    tau_psi = 0;
    tau_phi = 0;
    tau_theta = 0;
}

// Control torques (N.m) given reference angles (rad) and current angles ( rad ) and angular velocities ( rad /s)
void AttitudeController :: control(float phi_r, float theta_r, float psi_r, float phi, float theta, float psi, float p, float q, float r)
{
    tau_theta = -theta * 0.00071*2 - q * 0.005;

    tau_phi = -phi * 0.00071*2 - p * 0.005;

    tau_psi = -psi * 0.00071*2 - r * 0.005;
}

// // Control torque (N.m) given reference angle ( rad ) and current angle ( rad ) and angular velocity ( rad /s) with given controller gains
// float AttitudeController :: control_state_regulator(float angle_r, float angle, float rate, float kp, float kd)
// {
// (...)
// }