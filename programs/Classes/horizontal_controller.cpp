# include "mbed.h"
# include "horizontal_controller.h"

// Class constructor
HorizontalController :: HorizontalController ()
{
    theta_r = 0;
    phi_r = 0;
}

// Control torques (N.m) given reference angles (rad) and current angles ( rad ) and angular velocities ( rad /s)
void HorizontalController :: control(float x, float y, float x_r, float y_r, float vx, float vy)
{
    theta_r = k1_hor * x + k2_hor *vx;
    phi_r = k1_hor * x + k2_hor *vy;
}


