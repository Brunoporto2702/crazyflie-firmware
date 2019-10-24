# include "mbed.h"
# include "vertical_controller.h"

// Class constructor
VerticalController :: VerticalController ()
{
    Ft = 0;
}

// Control torques (N.m) given reference angles (rad) and current angles ( rad ) and angular velocities ( rad /s)
void VerticalController :: control(float z, float w, float r)
{

    if (z< (0.3* r)){
        deploy();
    }
    Ft = m*(k1_vert * (r - z) - k2_vert * w + g);
}


void VerticalController::deploy( )
{
    Ft = 0.53*m*g;
}