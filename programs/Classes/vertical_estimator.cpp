#include "mbed.h"
#include "vertical_estimator.h"
#include <cmath>

//Class constructor
VerticalEstimator::VerticalEstimator()  :  range(E_SDA, E_SCL)
{
    z = 0;
    w = 0;
    z_m_last = 0;
}

//Initialize Class
void VerticalEstimator::init()
{
    wait(1.0);
    range.init();
}

//Estimate Euler angles (rad)and angular velocities (rad/s)
void VerticalEstimator::predict()
{
    z = z + w*dt; 
}

void VerticalEstimator::correct(float theta, float phi)
{
    range.read();
    if(range.d<2.0)
    {
        float z_m = range.d*cos(phi)*cos(theta);
        float w_m = (z_m - z_m_last)/dt_vert;
        z_m_last = z_m;
        z = z + ro_vert2*(z_m-z);
        w = w + ro_vert1*(w_m-w);
    }
}