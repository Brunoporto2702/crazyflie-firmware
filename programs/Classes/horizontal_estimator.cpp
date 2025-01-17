#include "mbed.h"
#include "horizontal_estimator.h"
#include <cmath>

//Class constructor
HorizontalEstimator::HorizontalEstimator()  :  flow(E_MOSI,E_MISO,E_SCK,E_CS1)
{
   delta_x = 0;
   delta_y = 0;
   v_x = 0;
    v_y = 0;
   v_x_last = 0;
   v_y_last = 0;
   x = 0;
   y = 0;
}

//Initialize Class
void HorizontalEstimator::init()
{
    flow.init();
}

//Estimate Euler angles (rad)and angular velocities (rad/s)
void HorizontalEstimator::predict()
{
    
}

void HorizontalEstimator::correct(float z, float theta, float phi, float q, float p)
{
    float d = z / (cos(theta)*cos(phi));
    flow.read();
    float delta_x_m = flow.px*sigma_delta_p;
    float delta_y_m = flow.py*sigma_delta_p; 
    float v_x_m = (sigma_v*flow.px + q)*d;
    float v_y_m = (sigma_v*flow.py - p)*d;
    // float v_x_m = (sigma_v*flow.px)*d;
    // float v_y_m = (sigma_v*flow.py)*d;
    //delta_x = delta_x_m;
    //delta_y = delta_y_m;
    v_x = (1.0-alfa_lff_flow)*v_x + alfa_lff_flow*v_x_m;
    v_y = (1.0-alfa_lff_flow)*v_y + alfa_lff_flow*v_y_m;
    x = x+v_x*dt;
    y = y+v_y*dt;
}