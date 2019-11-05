#ifndef horizontal_estimator_h
#define horizontal_estimator_h

#include "mbed.h"
#include "crazyflie.h"
#include "src/utils/pin_names.h"
#include "src/utils/parameters.h"
#include "src/drivers/mpu9250.h"

#include "drivers/pmw3901.h"

//Attitude estimator class

class HorizontalEstimator 
{
    public:
        //Class Contructor
        HorizontalEstimator();

        //Initialize class
        void init();

        //Estimate Euler angles (rad)and angular velocities (rad/s)
        void predict();

        //Compansate distance based on attitute 
        void correct(float z, float theta, float phi, float q, float p);

        //distance in meters 
        float delta_x;
        float delta_y;

        //Z velocity (rad/s)
        float v_x;
        float v_y;

        // position in meters
        float x;
        float y;


    private:
        PMW3901 flow;

        float v_x_last;
        float v_y_last;

        
};

#endif