#ifndef vertical_estimator_h
#define vertical_estimator_h

#include "mbed.h"
#include "crazyflie.h"
#include "src/utils/pin_names.h"
#include "src/utils/parameters.h"
#include "src/drivers/mpu9250.h"

#include "vl53l0x.h"

//Attitude estimator class

class VerticalEstimator 
{
    public:
        //Class Contructor
        VerticalEstimator();

        //Initialize class
        void init();

        //Estimate Euler angles (rad)and angular velocities (rad/s)
        void predict();

        //Compansate distance based on attitute 
        void correct(float theta, float phi);

        //Real Z distance (rad)
        float z;

        //Z velocity (rad/s)
        float w;

    private:
        VL53L0X range;

        float z_m_last;
};

#endif