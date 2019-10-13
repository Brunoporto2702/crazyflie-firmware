#ifndef attitude_estimator_h
#define attitude_estimator_h

#include "mbed.h"
#include "src/utils/pin_names.h"
#include "src/utils/parameters.h"
#include "src/drivers/mpu9250.h"

//Attitude estimator class

class AttitudeEstimator 
{
    public:
        //Class Contructor
        AttitudeEstimator();

        //Initialize class
        void init();

        //Estimate Euler angles (rad)and angular velocities (rad/s)
        void estimate();

        //Euler angles (rad)
        float phi, theta, psi;

        //Angular velocities (rad/s)
        float p,q,r;

        //Pega sinal do número (usada para o sinal do az - teoria)
        float get_sign(float num);

    private:
        MPU9250 imu;

        float p_bias;
        float q_bias;
        float r_bias;

        DigitalOut led_blue;
};

#endif