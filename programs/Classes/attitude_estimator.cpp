#include "mbed.h"
#include "attitude_estimator.h"
#include <cmath>

//Class constructor
AttitudeEstimator::AttitudeEstimator()  :  imu(IMU_SDA,IMU_SCL),led_blue(LED_BLUE_L,!false)
{
    phi = 0;
    theta =0;
    psi = 0;
    p = 0;
    q = 0;
    r = 0;
    p_bias = 0;
    led_blue = 0;
}

//Initialize Class
void AttitudeEstimator::init()
{
    wait(1.0);
    imu.init();

    // calculando media para compensar erro estático
    for (int i = 0; i<500; i++)
    {
        imu.read();
        p_bias = p_bias + imu.gx/500.0;
        q_bias = q_bias + imu.gy/500;
        r_bias = r_bias + imu.gz/500;
        wait(dt);
        if (i%10 == 0)
        {
            led_blue = !led_blue;
        }
    }
    led_blue = 0;
}

//Estimate Euler angles (rad)and angular velocities (rad/s)
void AttitudeEstimator::estimate()
{
    imu.read();
    
    // removendo erro estático
    p = imu.gx - p_bias;
    q = imu.gy - q_bias;
    r = imu.gz - r_bias;

    // integrando o valor lido pelo giroscópio
    float phi_g = phi + (p*dt + sin(phi)*tan(theta)*q*dt + cos(phi)*tan(theta)*r*dt);
    // Aquisitando valor lido pelo acelerômetro
    float phi_a = atan2(-imu.ay,-imu.az);

    // integrando o valor lido pelo giroscópio
    float theta_g = theta + (cos(phi)*q*dt-sin(phi)*r*dt);
    // Aquisitando valor lido pelo acelerômetro
    float theta_a = atan2(imu.ax,-get_sign(imu.az)*sqrt(pow(imu.az,2)+pow(imu.ay,2)));

    // integrando o valor lido pelo giroscópio
    float psi_g = psi + (sin(phi)*1/cos(theta)*q*dt+cos(phi)*1/cos(theta)*r*dt);

    // FIltro complementar - Acelerômetro/Giroscópio (juntando os dois)
    phi = (1-alpha_lff)*(phi_g)+alpha_lff*phi_a;
    theta = (1-alpha_lff)*theta_g+alpha_lff*theta_a;
    psi = psi_g;
}

float AttitudeEstimator::get_sign(float num)
{
    if (num <0)
    {
        return -1;
    }
    else 
    {
        return 1;
    }
}