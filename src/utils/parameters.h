#ifndef parameters_h
#define parameters_h
#include <cmath>



// Physical constants
const float pi = 3.1416f;
const float g = 9.81f;              // m/s^2

// Quadcopter dimensions
const float m = 30e-3f;             // kg
const float I_xx = 16.0e-6f;        // kg.m^2
const float I_yy = 16.0e-6f;        // kg.m^2
const float I_zz = 29.0e-6f;        // kg.m^2
const float l = 33e-3f;             // m

//Parâmetros de controle de rotação do motor em malha aberta  (lab 02)

const float a1 = -4.7254e-5f;   //parametro que multiplica o termo de primeiro grau
const float a2 = 1.9527e-7f;   //parametro que multiplica o termo de segundo grau 

//Parametros aerodinâmicos do drone   (lab 03)

const float kl = 2.2591e-8f; //constante de sustentação

//Parametros aerodinâmicos do drone   (lab 04)

const float kd = 1.6754e-10f; //constante de arrasto

//Intervalo de tempo para estimador de attitude
const float dt = 2.0/1000.0; //s

//Intervalo de tempo para estimador vertical
const float dt_vert = 50.0/1000.0; //s

// constantes do estimador vertical
const float ro_vert1 = 0.30;
const float ro_vert2 = 0.30;

// Ganhos k1 e k2 controlador de attitude
// const float k1_att = 0.00071*0.5;
// const float k2_att = 0.005;
const float k1_att = 4;
const float k2_att = 16;

// Ganhos k1 e k2 controlador vertical
const float k1_vert = 5.8;//0.24;
const float k2_vert = 3.4;//0.12;
// const float k1_vert = 1.1;
// const float k2_vert = 6.0;

// constantes de conversão de pixels para metros (sensor flow)
const float alfa_flow = 42.0/180.0*pi;
const float w_pixels = 420.0;
const float sigma_delta_p = 2.0*tan(alfa_flow/2.0)/w_pixels;
const float sigma_v = sigma_delta_p/dt;

//parâmetros alfa filtro passa baixa vel. flow
const float alfa_lff_flow = 0.5;

// Filtro passa baixa (Parâmetros)
//float omega_c = 6.2;  // frequencia de corte
//float alpha = (omega_c*dt)/(1+omega_c*dt);
const float alpha_lff = 0.01;  

// Ganhos controlador horizontal
// const float OS = 0.5;
// const float zeta = sqrt((pow(log(OS/100.0),2.0))/(pow(pi,2.0)+pow(log(OS/100.0),2.0)));
// const float ts = 4; //segundos
// const float wn = ts*zeta/4;


// const float k1_hor = 0.265;
// const float k2_hor = 1.0;
const float k1_hor = 1.0;
const float k2_hor = 6.0;


#endif