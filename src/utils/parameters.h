#ifndef parameters_h
#define parameters_h

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
const float k1_att = 0.00071*1.5;
const float k2_att = 0.005;

// Ganhos k1 e k2 controlador vertical
const float k1_vert = 5.8;//0.24;
const float k2_vert = 3.4;//0.12;


// Filtro passa baixa (Parâmetros)
//float omega_c = 6.2;  // frequencia de corte
//float alpha = (omega_c*dt)/(1+omega_c*dt);
const float alpha_lff = 0.01;  

#endif