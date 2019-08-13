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

//Parâmetros de controle de rotação em malha aberta

const float a1 = -4.7254e-5f;   //parametro que multiplica o termo de primeiro grau
const float a2 = 1.9527e-7f;   //parametro que multiplica o termo de segundo grau

#endif