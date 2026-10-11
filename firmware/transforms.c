#include<math.h>
#include "transforms.h"

// clarke transform
void clarke(float a, float b, float c, float *alpha, float *beta){

float mag_inv = 2.0f/3.0f;
*alpha =  mag_inv*(a - 0.5f*b - 0.5*c);
*beta = mag_inv*(sqrtf(3)/2)*(b-c);
}


// park transform

void park(float alpha, float beta, float *d, float *q, float theta){

*d =  alpha*cosf(theta) + beta * sinf(theta);
*q = -alpha*sinf(theta) + beta * cosf(theta);
}

// inv park

void inv_park(float d, float q, float *alpha, float *beta, float theta){
  
  *alpha = d*cosf(theta) - q*sinf(theta); 
  *beta  = d*sinf(theta) + q*cosf(theta);
}

// inv clarke

void inv_clarke(float alpha, float beta, float *a, float *b, float *c){

  *a= alpha;
  *b= -0.5f*alpha + sqrtf(3)*0.5f*beta;
  *c= -0.5f*alpha - sqrtf(3)*0.5f*beta;
}