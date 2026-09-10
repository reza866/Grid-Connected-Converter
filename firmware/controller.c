#include "controller.h" 
#include <math.h>

void controller_init(ControllerState *st);


// clarke transform
void clarke(float a, float b, float c, float *alpha, float *beta){

float mag_inv = 2.0f/3.f0;
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

void controller_step(ControllerState *st, const ControllerParams *p, float ia, float ib, float ic, float id_ref, float iq_ref, float theta, float *va, float *vb, float *vc){








//controller update

float ctrl_update(ControllerParams *p, ControllerState *st, float ref, float meas){

    float error = ref - meas;
    float prop = p->Kp*error;
    float next_integral = p->Ki * error * p->Ts;

    float ctr_effort = prop + next_integral;
    float k_anti;
    // anti windup
    if (ctr_effort > v_max) {
      ctr_effort = v_max;
      st->integ_d = next_integral - k_anti *(ctr_effort -v_max); 
    } else if(ctr_effort < -v_max){
      ctrl_effort = -v_max;
      st->integ_d = next_integral - k_anti *(ctr_effort -v_max);
    } else{
      st->integ_d = next_integral;
    }

  }

}


