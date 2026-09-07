#include "controller.h" 
#include <math.h>

void controller_init(ControllerState *st);


void controller_step(ControllerState *st, const ControllerParams *p, float ia, float ib, float ic, float id_ref, float iq_ref, float theta, float *va, float *vb, float *vc){


// clarke transform

float mag_inv = 2/3;
float i_alpha =  mag_inv*(ia - 0.5f*ib - 0.5*ic);
float i_beta = mag_inv*(sqrtf(3)/2)(ib-ic);

// park transform

id =  i_alpha*cosf(theta) + i_beta * sinf(theta);
iq = -i_alpha*sinf(theta) + i_beta * cosf(theta);

//controller update

float ctrl_update(ControllerParams *p, ControllerState *st, float ref, float meas){

    float error = ref - meas;
    float prop = p->Kp*error;
    float next_integral = p->Ki * error * p->Ts;

    float ctr_effort = prop + next_integral;
    
    // anti windup
    if (ctr_effort > v_max) {
      ctr_effort = v_max;
    } else if(ctr_effort < -v_max){
      ctrl_effort = -v_max;
    } else{
      st->integ_d = next_integral;
    }

  }

}


