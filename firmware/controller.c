#include "controller.h" 
#include <math.h>

void controller_init(ControllerState *st);


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

//controller update d-axis

float ctrl_update_d(ControllerParams *p, ControllerState *st, float ref, float meas){

    float error = ref - meas;
    float prop = p->Kp*error;
    float next_integral = p->Ki * error * p->Ts;

    float ctrl_effort = prop + next_integral;
    // TO DO
    float k_anti = 1;
    // anti windup
    if (ctrl_effort > p->v_max) {
      ctrl_effort = p->v_max;
      st->integral_d = next_integral - k_anti *(ctrl_effort -(p->v_max)); 
    } else if(ctrl_effort < -(p->v_max)){
      ctrl_effort = -(p->v_max);
      st->integral_d = next_integral - k_anti *(ctrl_effort -(p->v_max));
    } else{
      st->integral_d = next_integral;
    }

  }

//controller update q-axis

float ctrl_update_d(ControllerParams *p, ControllerState *st, float ref, float meas){

    float error = ref - meas;
    float prop = p->Kp*error;
    float next_integral = p->Ki * error * p->Ts;

    float ctrl_effort = prop + next_integral;
    //TO DO
    float k_anti = 1;
    // anti windup
    if (ctrl_effort > p->v_max) {
      ctrl_effort = p->v_max;
      st->integral_q = next_integral - k_anti *(ctrl_effort -(p->v_max)); 
    } else if(ctrl_effort < -(p->v_max)){
      ctrl_effort = -(p->v_max);
      st->integral_q = next_integral - k_anti *(ctrl_effort -(p->v_max));
    } else{
      st->integral_q = next_integral;
    }

  }


void controller_step(ControllerState *st, const ControllerParams *p, float ia, float ib, float ic, float id_ref, float iq_ref, float theta, float *va, float *vb, float *vc){

float i_alpha = 0.0f;
float i_beta = 0.0f;
clarke(ia, ib, ic, &i_alpha, &i_beta);


float id = 0.0f;
float iq = 0.0f;
park(i_alpha, i_beta, &id, &iq, theta);

float vd = 0.0f;
float vq = 0.0f;

vd = ctrl_update_d(p, st, id_ref, id);
vq = ctrl_update_q(p, st, iq_ref, iq);

float v_alpha = 0.0f;
float v_beta = 0.0f;

inv_park(vd, vq, &v_alpha, &v_beta, theta);

float va = 0.0f; float vb = 0.0f; float vc = 0.0f;

inv_clarke(v_alpha, v_beta, &va, &vb, &vc);

}






