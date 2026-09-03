typedef struct{
  float Kp, Ki;
  float  Ts;
  float v_max;
} ControllerParams;

typedef struct{
  float integral_d;
  float integral_q;
} ControllerState;

void controller_init(ControllerState *st);

void controller_step(ControllerState *st, const ControllerParams *p, float ia, float ib, float ic, float id_ref, float iq_ref, float theta, float *va, float *vb, float *vc);
