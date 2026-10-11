typedef struct {
float Kp, Ki;       // PI gains
float  Ts;          // sample period, s
float omega_nom;    // nominal grid angular frequency (rad/s)
float omega_max;   // frequency correction sauration (rad/s)
float ema_alpha;    // lock filter coefficient
float lock_thresh;  // |vq| filtered below this counts toward lock
int lock_debounce;  // consecutive samples required to declare lock
} PLLParams;

typedef struct {
float theta_hat;  // current angle estimate (rad)
float omega_hat;  // current frequency estimate (rad/s)
float integ;      // the PI's integrator state
float vq_filt;    // the lock filter's running EMA of |vq|
int lock_count; // debounce counter
int locked;       // the lock flag
}PLLState;

void pll_init(PLLState *pll, const PLLParams *p, float valpha0, float vbeta0);

void pll_step(PLLState *ppl, const PLLParams *p, float valpha, float vbeta );