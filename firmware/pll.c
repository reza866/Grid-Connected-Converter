#include<math.h>


void pll_init(PLLState *pll, const PLLParams *p, float valpha0, float vbeta0){
/* TODO 3: pll_init
 * Seed theta_hat from atan2f(vbeta0, valpha0) -- the practical point about
 * initialization. Set omega_hat to p->omega_nom, and zero every other state
 * field (integ, vq_filt, lock_count, locked).*/
    pll->theta_hat = atan2f(vbeta0,valpha0);
    pll->omega_hat = p->omega_nom;
    pll->integ = 0;
    pll->vq_filt = 0;
    pll->lock_count = 0;
    pll->locked = 0;
}

void pll_step(PLLState *ppl, const PLLParams *p, float valpha, float vbeta ){
    /* 4a: call park_transform to get vd, vq at the CURRENT theta_hat.
     *     (vd is computed but not otherwise used by the PLL itself.)
     */
    float vd, vq;
    /* TODO */

    /* 4b: PI with clamped anti-windup, same structure as Day 1/Day 6:
     *   error = vq
     *   prop = Kp * error
     *   integ_next = integ + Ki*Ts*error
     *   domega_raw = prop + integ_next
     *   domega = clampf(domega_raw, -domega_max, domega_max)
     *   use saturated(domega_raw, -domega_max, domega_max) to decide
     *   whether to commit integ_next to pll->integ (freeze if saturated
     *   AND error would push further into saturation).
     *   Then: pll->omega_hat = p->omega_nom + domega;
     */
    /* TODO */

    /* 4c: advance and wrap the angle:
     *   pll->theta_hat = wrap_angle(pll->theta_hat + pll->omega_hat * p->Ts);
     */
    /* TODO */

    /* 4d: lock metric -- EMA-filter |vq| into pll->vq_filt, then debounce:
     *   if vq_filt < lock_thresh, increment lock_count and set locked=1
     *   once lock_count reaches lock_debounce; otherwise reset both to 0.
     */
    /* TODO */
}