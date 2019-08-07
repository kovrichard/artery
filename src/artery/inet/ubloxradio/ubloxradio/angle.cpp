/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * angle.cpp
 *
 * Code generation for function 'angle'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "angle.h"
#include "cmlri.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void angle(const creal_T x[4], double y[4])
{
  y[0] = rt_atan2d_snf(x[0].im, x[0].re);
  y[1] = rt_atan2d_snf(x[1].im, x[1].re);
  y[2] = rt_atan2d_snf(x[2].im, x[2].re);
  y[3] = rt_atan2d_snf(x[3].im, x[3].re);
}

/* End of code generation (angle.cpp) */
