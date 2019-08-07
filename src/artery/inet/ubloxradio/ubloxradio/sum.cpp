/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sum.cpp
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sum.h"

/* Function Definitions */
creal_T sum(const creal_T x[64])
{
  creal_T y;
  int k;
  y = x[0];
  for (k = 0; k < 63; k++) {
    y.re += x[k + 1].re;
    y.im += x[k + 1].im;
  }

  return y;
}

/* End of code generation (sum.cpp) */
