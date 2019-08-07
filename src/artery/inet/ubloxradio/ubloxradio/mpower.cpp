/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mpower.cpp
 *
 * Code generation for function 'mpower'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "mpower.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
double mpower(double a, double b)
{
  return rt_powd_snf(a, b);
}

/* End of code generation (mpower.cpp) */
