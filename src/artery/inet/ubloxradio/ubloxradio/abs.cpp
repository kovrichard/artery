/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * abs.cpp
 *
 * Code generation for function 'abs'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "abs.h"
#include "kaiser.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void b_abs(const creal_T x[127], double y[127])
{
  int k;
  for (k = 0; k < 127; k++) {
    y[k] = rt_hypotd_snf(x[k].re, x[k].im);
  }
}

void c_abs(const double x[48], double y[48])
{
  int k;
  for (k = 0; k < 48; k++) {
    y[k] = std::abs(x[k]);
  }
}

void d_abs(const double x_data[], const int x_size[2], double y_data[], int
           y_size[2])
{
  int nx;
  int k;
  nx = x_size[1];
  y_size[0] = 1;
  y_size[1] = (signed char)x_size[1];
  for (k = 0; k < nx; k++) {
    y_data[k] = std::abs(x_data[k]);
  }
}

/* End of code generation (abs.cpp) */
