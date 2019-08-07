/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * kaiser.cpp
 *
 * Code generation for function 'kaiser'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "kaiser.h"
#include "besseli.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void kaiser(double BTA, double w[64])
{
  creal_T x;
  double bes;
  int k;
  double r;
  double x_im;
  x = besseli(BTA);
  bes = rt_hypotd_snf(x.re, x.im);
  for (k = 0; k < 32; k++) {
    r = (1.0 + 2.0 * (((double)k + 33.0) - 33.0)) / 63.0;
    x = besseli(BTA * std::sqrt((1.0 - r) * (1.0 + r)));
    r = x.re;
    x_im = x.im;
    if (x_im == 0.0) {
      x.re = r / bes;
      x.im = 0.0;
    } else if (r == 0.0) {
      x.re = 0.0;
      x.im = x_im / bes;
    } else {
      x.re = r / bes;
      x.im = x_im / bes;
    }

    w[k + 32] = rt_hypotd_snf(x.re, x.im);
  }

  for (k = 0; k < 32; k++) {
    w[k] = w[63 - k];
  }
}

/* End of code generation (kaiser.cpp) */
