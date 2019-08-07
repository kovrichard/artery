/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sqrt.cpp
 *
 * Code generation for function 'sqrt'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sqrt.h"
#include "kaiser.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void b_sqrt(creal_T *x)
{
  double xr;
  double xi;
  double yr;
  double absxr;
  xr = x->re;
  xi = x->im;
  if (xi == 0.0) {
    if (xr < 0.0) {
      yr = 0.0;
      xr = std::sqrt(-xr);
    } else {
      yr = std::sqrt(xr);
      xr = 0.0;
    }
  } else if (xr == 0.0) {
    if (xi < 0.0) {
      yr = std::sqrt(-xi / 2.0);
      xr = -yr;
    } else {
      yr = std::sqrt(xi / 2.0);
      xr = yr;
    }
  } else if (rtIsNaN(xr)) {
    yr = xr;
  } else if (rtIsNaN(xi)) {
    yr = xi;
    xr = xi;
  } else if (rtIsInf(xi)) {
    yr = std::abs(xi);
    xr = xi;
  } else if (rtIsInf(xr)) {
    if (xr < 0.0) {
      yr = 0.0;
      xr = xi * -xr;
    } else {
      yr = xr;
      xr = 0.0;
    }
  } else {
    absxr = std::abs(xr);
    yr = std::abs(xi);
    if ((absxr > 4.4942328371557893E+307) || (yr > 4.4942328371557893E+307)) {
      absxr *= 0.5;
      yr = rt_hypotd_snf(absxr, yr * 0.5);
      if (yr > absxr) {
        yr = std::sqrt(yr) * std::sqrt(1.0 + absxr / yr);
      } else {
        yr = std::sqrt(yr) * 1.4142135623730951;
      }
    } else {
      yr = std::sqrt((rt_hypotd_snf(absxr, yr) + absxr) * 0.5);
    }

    if (xr > 0.0) {
      xr = 0.5 * (xi / yr);
    } else {
      if (xi < 0.0) {
        xr = -yr;
      } else {
        xr = yr;
      }

      yr = 0.5 * (xi / xr);
    }
  }

  x->re = yr;
  x->im = xr;
}

/* End of code generation (sqrt.cpp) */
