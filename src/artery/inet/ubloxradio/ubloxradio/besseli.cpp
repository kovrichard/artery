/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * besseli.cpp
 *
 * Code generation for function 'besseli'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "besseli.h"
#include "cbesi.h"

/* Function Definitions */
creal_T besseli(double z)
{
  creal_T w;
  creal_T zd;
  int unusedU0;
  int ierr;
  double w_re;
  zd.re = z;
  zd.im = 0.0;
  if (rtIsNaN(z)) {
    w.re = rtNaN;
    w.im = 0.0;
  } else {
    cbesi(zd, &w, &unusedU0, &ierr);
    if (ierr == 5) {
      w.re = rtNaN;
      w.im = 0.0;
    } else {
      if (ierr == 2) {
        w.re = rtInf;
        w.im = 0.0;
      }
    }

    if (z > 0.0) {
      w_re = w.re;
      w.re = w_re;
      w.im = 0.0;
    }
  }

  return w;
}

/* End of code generation (besseli.cpp) */
