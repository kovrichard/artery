/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * apply_cfo.cpp
 *
 * Code generation for function 'apply_cfo'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "apply_cfo.h"
#include "sim_rx_mex_emxutil.h"

/* Function Definitions */
void apply_cfo(const emxArray_creal_T *in, double cfo, emxArray_creal_T *out)
{
  emxArray_real_T *y;
  double r;
  double ai;
  double z_re;
  int nx;
  int loop_ub;
  emxArray_creal_T *b_y;
  double y_im;
  emxInit_real_T(&y, 2);

  /* APPLY_CFO Apply frequency offset to input waveform */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    February 2019; Last revision: 22-February-2019 */
  /*  Copyright (C) u-blox */
  /*  */
  /*  All rights reserved. */
  /*  */
  /*  Permission to use, copy, modify, and distribute this software for any */
  /*  purpose without fee is hereby granted, provided that this entire notice */
  /*  is included in all copies of any software which is or includes a copy */
  /*  or modification of this software and in all copies of the supporting */
  /*  documentation for such software. */
  /*  */
  /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
  /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
  /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
  /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
  /*  */
  /*  Project: ubx-v2x */
  /*  Purpose: V2X baseband simulation model */
  /*  Frequency shift */
  r = cfo * 0.0 * 5.9E+9;
  ai = cfo * 6.2831853071795862 * 5.9E+9;
  if (ai == 0.0) {
    z_re = r / 1.0E+7;
    r = 0.0;
  } else if (r == 0.0) {
    z_re = 0.0;
    r = ai / 1.0E+7;
  } else {
    z_re = rtNaN;
    r = ai / 1.0E+7;
  }

  if (in->size[0] - 1 < 0) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    ai = (double)in->size[0] - 1.0;
    nx = y->size[0] * y->size[1];
    y->size[0] = 1;
    loop_ub = (int)ai;
    y->size[1] = loop_ub + 1;
    emxEnsureCapacity_real_T(y, nx);
    for (nx = 0; nx <= loop_ub; nx++) {
      y->data[nx] = nx;
    }
  }

  emxInit_creal_T(&b_y, 2);
  nx = b_y->size[0] * b_y->size[1];
  b_y->size[0] = 1;
  b_y->size[1] = y->size[1];
  emxEnsureCapacity_creal_T(b_y, nx);
  loop_ub = y->size[0] * y->size[1];
  for (nx = 0; nx < loop_ub; nx++) {
    b_y->data[nx].re = y->data[nx] * z_re;
    b_y->data[nx].im = y->data[nx] * r;
  }

  emxFree_real_T(&y);
  nx = b_y->size[1];
  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    if (b_y->data[loop_ub].im == 0.0) {
      ai = b_y->data[loop_ub].re;
      b_y->data[loop_ub].re = std::exp(ai);
      b_y->data[loop_ub].im = 0.0;
    } else if (rtIsInf(b_y->data[loop_ub].im) && rtIsInf(b_y->data[loop_ub].re) &&
               (b_y->data[loop_ub].re < 0.0)) {
      b_y->data[loop_ub].re = 0.0;
      b_y->data[loop_ub].im = 0.0;
    } else {
      r = std::exp(b_y->data[loop_ub].re / 2.0);
      y_im = b_y->data[loop_ub].im;
      ai = b_y->data[loop_ub].im;
      b_y->data[loop_ub].re = r * (r * std::cos(y_im));
      b_y->data[loop_ub].im = r * (r * std::sin(ai));
    }
  }

  /*  Apply shift */
  nx = out->size[0];
  out->size[0] = in->size[0];
  emxEnsureCapacity_creal_T(out, nx);
  loop_ub = in->size[0];
  for (nx = 0; nx < loop_ub; nx++) {
    r = in->data[nx].re;
    z_re = in->data[nx].im;
    ai = b_y->data[nx].re;
    y_im = b_y->data[nx].im;
    out->data[nx].re = r * ai - z_re * y_im;
    out->data[nx].im = r * y_im + z_re * ai;
  }

  emxFree_creal_T(&b_y);
}

/* End of code generation (apply_cfo.cpp) */
