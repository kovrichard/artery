/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * apply_time_window.cpp
 *
 * Code generation for function 'apply_time_window'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "apply_time_window.h"
#include "sim_rx_mex_emxutil.h"

/* Function Declarations */
static int div_nde_s32_floor(int numerator, int denominator);

/* Function Definitions */
static int div_nde_s32_floor(int numerator, int denominator)
{
  int b_numerator;
  if (((numerator < 0) != (denominator < 0)) && (numerator % denominator != 0))
  {
    b_numerator = -1;
  } else {
    b_numerator = 0;
  }

  return numerator / denominator + b_numerator;
}

void apply_time_window(const emxArray_creal_T *in, boolean_T enabled,
  emxArray_creal_T *out)
{
  int i9;
  int loop_ub;
  double in_re;
  double in_im;
  int idx_size_idx_1;
  int idx_data[2740];
  creal_T tmp_data[2740];
  creal_T b_tmp_data[2740];
  int c_tmp_data[2740];

  /* APPLY_TIME_WINDOW Apply time-domain window to improve spectral shape */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    February 2019; Last revision: 19-February-2019 */
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
  /*  Default output is equal to input */
  i9 = out->size[0];
  out->size[0] = in->size[0];
  emxEnsureCapacity_creal_T(out, i9);
  loop_ub = in->size[0];
  for (i9 = 0; i9 < loop_ub; i9++) {
    out->data[i9] = in->data[i9];
  }

  /*  Apply time-domain windowing */
  if (enabled) {
    /*  Length of waveform */
    /*  Half the amplitude on first sample */
    in_re = in->data[0].re;
    in_im = in->data[0].im;
    if (in_im == 0.0) {
      out->data[0].re = in_re / 2.0;
      out->data[0].im = 0.0;
    } else if (in_re == 0.0) {
      out->data[0].re = 0.0;
      out->data[0].im = in_im / 2.0;
    } else {
      out->data[0].re = in_re / 2.0;
      out->data[0].im = in_im / 2.0;
    }

    /*  OFDM symbol boundary samples */
    i9 = in->size[0] - 160;
    loop_ub = div_nde_s32_floor(i9, 80);
    idx_size_idx_1 = loop_ub + 1;
    for (i9 = 0; i9 <= loop_ub; i9++) {
      idx_data[i9] = 80 + 80 * i9;
    }

    /*  Perform widnowing on preamble & data fields */
    for (i9 = 0; i9 < idx_size_idx_1; i9++) {
      tmp_data[i9] = out->data[idx_data[i9]];
    }

    for (i9 = 0; i9 < idx_size_idx_1; i9++) {
      b_tmp_data[i9] = out->data[idx_data[i9] - 64];
    }

    if (0 <= idx_size_idx_1 - 1) {
      memcpy(&c_tmp_data[0], &idx_data[0], (unsigned int)(idx_size_idx_1 * (int)
              sizeof(int)));
    }

    for (i9 = 0; i9 < idx_size_idx_1; i9++) {
      in_re = tmp_data[i9].re + b_tmp_data[i9].re;
      in_im = tmp_data[i9].im + b_tmp_data[i9].im;
      if (in_im == 0.0) {
        out->data[c_tmp_data[i9]].re = in_re / 2.0;
        out->data[c_tmp_data[i9]].im = 0.0;
      } else if (in_re == 0.0) {
        out->data[c_tmp_data[i9]].re = 0.0;
        out->data[c_tmp_data[i9]].im = in_im / 2.0;
      } else {
        out->data[c_tmp_data[i9]].re = in_re / 2.0;
        out->data[c_tmp_data[i9]].im = in_im / 2.0;
      }
    }
  }
}

/* End of code generation (apply_time_window.cpp) */
