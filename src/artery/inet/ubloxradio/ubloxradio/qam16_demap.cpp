/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qam16_demap.cpp
 *
 * Code generation for function 'qam16_demap'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "qam16_demap.h"

/* Function Definitions */
void qam16_demap(const double sym_in[48], double llr_out[96])
{
  int trueCount;
  int k;
  int partialTrueCount;
  double x;
  int b_trueCount;
  double idx_1_tmp[48];
  int i;
  boolean_T b0;
  boolean_T idx_1[48];
  boolean_T idx_2[48];
  signed char tmp_data[48];
  signed char b_tmp_data[48];
  double x_data[48];
  signed char c_tmp_data[48];
  signed char d_tmp_data[48];
  signed char e_tmp_data[48];

  /* QAM16_DEMAP Performs demapping of QAM-16 modulation */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    August 2018; Last revision: 30-August-2018 */
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
  /*  Initialize output LLR vector */
  memset(&llr_out[0], 0, 96U * sizeof(double));

  /*  Find regions of input symbols */
  /*  Calculate LLRs (1st soft-bit) using a different equation for each region */
  trueCount = 0;
  for (k = 0; k < 48; k++) {
    x = std::abs(sym_in[k]);
    idx_1_tmp[k] = x;
    b0 = (x > 2.0);
    idx_1[k] = b0;
    idx_2[k] = (x <= 2.0);
    if (b0) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_1[i]) {
      tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_1[i]) {
      b_trueCount++;
    }
  }

  partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_1[i]) {
      b_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }
  }

  for (k = 0; k < b_trueCount; k++) {
    x_data[k] = sym_in[b_tmp_data[k] - 1];
  }

  for (k = 0; k < b_trueCount; k++) {
    x = x_data[k];
    if (x_data[k] < 0.0) {
      x = -1.0;
    } else if (x_data[k] > 0.0) {
      x = 1.0;
    } else {
      if (x_data[k] == 0.0) {
        x = 0.0;
      }
    }

    x_data[k] = x;
  }

  partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_1[i]) {
      c_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }
  }

  for (k = 0; k < trueCount; k++) {
    llr_out[(c_tmp_data[k] - 1) << 1] = 8.0 * sym_in[tmp_data[k] - 1] - x_data[k]
      * 8.0;
  }

  trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  k = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      d_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_2[i]) {
      e_tmp_data[k] = (signed char)(i + 1);
      k++;
    }
  }

  for (k = 0; k < trueCount; k++) {
    llr_out[(e_tmp_data[k] - 1) << 1] = 4.0 * sym_in[d_tmp_data[k] - 1];
  }

  /*  Calculate LLRs (2nd soft-bit) */
  for (k = 0; k < 48; k++) {
    llr_out[1 + (k << 1)] = -4.0 * idx_1_tmp[k] + 8.0;
  }
}

/* End of code generation (qam16_demap.cpp) */
