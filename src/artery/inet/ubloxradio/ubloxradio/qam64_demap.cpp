/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qam64_demap.cpp
 *
 * Code generation for function 'qam64_demap'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "qam64_demap.h"
#include "abs.h"
#include "sign.h"

/* Function Definitions */
void qam64_demap(const double sym_in[48], double llr_out[144])
{
  double idx_1_tmp[48];
  int trueCount;
  int i;
  int partialTrueCount;
  boolean_T b1;
  int b_trueCount;
  boolean_T idx_1[48];
  boolean_T idx_2[48];
  boolean_T idx_3[48];
  boolean_T idx_4[48];
  int b_partialTrueCount;
  signed char tmp_data[48];
  int tmp_size[2];
  signed char b_tmp_data[48];
  signed char c_tmp_data[48];
  double d_tmp_data[48];
  signed char e_tmp_data[48];
  signed char f_tmp_data[48];
  signed char g_tmp_data[48];
  signed char h_tmp_data[48];
  signed char i_tmp_data[48];
  signed char j_tmp_data[48];
  signed char k_tmp_data[48];
  signed char l_tmp_data[48];
  int sym_in_size[2];
  signed char m_tmp_data[48];
  signed char n_tmp_data[48];
  double sym_in_data[48];
  int b_sym_in_size[2];
  signed char o_tmp_data[48];
  signed char p_tmp_data[48];
  int c_sym_in_size[2];
  signed char q_tmp_data[48];
  signed char r_tmp_data[48];
  int d_sym_in_size[2];
  signed char s_tmp_data[48];
  signed char t_tmp_data[48];
  int e_sym_in_size[2];
  signed char u_tmp_data[48];
  signed char v_tmp_data[48];

  /* QAM64_DEMAP Performs demapping of QAM-64 modulation */
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
  memset(&llr_out[0], 0, 144U * sizeof(double));

  /*  Find regions of input symbols */
  c_abs(sym_in, idx_1_tmp);

  /*  Calculate LLRs (1st soft-bit) using a different equation for each region */
  trueCount = 0;
  for (i = 0; i < 48; i++) {
    b1 = (idx_1_tmp[i] > 6.0);
    idx_1[i] = b1;
    idx_2[i] = ((idx_1_tmp[i] <= 6.0) && (idx_1_tmp[i] > 4.0));
    idx_3[i] = ((idx_1_tmp[i] <= 4.0) && (idx_1_tmp[i] > 2.0));
    idx_4[i] = (idx_1_tmp[i] <= 2.0);
    if (b1) {
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
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_1[i]) {
      b_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_1[i]) {
      c_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  tmp_size[0] = 1;
  tmp_size[1] = b_trueCount;
  for (partialTrueCount = 0; partialTrueCount < b_trueCount; partialTrueCount++)
  {
    d_tmp_data[partialTrueCount] = sym_in[b_tmp_data[partialTrueCount] - 1];
  }

  b_sign(d_tmp_data, tmp_size);
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    llr_out[3 * (c_tmp_data[partialTrueCount] - 1)] = 16.0 *
      sym_in[tmp_data[partialTrueCount] - 1] - d_tmp_data[partialTrueCount] *
      48.0;
  }

  trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      e_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_2[i]) {
      b_trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      f_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_2[i]) {
      g_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  tmp_size[0] = 1;
  tmp_size[1] = b_trueCount;
  for (partialTrueCount = 0; partialTrueCount < b_trueCount; partialTrueCount++)
  {
    d_tmp_data[partialTrueCount] = sym_in[f_tmp_data[partialTrueCount] - 1];
  }

  b_sign(d_tmp_data, tmp_size);
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    llr_out[3 * (g_tmp_data[partialTrueCount] - 1)] = 12.0 *
      sym_in[e_tmp_data[partialTrueCount] - 1] - d_tmp_data[partialTrueCount] *
      24.0;
  }

  trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_3[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_3[i]) {
      h_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_3[i]) {
      b_trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_3[i]) {
      i_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_3[i]) {
      j_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  tmp_size[0] = 1;
  tmp_size[1] = b_trueCount;
  for (partialTrueCount = 0; partialTrueCount < b_trueCount; partialTrueCount++)
  {
    d_tmp_data[partialTrueCount] = sym_in[i_tmp_data[partialTrueCount] - 1];
  }

  b_sign(d_tmp_data, tmp_size);
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    llr_out[3 * (j_tmp_data[partialTrueCount] - 1)] = 8.0 *
      sym_in[h_tmp_data[partialTrueCount] - 1] - d_tmp_data[partialTrueCount] *
      8.0;
  }

  trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_4[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_4[i]) {
      k_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_4[i]) {
      l_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    llr_out[3 * (l_tmp_data[partialTrueCount] - 1)] = 4.0 *
      sym_in[k_tmp_data[partialTrueCount] - 1];
  }

  /*  Find regions of input symbols */
  /*  Calculate LLRs (2nd soft-bit) using a different equation for each region */
  trueCount = 0;
  for (i = 0; i < 48; i++) {
    idx_2[i] = ((idx_1_tmp[i] <= 6.0) && (idx_1_tmp[i] > 2.0));
    idx_3[i] = (idx_1_tmp[i] <= 2.0);
    if (idx_1[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_1[i]) {
      m_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_1[i]) {
      n_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  sym_in_size[0] = 1;
  sym_in_size[1] = trueCount;
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    sym_in_data[partialTrueCount] = sym_in[m_tmp_data[partialTrueCount] - 1];
  }

  d_abs(sym_in_data, sym_in_size, d_tmp_data, tmp_size);
  i = tmp_size[1];
  for (partialTrueCount = 0; partialTrueCount < i; partialTrueCount++) {
    llr_out[1 + 3 * (n_tmp_data[partialTrueCount] - 1)] = -8.0 *
      d_tmp_data[partialTrueCount] + 40.0;
  }

  trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      o_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_2[i]) {
      p_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  b_sym_in_size[0] = 1;
  b_sym_in_size[1] = trueCount;
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    sym_in_data[partialTrueCount] = sym_in[o_tmp_data[partialTrueCount] - 1];
  }

  d_abs(sym_in_data, b_sym_in_size, d_tmp_data, tmp_size);
  i = tmp_size[1];
  for (partialTrueCount = 0; partialTrueCount < i; partialTrueCount++) {
    llr_out[1 + 3 * (p_tmp_data[partialTrueCount] - 1)] = -4.0 *
      d_tmp_data[partialTrueCount] + 16.0;
  }

  trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_3[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_3[i]) {
      q_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_3[i]) {
      r_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  c_sym_in_size[0] = 1;
  c_sym_in_size[1] = trueCount;
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    sym_in_data[partialTrueCount] = sym_in[q_tmp_data[partialTrueCount] - 1];
  }

  d_abs(sym_in_data, c_sym_in_size, d_tmp_data, tmp_size);
  i = tmp_size[1];
  for (partialTrueCount = 0; partialTrueCount < i; partialTrueCount++) {
    llr_out[1 + 3 * (r_tmp_data[partialTrueCount] - 1)] = -8.0 *
      d_tmp_data[partialTrueCount] + 24.0;
  }

  /*  Find regions of input symbols */
  /*  Calculate LLRs (3rd soft-bit) using a different equation for each region */
  trueCount = 0;
  for (i = 0; i < 48; i++) {
    b1 = (idx_1_tmp[i] > 4.0);
    idx_1[i] = b1;
    idx_2[i] = (idx_1_tmp[i] <= 4.0);
    if (b1) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_1[i]) {
      s_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_1[i]) {
      t_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  d_sym_in_size[0] = 1;
  d_sym_in_size[1] = trueCount;
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    sym_in_data[partialTrueCount] = sym_in[s_tmp_data[partialTrueCount] - 1];
  }

  d_abs(sym_in_data, d_sym_in_size, d_tmp_data, tmp_size);
  i = tmp_size[1];
  for (partialTrueCount = 0; partialTrueCount < i; partialTrueCount++) {
    llr_out[2 + 3 * (t_tmp_data[partialTrueCount] - 1)] = -4.0 *
      d_tmp_data[partialTrueCount] + 24.0;
  }

  trueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  b_partialTrueCount = 0;
  for (i = 0; i < 48; i++) {
    if (idx_2[i]) {
      u_tmp_data[partialTrueCount] = (signed char)(i + 1);
      partialTrueCount++;
    }

    if (idx_2[i]) {
      v_tmp_data[b_partialTrueCount] = (signed char)(i + 1);
      b_partialTrueCount++;
    }
  }

  e_sym_in_size[0] = 1;
  e_sym_in_size[1] = trueCount;
  for (partialTrueCount = 0; partialTrueCount < trueCount; partialTrueCount++) {
    sym_in_data[partialTrueCount] = sym_in[u_tmp_data[partialTrueCount] - 1];
  }

  d_abs(sym_in_data, e_sym_in_size, d_tmp_data, tmp_size);
  i = tmp_size[1];
  for (partialTrueCount = 0; partialTrueCount < i; partialTrueCount++) {
    llr_out[2 + 3 * (v_tmp_data[partialTrueCount] - 1)] = 4.0 *
      d_tmp_data[partialTrueCount] - 8.0;
  }
}

/* End of code generation (qam64_demap.cpp) */
