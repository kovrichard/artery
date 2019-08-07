/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * data_tx.cpp
 *
 * Code generation for function 'data_tx'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "data_tx.h"
#include "mod1.h"
#include "sim_rx_mex_emxutil.h"
#include "fft.h"
#include "bi2de.h"
#include "floor1.h"
#include "mod.h"
#include "scrambler_tx.h"
#include "kaiser.h"
#include "mpower.h"
#include "flipud.h"
#include "step.h"
#include "sim_rx_mex_rtwutil.h"
#include "sim_rx_mex_data.h"

/* Variable Definitions */
static comm_ConvolutionalEncoder_0 b_bcc_obj;
static boolean_T b_bcc_obj_not_empty;

/* Function Definitions */
void data_tx(double PHY_length, const boolean_T PHY_pn_seq[7], const double
             PHY_pilot_idx[4], const double PHY_pilot_val[4], const double
             PHY_polarity_sign[127], const double PHY_data_idx[48], double
             PHY_r_num, double PHY_n_bpscs, double PHY_n_cbps, double PHY_n_dbps,
             double PHY_n_sym, double pad_len, const emxArray_boolean_T
             *padding_out, double w_beta, emxArray_creal_T *data_wf,
             emxArray_creal_T *data_f_mtx)
{
  int i;
  emxArray_boolean_T *padding_vec;
  boolean_T pn_state[7];
  int i5;
  int loop_ub;
  int i6;
  emxArray_boolean_T *scrambler_out;
  emxArray_real_T *y;
  emxArray_boolean_T *b_padding_out;
  double a;
  int i_sym;
  double s;
  double idx0;
  int b_loop_ub;
  double idx1;
  double b;
  double dv8[64];
  int bcc_out_size_idx_0;
  int scrambler_out_size_idx_0;
  boolean_T scrambler_out_data[216];
  unsigned int tmp;
  boolean_T bcc_out_data[432];
  boolean_T b_bcc_out_data[324];
  boolean_T c_bcc_out_data[288];
  double interlvr_out_data[288];
  double ii_data[288];
  int ii_size[2];
  int tmp_size[2];
  double tmp_data[288];
  int b_tmp_size[2];
  double b_tmp_data[288];
  int b_ii_size[2];
  double b_ii_data[288];
  short c_tmp_data[288];
  creal_T data_f[64];
  creal_T mod_table_data[64];
  int interlvr_out_size[2];
  unsigned long u0;
  double qbits_data[64];
  double b_interlvr_out_data[1728];
  int b_interlvr_out_size[1];
  double d_tmp_data[64];
  static const double w[4] = { -1.0, -0.33333333333333331, 1.0,
    0.33333333333333331 };

  static const double b_w[8] = { -1.0, -0.71428571428571419,
    -0.14285714285714285, -0.42857142857142855, 1.0, 0.71428571428571419,
    0.14285714285714285, 0.42857142857142855 };

  double b_data[64];
  creal_T b_b_data[288];
  creal_T data_fs[64];
  creal_T b_data_fs[64];

  /* DATA_TX Transmitter processing of all DATA OFDM symbols */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    August 2018; Last revision: 19-February-2019 */
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
  /*  Store this as a persistent variable to avoid reinitialization */
  /*  Needed for code generation */
  /*  Create or reset system object */
  if (!b_bcc_obj_not_empty) {
    Constructor(&b_bcc_obj);
    b_bcc_obj_not_empty = true;
  } else {
    if (b_bcc_obj.S0_isInitialized == 1) {
      InitializeConditions(&b_bcc_obj);
    }
  }

  /*  Initialize state of PN generator */
  for (i = 0; i < 7; i++) {
    pn_state[i] = PHY_pn_seq[i];
  }

  emxInit_boolean_T(&padding_vec, 1);
  flipud(pn_state);

  /*  Mark the tail bits so that they can be nulled after scrambling */
  i5 = padding_vec->size[0];
  loop_ub = (int)(16.0 + PHY_length * 8.0);
  i = (int)pad_len;
  padding_vec->size[0] = (loop_ub + i) + 6;
  emxEnsureCapacity_boolean_T(padding_vec, i5);
  for (i5 = 0; i5 < loop_ub; i5++) {
    padding_vec->data[i5] = true;
  }

  for (i5 = 0; i5 < 6; i5++) {
    padding_vec->data[i5 + loop_ub] = false;
  }

  for (i5 = 0; i5 < i; i5++) {
    padding_vec->data[(i5 + loop_ub) + 6] = true;
  }

  /*  Initialize time-domain waveform output */
  i5 = data_wf->size[0];
  loop_ub = (int)(PHY_n_sym * 80.0);
  data_wf->size[0] = loop_ub;
  emxEnsureCapacity_creal_T(data_wf, i5);
  for (i5 = 0; i5 < loop_ub; i5++) {
    data_wf->data[i5].re = 0.0;
    data_wf->data[i5].im = 0.0;
  }

  /*  Initialize frequency-domain output */
  i5 = data_f_mtx->size[0] * data_f_mtx->size[1];
  data_f_mtx->size[0] = 64;
  i6 = (int)PHY_n_sym;
  data_f_mtx->size[1] = i6;
  emxEnsureCapacity_creal_T(data_f_mtx, i5);
  loop_ub = i6 << 6;
  for (i5 = 0; i5 < loop_ub; i5++) {
    data_f_mtx->data[i5].re = 0.0;
    data_f_mtx->data[i5].im = 0.0;
  }

  /*  Loop for each OFDM symbol */
  i5 = (int)((PHY_n_sym - 1.0) + 1.0);
  emxInit_boolean_T(&scrambler_out, 1);
  emxInit_real_T(&y, 2);
  emxInit_boolean_T(&b_padding_out, 1);
  if (0 <= i5 - 1) {
    a = PHY_n_cbps / 16.0;
    s = PHY_n_bpscs / 2.0;
    if (!(s > 1.0)) {
      s = 1.0;
    }

    b_loop_ub = (int)PHY_n_cbps;
    b = mpower(2.0, PHY_n_bpscs) - 1.0;
    kaiser(w_beta, dv8);
  }

  for (i_sym = 0; i_sym < i5; i_sym++) {
    /*  Index of bits into scrambler per OFDM symbol */
    idx0 = (double)i_sym * PHY_n_dbps + 1.0;
    idx1 = ((double)i_sym + 1.0) * PHY_n_dbps;

    /*  Perform scrambling with given PN sequence */
    if (idx0 > idx1) {
      i6 = 0;
      bcc_out_size_idx_0 = 0;
    } else {
      i6 = (int)idx0 - 1;
      bcc_out_size_idx_0 = (int)idx1;
    }

    i = b_padding_out->size[0];
    loop_ub = bcc_out_size_idx_0 - i6;
    b_padding_out->size[0] = loop_ub;
    emxEnsureCapacity_boolean_T(b_padding_out, i);
    for (bcc_out_size_idx_0 = 0; bcc_out_size_idx_0 < loop_ub;
         bcc_out_size_idx_0++) {
      b_padding_out->data[bcc_out_size_idx_0] = padding_out->data[i6 +
        bcc_out_size_idx_0];
    }

    scrambler_tx(b_padding_out, pn_state, scrambler_out);

    /*  Set scrambled tail bits to zero */
    if (idx0 > idx1) {
      i6 = 0;
    } else {
      i6 = (int)idx0 - 1;
    }

    scrambler_out_size_idx_0 = scrambler_out->size[0];
    loop_ub = scrambler_out->size[0];
    for (bcc_out_size_idx_0 = 0; bcc_out_size_idx_0 < loop_ub;
         bcc_out_size_idx_0++) {
      scrambler_out_data[bcc_out_size_idx_0] = (scrambler_out->
        data[bcc_out_size_idx_0] && padding_vec->data[i6 + bcc_out_size_idx_0]);
    }

    /*  Process data through BCC encoder */
    if (b_bcc_obj.S0_isInitialized != 1) {
      b_bcc_obj.S0_isInitialized = 1;

      /* System object Initialization function: comm.ConvolutionalEncoder */
      b_bcc_obj.W0_currState = 0U;
    }

    /* System object Outputs function: comm.ConvolutionalEncoder */
    bcc_out_size_idx_0 = scrambler_out_size_idx_0 << 1;
    for (loop_ub = 0; loop_ub < scrambler_out_size_idx_0; loop_ub++) {
      i = (int)(b_bcc_obj.W0_currState + (scrambler_out_data[loop_ub] << 6));
      tmp = b_bcc_obj.P1_OutputVec[i];
      b_bcc_obj.W0_currState = b_bcc_obj.P0_StateVec[i];
      i = loop_ub << 1;
      bcc_out_data[i + 1] = ((tmp & 1U) != 0U);
      tmp >>= 1U;
      bcc_out_data[i] = ((tmp & 1U) != 0U);
    }

    /*  Perform puncturing if needed */
    switch ((int)PHY_r_num) {
     case 2:
      loop_ub = (signed char)(bcc_out_size_idx_0 / 4);
      for (i6 = 0; i6 < loop_ub; i6++) {
        i = i6 << 2;
        b_bcc_out_data[3 * i6] = bcc_out_data[i];
        b_bcc_out_data[1 + 3 * i6] = bcc_out_data[1 + i];
        b_bcc_out_data[2 + 3 * i6] = bcc_out_data[2 + i];
      }

      loop_ub = 3 * (bcc_out_size_idx_0 / 4);
      if (0 <= loop_ub - 1) {
        memcpy(&bcc_out_data[0], &b_bcc_out_data[0], (unsigned int)(loop_ub *
                (int)sizeof(boolean_T)));
      }
      break;

     case 3:
      loop_ub = (signed char)(bcc_out_size_idx_0 / 6);
      for (i6 = 0; i6 < loop_ub; i6++) {
        i = i6 << 2;
        c_bcc_out_data[i] = bcc_out_data[6 * i6];
        c_bcc_out_data[1 + i] = bcc_out_data[1 + 6 * i6];
        c_bcc_out_data[2 + i] = bcc_out_data[2 + 6 * i6];
        c_bcc_out_data[3 + i] = bcc_out_data[5 + 6 * i6];
      }

      loop_ub = (bcc_out_size_idx_0 / 6) << 2;
      if (0 <= loop_ub - 1) {
        memcpy(&bcc_out_data[0], &c_bcc_out_data[0], (unsigned int)(loop_ub *
                (int)sizeof(boolean_T)));
      }
      break;
    }

    /*  Apply interleaving per OFDM symbol */
    /* INTERLEAVER Bit interleaver */
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
    /*  Interleaver configuration */
    /*  s-parameter */
    /*  Initialize output */
    if (0 <= b_loop_ub - 1) {
      memset(&interlvr_out_data[0], 0, (unsigned int)(b_loop_ub * (int)sizeof
              (double)));
    }

    /*  Input index */
    if (rtIsNaN(PHY_n_cbps - 1.0)) {
      i6 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i6);
      y->data[0] = rtNaN;
    } else if (PHY_n_cbps - 1.0 < 0.0) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else if (rtIsInf(PHY_n_cbps - 1.0) && (0.0 == PHY_n_cbps - 1.0)) {
      i6 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i6);
      y->data[0] = rtNaN;
    } else {
      i6 = y->size[0] * y->size[1];
      y->size[0] = 1;
      loop_ub = (int)std::floor(PHY_n_cbps - 1.0);
      y->size[1] = loop_ub + 1;
      emxEnsureCapacity_real_T(y, i6);
      for (i6 = 0; i6 <= loop_ub; i6++) {
        y->data[i6] = i6;
      }
    }

    /*  First permutation */
    d_mod(y->data, y->size, ii_data, ii_size);
    tmp_size[0] = 1;
    tmp_size[1] = y->size[1];
    loop_ub = y->size[0] * y->size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      tmp_data[i6] = y->data[i6] / 16.0;
    }

    b_floor(tmp_data, tmp_size);
    loop_ub = ii_size[0] * ii_size[1] - 1;
    for (i6 = 0; i6 <= loop_ub; i6++) {
      ii_data[i6] = a * ii_data[i6] + tmp_data[i6];
    }

    /*  Second permutation */
    /*  Interleaver mapping */
    tmp_size[0] = 1;
    tmp_size[1] = ii_size[1];
    loop_ub = ii_size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      tmp_data[i6] = ii_data[i6] / s;
    }

    b_floor(tmp_data, tmp_size);
    b_tmp_size[0] = 1;
    b_tmp_size[1] = ii_size[1];
    loop_ub = ii_size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_tmp_data[i6] = 16.0 * ii_data[i6] / PHY_n_cbps;
    }

    b_floor(b_tmp_data, b_tmp_size);
    b_ii_size[0] = 1;
    b_ii_size[1] = ii_size[1];
    loop_ub = ii_size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_ii_data[i6] = (ii_data[i6] + PHY_n_cbps) - b_tmp_data[i6];
    }

    e_mod(b_ii_data, b_ii_size, s, b_tmp_data, b_tmp_size);
    loop_ub = tmp_size[0] * tmp_size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      c_tmp_data[i6] = (short)((s * tmp_data[i6] + b_tmp_data[i6]) + 1.0);
    }

    loop_ub = y->size[0] * y->size[1];
    for (i6 = 0; i6 < loop_ub; i6++) {
      interlvr_out_data[c_tmp_data[i6] - 1] = bcc_out_data[(int)(y->data[i6] +
        1.0) - 1];
    }

    /*  Initialize f-domain data symbol */
    memset(&data_f[0], 0, sizeof(creal_T) << 6);

    /*  Insert pilots with correct polarity */
    idx0 = PHY_polarity_sign[(int)(f_mod((double)i_sym + 1.0, 127.0) + 1.0) - 1];
    i6 = (int)PHY_pilot_idx[0] - 1;
    data_f[i6].re = idx0 * PHY_pilot_val[0];
    data_f[i6].im = 0.0;
    i6 = (int)PHY_pilot_idx[1] - 1;
    data_f[i6].re = idx0 * PHY_pilot_val[1];
    data_f[i6].im = 0.0;
    i6 = (int)PHY_pilot_idx[2] - 1;
    data_f[i6].re = idx0 * PHY_pilot_val[2];
    data_f[i6].im = 0.0;
    i6 = (int)PHY_pilot_idx[3] - 1;
    data_f[i6].re = idx0 * PHY_pilot_val[3];
    data_f[i6].im = 0.0;

    /*  Insert modulated data into f-domain data symbol */
    /* MAPPER_TX Modulation mapper */
    /*  */
    /*    Author: Ioannis Sarris, u-blox */
    /*    email: ioannis.sarris@u-blox.com */
    /*    August 2018; Last revision: 19-February-2019 */
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
    /*  Get all possible decimal values for q */
    if (rtIsNaN(b)) {
      i6 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i6);
      y->data[0] = rtNaN;
    } else if (b < 0.0) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else if (rtIsInf(b) && (0.0 == b)) {
      i6 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i6);
      y->data[0] = rtNaN;
    } else {
      i6 = y->size[0] * y->size[1];
      y->size[0] = 1;
      loop_ub = (int)std::floor(b);
      y->size[1] = loop_ub + 1;
      emxEnsureCapacity_real_T(y, i6);
      for (i6 = 0; i6 <= loop_ub; i6++) {
        y->data[i6] = i6;
      }
    }

    /*  Create modulation table and appropriate normalization factor */
    switch ((int)PHY_n_bpscs) {
     case 1:
      /*  BPSK */
      loop_ub = y->size[1];
      for (i6 = 0; i6 < loop_ub; i6++) {
        mod_table_data[i6].re = (signed char)((signed char)((signed char)((int)
          (y->data[i6] + 1.0) - 1) << 1) - 1);
        mod_table_data[i6].im = 0.0;
      }

      idx0 = 1.0;
      break;

     case 2:
      /*  QPSK */
      loop_ub = y->size[1];
      for (i = 0; i < loop_ub; i++) {
        idx0 = rt_roundd_snf(y->data[i]);
        if (idx0 < 1.8446744073709552E+19) {
          if (idx0 >= 0.0) {
            u0 = (unsigned long)idx0;
          } else {
            u0 = 0UL;
          }
        } else if (idx0 >= 1.8446744073709552E+19) {
          u0 = MAX_uint64_T;
        } else {
          u0 = 0UL;
        }

        qbits_data[i] = (double)(u0 & 1UL);
      }

      tmp_size[0] = 1;
      tmp_size[1] = y->size[1];
      i = y->size[0] * y->size[1];
      for (i6 = 0; i6 < i; i6++) {
        tmp_data[i6] = y->data[i6] / 2.0;
      }

      b_floor(tmp_data, tmp_size);
      i = tmp_size[1];
      for (i6 = 0; i6 < i; i6++) {
        d_tmp_data[i6] = (signed char)((signed char)((signed char)((int)
          (tmp_data[i6] + 1.0) - 1) << 1) - 1);
      }

      for (i6 = 0; i6 < loop_ub; i6++) {
        b_data[i6] = (signed char)((signed char)((signed char)((int)
          (qbits_data[i6] + 1.0) - 1) << 1) - 1);
      }

      loop_ub = tmp_size[1];
      for (i6 = 0; i6 < loop_ub; i6++) {
        mod_table_data[i6].re = d_tmp_data[i6] + 0.0 * b_data[i6];
        mod_table_data[i6].im = b_data[i6];
      }

      idx0 = 0.70710678118654746;
      break;

     case 4:
      /*  16-QAM */
      loop_ub = y->size[1];
      for (i = 0; i < loop_ub; i++) {
        idx0 = rt_roundd_snf(y->data[i]);
        if (idx0 < 1.8446744073709552E+19) {
          if (idx0 >= 0.0) {
            u0 = (unsigned long)idx0;
          } else {
            u0 = 0UL;
          }
        } else if (idx0 >= 1.8446744073709552E+19) {
          u0 = MAX_uint64_T;
        } else {
          u0 = 0UL;
        }

        qbits_data[i] = (double)(u0 & 3UL);
      }

      tmp_size[0] = 1;
      tmp_size[1] = y->size[1];
      i = y->size[0] * y->size[1];
      for (i6 = 0; i6 < i; i6++) {
        tmp_data[i6] = y->data[i6] / 4.0;
      }

      b_floor(tmp_data, tmp_size);
      i = tmp_size[1];
      for (i6 = 0; i6 < i; i6++) {
        d_tmp_data[i6] = w[(int)(tmp_data[i6] + 1.0) - 1];
      }

      for (i6 = 0; i6 < loop_ub; i6++) {
        b_data[i6] = w[(int)(qbits_data[i6] + 1.0) - 1];
      }

      loop_ub = tmp_size[1];
      for (i6 = 0; i6 < loop_ub; i6++) {
        mod_table_data[i6].re = d_tmp_data[i6] + 0.0 * b_data[i6];
        mod_table_data[i6].im = b_data[i6];
      }

      idx0 = 0.94868329805051377;
      break;

     case 6:
      /*  64-QAM */
      loop_ub = y->size[1];
      for (i = 0; i < loop_ub; i++) {
        idx0 = rt_roundd_snf(y->data[i]);
        if (idx0 < 1.8446744073709552E+19) {
          if (idx0 >= 0.0) {
            u0 = (unsigned long)idx0;
          } else {
            u0 = 0UL;
          }
        } else if (idx0 >= 1.8446744073709552E+19) {
          u0 = MAX_uint64_T;
        } else {
          u0 = 0UL;
        }

        qbits_data[i] = (double)(u0 & 7UL);
      }

      tmp_size[0] = 1;
      tmp_size[1] = y->size[1];
      i = y->size[0] * y->size[1];
      for (i6 = 0; i6 < i; i6++) {
        tmp_data[i6] = y->data[i6] / 8.0;
      }

      b_floor(tmp_data, tmp_size);
      i = tmp_size[1];
      for (i6 = 0; i6 < i; i6++) {
        d_tmp_data[i6] = b_w[(int)(tmp_data[i6] + 1.0) - 1];
      }

      for (i6 = 0; i6 < loop_ub; i6++) {
        b_data[i6] = b_w[(int)(qbits_data[i6] + 1.0) - 1];
      }

      loop_ub = tmp_size[1];
      for (i6 = 0; i6 < loop_ub; i6++) {
        mod_table_data[i6].re = d_tmp_data[i6] + 0.0 * b_data[i6];
        mod_table_data[i6].im = b_data[i6];
      }

      idx0 = 1.0801234497346432;
      break;

     default:
      /*  Needed for code-generation */
      idx0 = 1.0;
      break;
    }

    /*  Group bits per q */
    /*  Convert each set of q bits to decimal */
    i = (int)((double)(int)PHY_n_cbps / PHY_n_bpscs);
    interlvr_out_size[0] = i;
    loop_ub = (int)PHY_n_bpscs;
    interlvr_out_size[1] = loop_ub;
    for (i6 = 0; i6 < loop_ub; i6++) {
      for (bcc_out_size_idx_0 = 0; bcc_out_size_idx_0 < i; bcc_out_size_idx_0++)
      {
        b_interlvr_out_data[bcc_out_size_idx_0 + i * i6] = interlvr_out_data[i6
          + loop_ub * bcc_out_size_idx_0];
      }
    }

    bi2de(b_interlvr_out_data, interlvr_out_size, interlvr_out_data,
          b_interlvr_out_size);

    /*  Modulation symbols are obtained by mapping to modulation table */
    loop_ub = b_interlvr_out_size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_b_data[i6] = mod_table_data[(int)(interlvr_out_data[i6] + 1.0) - 1];
    }

    loop_ub = b_interlvr_out_size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_b_data[i6].re *= idx0;
      b_b_data[i6].im *= idx0;
    }

    for (i6 = 0; i6 < 48; i6++) {
      data_f[(int)PHY_data_idx[i6] - 1] = b_b_data[i6];
    }

    /*  Apply spectral shaping window */
    for (i = 0; i < 64; i++) {
      data_fs[i].re = dv8[i] * data_f[i].re;
      data_fs[i].im = dv8[i] * data_f[i].im;
    }

    /*  Perform IFFT & normalize */
    /* DOT11_IFFT 802.11 IFFT */
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
    /*  IFFT with shifting and normalization */
    for (i = 0; i < 64; i++) {
      b_data_fs[i] = data_fs[iv0[i]];
    }

    r2br_r2dit_trig_impl(b_data_fs, dv0, dv1, data_fs);
    for (i6 = 0; i6 < 64; i6++) {
      data_fs[i6].re = 0.13867504905630729 * (64.0 * (0.015625 * data_fs[i6].re));
      data_fs[i6].im = 0.13867504905630729 * (64.0 * (0.015625 * data_fs[i6].im));
    }

    /*  Append CP */
    idx0 = (double)i_sym * 80.0;
    for (i6 = 0; i6 < 16; i6++) {
      data_wf->data[i6 + (int)idx0] = data_fs[48 + i6];
    }

    for (i6 = 0; i6 < 64; i6++) {
      data_wf->data[(i6 + (int)idx0) + 16] = data_fs[i6];
    }

    /*  Store f-domain symbols which are needed for (genie) channel tracking at the Rx */
    for (i6 = 0; i6 < 64; i6++) {
      data_f_mtx->data[i6 + (i_sym << 6)] = data_f[i6];
    }
  }

  emxFree_boolean_T(&b_padding_out);
  emxFree_real_T(&y);
  emxFree_boolean_T(&scrambler_out);
  emxFree_boolean_T(&padding_vec);
}

void data_tx_free()
{
  /* System object Destructor function: comm.ConvolutionalEncoder */
  if (b_bcc_obj.S0_isInitialized == 1) {
    b_bcc_obj.S0_isInitialized = 2;
  }
}

void data_tx_init()
{
  b_bcc_obj_not_empty = false;
  b_bcc_obj.S0_isInitialized = 0;
}

/* End of code generation (data_tx.cpp) */
