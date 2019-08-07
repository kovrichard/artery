/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sig_rx.cpp
 *
 * Code generation for function 'sig_rx'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sig_rx.h"
#include "bi2de.h"
#include "mod.h"
#include "SystemCore.h"
#include "mpower.h"
#include "deinterleaver.h"
#include "kaiser.h"
#include "mean.h"
#include "angle.h"
#include "fft.h"
#include "ViterbiDecoder.h"
#include "sim_rx_mex_rtwutil.h"
#include "sim_rx_mex_data.h"

/* Variable Definitions */
static commcodegen_ViterbiDecoder vit_obj;
static boolean_T vit_obj_not_empty;

/* Function Definitions */
void sig_rx(const creal_T r[64], creal_T h_est[64], double *SIG_CFG_length,
            double *SIG_CFG_mcs, boolean_T *SIG_CFG_sig_err, double
            *SIG_CFG_r_num, double *SIG_CFG_r_denom, double *SIG_CFG_n_bpscs,
            double *SIG_CFG_n_cbps, double *SIG_CFG_n_dbps, double
            *SIG_CFG_n_sym, double *r_cfo)
{
  int i;
  creal_T y[64];
  creal_T h_in[64];
  double pp;
  double b_r;
  double brm;
  double y_re;
  double y_im;
  creal_T b_y[4];
  double dv12[4];
  double snr[48];
  double dv13[48];
  static const signed char iv11[48] = { 6, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 26, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 54, 55, 56, 57, 58 };

  double b_y_re;
  commcodegen_ViterbiDecoder *obj;
  comm_ViterbiDecoder_4 *b_obj;
  unsigned int minstateLastTB;
  int tbworkLastTB;
  int ib;
  signed char sig_msg[24];
  int indx1;
  unsigned int minstate;
  double b_indx1;
  boolean_T guard1 = false;
  static const signed char iv12[8] = { 1, 3, 1, 3, 1, 3, 2, 3 };

  static const signed char iv13[8] = { 2, 4, 2, 4, 2, 4, 3, 4 };

  static const signed char iv14[8] = { 1, 1, 2, 2, 4, 4, 6, 6 };

  /* SIG_RX SIG message receiver/deparser */
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
  /*  Store this as a persistent variable to avoid reinitialization */
  /*  Initialize structure */
  *SIG_CFG_length = 0.0;
  *SIG_CFG_mcs = 0.0;
  *SIG_CFG_sig_err = true;
  *SIG_CFG_r_num = 0.0;
  *SIG_CFG_r_denom = 0.0;
  *SIG_CFG_n_bpscs = 0.0;
  *SIG_CFG_n_cbps = 0.0;
  *SIG_CFG_n_dbps = 0.0;
  *SIG_CFG_n_sym = 0.0;

  /*  Create system object */
  if (!vit_obj_not_empty) {
    ViterbiDecoder_ViterbiDecoder(&vit_obj);
    vit_obj_not_empty = true;
  }

  /*  Perform FFT & normalization */
  /* DOT11_FFT 802.11 FFT */
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
  /*  FFT with shifting and normalization */
  for (i = 0; i < 64; i++) {
    y[i] = r[iv5[i]];
  }

  r2br_r2dit_trig_impl(y, dv0, dv3, h_in);
  for (i = 0; i < 64; i++) {
    pp = h_in[iv0[i]].re * 7.2111025509279782;
    b_r = h_in[iv0[i]].im * 7.2111025509279782;
    if (b_r == 0.0) {
      y[i].re = pp / 64.0;
      y[i].im = 0.0;
    } else if (pp == 0.0) {
      y[i].re = 0.0;
      y[i].im = b_r / 64.0;
    } else {
      y[i].re = pp / 64.0;
      y[i].im = b_r / 64.0;
    }
  }

  /*  Frequency-domain smoothing */
  memcpy(&h_in[0], &h_est[0], sizeof(creal_T) << 6);

  /* FD_SMOOTH Frequency-domain smoothing */
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
  /*  Replicate missing values to enable smoothing */
  for (i = 0; i < 6; i++) {
    h_in[i] = h_est[6];
  }

  pp = h_in[31].re + h_in[33].re;
  b_r = h_in[31].im + h_in[33].im;
  if (b_r == 0.0) {
    h_in[32].re = pp / 2.0;
    h_in[32].im = 0.0;
  } else if (pp == 0.0) {
    h_in[32].re = 0.0;
    h_in[32].im = b_r / 2.0;
  } else {
    h_in[32].re = pp / 2.0;
    h_in[32].im = b_r / 2.0;
  }

  for (i = 0; i < 5; i++) {
    h_in[59 + i] = h_in[58];
  }

  /*  Moving-average smoothing */
  memset(&h_est[0], 0, sizeof(creal_T) << 6);

  /*  Apply length-3 window */
  for (i = 0; i < 62; i++) {
    h_est[1 + i].re = (h_in[i].re * 0.15 + h_in[1 + i].re * 0.7) + h_in[2 + i].
      re * 0.15;
    h_est[1 + i].im = (h_in[i].im * 0.15 + h_in[1 + i].im * 0.7) + h_in[2 + i].
      im * 0.15;
  }

  /*  Pilot equalization */
  /*  Residual CFO estimation */
  if (h_est[11].im == 0.0) {
    if (y[11].im == 0.0) {
      y_re = y[11].re / h_est[11].re;
      y_im = 0.0;
    } else if (y[11].re == 0.0) {
      y_re = 0.0;
      y_im = y[11].im / h_est[11].re;
    } else {
      y_re = y[11].re / h_est[11].re;
      y_im = y[11].im / h_est[11].re;
    }
  } else if (h_est[11].re == 0.0) {
    if (y[11].re == 0.0) {
      y_re = y[11].im / h_est[11].im;
      y_im = 0.0;
    } else if (y[11].im == 0.0) {
      y_re = 0.0;
      y_im = -(y[11].re / h_est[11].im);
    } else {
      y_re = y[11].im / h_est[11].im;
      y_im = -(y[11].re / h_est[11].im);
    }
  } else {
    brm = std::abs(h_est[11].re);
    pp = std::abs(h_est[11].im);
    if (brm > pp) {
      pp = h_est[11].im / h_est[11].re;
      b_r = h_est[11].re + pp * h_est[11].im;
      y_re = (y[11].re + pp * y[11].im) / b_r;
      y_im = (y[11].im - pp * y[11].re) / b_r;
    } else if (pp == brm) {
      if (h_est[11].re > 0.0) {
        pp = 0.5;
      } else {
        pp = -0.5;
      }

      if (h_est[11].im > 0.0) {
        b_r = 0.5;
      } else {
        b_r = -0.5;
      }

      y_re = (y[11].re * pp + y[11].im * b_r) / brm;
      y_im = (y[11].im * pp - y[11].re * b_r) / brm;
    } else {
      pp = h_est[11].re / h_est[11].im;
      b_r = h_est[11].im + pp * h_est[11].re;
      y_re = (pp * y[11].re + y[11].im) / b_r;
      y_im = (pp * y[11].im - y[11].re) / b_r;
    }
  }

  b_y[0].re = y_re;
  b_y[0].im = y_im;
  if (h_est[25].im == 0.0) {
    if (y[25].im == 0.0) {
      y_re = y[25].re / h_est[25].re;
      y_im = 0.0;
    } else if (y[25].re == 0.0) {
      y_re = 0.0;
      y_im = y[25].im / h_est[25].re;
    } else {
      y_re = y[25].re / h_est[25].re;
      y_im = y[25].im / h_est[25].re;
    }
  } else if (h_est[25].re == 0.0) {
    if (y[25].re == 0.0) {
      y_re = y[25].im / h_est[25].im;
      y_im = 0.0;
    } else if (y[25].im == 0.0) {
      y_re = 0.0;
      y_im = -(y[25].re / h_est[25].im);
    } else {
      y_re = y[25].im / h_est[25].im;
      y_im = -(y[25].re / h_est[25].im);
    }
  } else {
    brm = std::abs(h_est[25].re);
    pp = std::abs(h_est[25].im);
    if (brm > pp) {
      pp = h_est[25].im / h_est[25].re;
      b_r = h_est[25].re + pp * h_est[25].im;
      y_re = (y[25].re + pp * y[25].im) / b_r;
      y_im = (y[25].im - pp * y[25].re) / b_r;
    } else if (pp == brm) {
      if (h_est[25].re > 0.0) {
        pp = 0.5;
      } else {
        pp = -0.5;
      }

      if (h_est[25].im > 0.0) {
        b_r = 0.5;
      } else {
        b_r = -0.5;
      }

      y_re = (y[25].re * pp + y[25].im * b_r) / brm;
      y_im = (y[25].im * pp - y[25].re * b_r) / brm;
    } else {
      pp = h_est[25].re / h_est[25].im;
      b_r = h_est[25].im + pp * h_est[25].re;
      y_re = (pp * y[25].re + y[25].im) / b_r;
      y_im = (pp * y[25].im - y[25].re) / b_r;
    }
  }

  b_y[1].re = y_re;
  b_y[1].im = y_im;
  if (h_est[39].im == 0.0) {
    if (y[39].im == 0.0) {
      y_re = y[39].re / h_est[39].re;
      y_im = 0.0;
    } else if (y[39].re == 0.0) {
      y_re = 0.0;
      y_im = y[39].im / h_est[39].re;
    } else {
      y_re = y[39].re / h_est[39].re;
      y_im = y[39].im / h_est[39].re;
    }
  } else if (h_est[39].re == 0.0) {
    if (y[39].re == 0.0) {
      y_re = y[39].im / h_est[39].im;
      y_im = 0.0;
    } else if (y[39].im == 0.0) {
      y_re = 0.0;
      y_im = -(y[39].re / h_est[39].im);
    } else {
      y_re = y[39].im / h_est[39].im;
      y_im = -(y[39].re / h_est[39].im);
    }
  } else {
    brm = std::abs(h_est[39].re);
    pp = std::abs(h_est[39].im);
    if (brm > pp) {
      pp = h_est[39].im / h_est[39].re;
      b_r = h_est[39].re + pp * h_est[39].im;
      y_re = (y[39].re + pp * y[39].im) / b_r;
      y_im = (y[39].im - pp * y[39].re) / b_r;
    } else if (pp == brm) {
      if (h_est[39].re > 0.0) {
        pp = 0.5;
      } else {
        pp = -0.5;
      }

      if (h_est[39].im > 0.0) {
        b_r = 0.5;
      } else {
        b_r = -0.5;
      }

      y_re = (y[39].re * pp + y[39].im * b_r) / brm;
      y_im = (y[39].im * pp - y[39].re * b_r) / brm;
    } else {
      pp = h_est[39].re / h_est[39].im;
      b_r = h_est[39].im + pp * h_est[39].re;
      y_re = (pp * y[39].re + y[39].im) / b_r;
      y_im = (pp * y[39].im - y[39].re) / b_r;
    }
  }

  b_y[2].re = y_re;
  b_y[2].im = y_im;
  if (h_est[53].im == 0.0) {
    if (y[53].im == 0.0) {
      y_re = y[53].re / h_est[53].re;
      y_im = 0.0;
    } else if (y[53].re == 0.0) {
      y_re = 0.0;
      y_im = y[53].im / h_est[53].re;
    } else {
      y_re = y[53].re / h_est[53].re;
      y_im = y[53].im / h_est[53].re;
    }
  } else if (h_est[53].re == 0.0) {
    if (y[53].re == 0.0) {
      y_re = y[53].im / h_est[53].im;
      y_im = 0.0;
    } else if (y[53].im == 0.0) {
      y_re = 0.0;
      y_im = -(y[53].re / h_est[53].im);
    } else {
      y_re = y[53].im / h_est[53].im;
      y_im = -(y[53].re / h_est[53].im);
    }
  } else {
    brm = std::abs(h_est[53].re);
    pp = std::abs(h_est[53].im);
    if (brm > pp) {
      pp = h_est[53].im / h_est[53].re;
      b_r = h_est[53].re + pp * h_est[53].im;
      y_re = (y[53].re + pp * y[53].im) / b_r;
      y_im = (y[53].im - pp * y[53].re) / b_r;
    } else if (pp == brm) {
      if (h_est[53].re > 0.0) {
        pp = 0.5;
      } else {
        pp = -0.5;
      }

      if (h_est[53].im > 0.0) {
        b_r = 0.5;
      } else {
        b_r = -0.5;
      }

      y_re = (y[53].re * pp + y[53].im * b_r) / brm;
      y_im = (y[53].im * pp - y[53].re * b_r) / brm;
    } else {
      pp = h_est[53].re / h_est[53].im;
      b_r = h_est[53].im + pp * h_est[53].re;
      y_re = (pp * y[53].re + y[53].im) / b_r;
      y_im = (pp * y[53].im - y[53].re) / b_r;
    }
  }

  b_y[3].re = -y_re;
  b_y[3].im = -y_im;
  angle(b_y, dv12);
  *r_cfo = mean(dv12);

  /*  Data equalization with CFO compensation */
  y_re = *r_cfo * 0.0;
  if (-*r_cfo == 0.0) {
    y_re = std::exp(y_re);
    y_im = 0.0;
  } else {
    b_r = std::exp(y_re / 2.0);
    y_re = b_r * (b_r * std::cos(-*r_cfo));
    y_im = b_r * (b_r * std::sin(-*r_cfo));
  }

  /*  SNR input to Viterbi */
  /*  LLR demapping */
  /* LLR_DEMAP LLR demapping */
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
  /*  Initialize llr_out matrix */
  /*  BPSK */
  /*  Negative LLR values */
  /*  Deinterleaving */
  for (i = 0; i < 48; i++) {
    if (h_est[iv11[i]].im == 0.0) {
      if (y[iv11[i]].im == 0.0) {
        b_y_re = y[iv11[i]].re / h_est[iv11[i]].re;
        pp = 0.0;
      } else if (y[iv11[i]].re == 0.0) {
        b_y_re = 0.0;
        pp = y[iv11[i]].im / h_est[iv11[i]].re;
      } else {
        b_y_re = y[iv11[i]].re / h_est[iv11[i]].re;
        pp = y[iv11[i]].im / h_est[iv11[i]].re;
      }
    } else if (h_est[iv11[i]].re == 0.0) {
      if (y[iv11[i]].re == 0.0) {
        b_y_re = y[iv11[i]].im / h_est[iv11[i]].im;
        pp = 0.0;
      } else if (y[iv11[i]].im == 0.0) {
        b_y_re = 0.0;
        pp = -(y[iv11[i]].re / h_est[iv11[i]].im);
      } else {
        b_y_re = y[iv11[i]].im / h_est[iv11[i]].im;
        pp = -(y[iv11[i]].re / h_est[iv11[i]].im);
      }
    } else {
      brm = std::abs(h_est[iv11[i]].re);
      pp = std::abs(h_est[iv11[i]].im);
      if (brm > pp) {
        pp = h_est[iv11[i]].im / h_est[iv11[i]].re;
        b_r = h_est[iv11[i]].re + pp * h_est[iv11[i]].im;
        b_y_re = (y[iv11[i]].re + pp * y[iv11[i]].im) / b_r;
        pp = (y[iv11[i]].im - pp * y[iv11[i]].re) / b_r;
      } else if (pp == brm) {
        if (h_est[iv11[i]].re > 0.0) {
          pp = 0.5;
        } else {
          pp = -0.5;
        }

        if (h_est[iv11[i]].im > 0.0) {
          b_r = 0.5;
        } else {
          b_r = -0.5;
        }

        b_y_re = (y[iv11[i]].re * pp + y[iv11[i]].im * b_r) / brm;
        pp = (y[iv11[i]].im * pp - y[iv11[i]].re * b_r) / brm;
      } else {
        pp = h_est[iv11[i]].re / h_est[iv11[i]].im;
        b_r = h_est[iv11[i]].im + pp * h_est[iv11[i]].re;
        b_y_re = (pp * y[iv11[i]].re + y[iv11[i]].im) / b_r;
        pp = (pp * y[iv11[i]].im - y[iv11[i]].re) / b_r;
      }
    }

    snr[i] = -(4.0 * (b_y_re * y_re - pp * y_im)) * rt_hypotd_snf(h_est[iv11[i]]
      .re, h_est[iv11[i]].im);
  }

  deinterleaver(snr, dv13);
  memcpy(&snr[0], &dv13[0], 48U * sizeof(double));

  /*  Viterbi decoding */
  obj = &vit_obj;
  b_obj = &vit_obj.cSFunObject;

  /* System object Outputs function: comm.ViterbiDecoder */
  minstateLastTB = 0U;
  tbworkLastTB = 0;

  /* Set state metric for all zeros state equal to zero and all other state metrics equal to max values */
  obj->cSFunObject.W6_maxVal = 1.7976931348623157E+308;
  obj->cSFunObject.W1_stateMetric[0U] = 0.0;
  for (i = 0; i < 63; i++) {
    b_obj->W1_stateMetric[i + 1] = 1.7976931348623157E+308;
  }

  /* Set traceback memory to zero */
  for (i = 0; i < 1600; i++) {
    b_obj->W4_tbInput[i] = 0U;
    b_obj->W3_tbState[i] = 0U;
  }

  obj->cSFunObject.W5_tbPtr = 0;
  for (ib = 0; ib < 24; ib++) {
    i = ib << 1;

    /* Branch Metric Computation */
    for (indx1 = 0; indx1 < 4; indx1++) {
      b_obj->W0_bMetric[indx1] = 0.0;
      pp = snr[i + 1];
      if ((indx1 & 1) != 0) {
        pp++;
      } else {
        pp--;
      }

      pp = rt_powd_snf(pp, 2.0);
      b_obj->W0_bMetric[indx1] += pp;
      if ((int)((unsigned int)indx1 >> 1) != 0) {
        b_indx1 = snr[i] + 1.0;
      } else {
        b_indx1 = snr[i] - 1.0;
      }

      b_obj->W0_bMetric[indx1] += rt_powd_snf(b_indx1, 2.0);
    }

    /* State Metric Update */
    minstate = (unsigned int)ACS_D_D(64, b_obj->W2_tempMetric, 2,
      b_obj->W0_bMetric, b_obj->W1_stateMetric, b_obj->W3_tbState,
      b_obj->W4_tbInput, (int *)&b_obj->W5_tbPtr, b_obj->P0_StateVec,
      b_obj->P1_OutputVec, 1.7976931348623157E+308);

    /* TERMINATED mode: Start the final traceback path at the zero state. */
    if (ib == 23) {
      minstate = 0U;
    }

    /* Traceback decoding */
    i = b_obj->W5_tbPtr;

    /* Get starting minstate, starting tbwork for last loop */
    if (ib == 23) {
      minstateLastTB = minstate;
      tbworkLastTB = i;
    }

    /* Do not do traceback for first TbDepth times */
    /* Increment (mod TbDepth) the traceback index and store */
    if (b_obj->W5_tbPtr < 24) {
      b_obj->W5_tbPtr++;
    } else {
      b_obj->W5_tbPtr = 0;
    }

    for (indx1 = 0; indx1 < 24; indx1++) {
      i = (int)(minstateLastTB + (tbworkLastTB << 6));
      minstateLastTB = b_obj->W4_tbInput[i];
      sig_msg[23 - indx1] = (signed char)(minstateLastTB & 1U);
      minstateLastTB = b_obj->W3_tbState[i];
      if (tbworkLastTB > 0) {
        tbworkLastTB--;
      } else {
        tbworkLastTB = 24;
      }
    }
  }

  /*  Compare even parity to detect errors in L-SIG */
  pp = sig_msg[0];
  for (i = 0; i < 16; i++) {
    pp += (double)sig_msg[i + 1];
  }

  /*  Check parity bit */
  if (sig_msg[17] == b_mod(pp)) {
    /*  Length from L-SIG in octets */
    pp = 1.0;
    *SIG_CFG_length = 0.0;
    for (i = 0; i < 12; i++) {
      UPDATE_DECIMAL(SIG_CFG_length, pp, (double)sig_msg[5 + i]);
      pp *= 2.0;
    }

    /*  If length is invalid return (with error) */
    if (*SIG_CFG_length > 5.0) {
      /*  Convert binary datarate to Mbps */
      b_r = 0.0;
      UPDATE_DECIMAL(&b_r, 1.0, (double)sig_msg[0]);
      UPDATE_DECIMAL(&b_r, 2.0, (double)sig_msg[1]);
      UPDATE_DECIMAL(&b_r, 4.0, (double)sig_msg[2]);
      UPDATE_DECIMAL(&b_r, 8.0, (double)sig_msg[3]);

      /*  Find MCS */
      guard1 = false;
      switch ((int)b_r) {
       case 11:
        guard1 = true;
        break;

       case 15:
        *SIG_CFG_mcs = 1.0;
        guard1 = true;
        break;

       case 10:
        *SIG_CFG_mcs = 2.0;
        guard1 = true;
        break;

       case 14:
        *SIG_CFG_mcs = 3.0;
        guard1 = true;
        break;

       case 9:
        *SIG_CFG_mcs = 4.0;
        guard1 = true;
        break;

       case 13:
        *SIG_CFG_mcs = 5.0;
        guard1 = true;
        break;

       case 8:
        *SIG_CFG_mcs = 6.0;
        guard1 = true;
        break;

       case 12:
        *SIG_CFG_mcs = 7.0;
        guard1 = true;
        break;
      }

      if (guard1) {
        /*  Declare non-erroneous SIG after valid MCS detection */
        *SIG_CFG_sig_err = false;

        /*  MCS table */
        /*  Find code rate numerator/denominator & bits per modulation symbol */
        *SIG_CFG_r_num = iv12[(int)*SIG_CFG_mcs];
        *SIG_CFG_r_denom = iv13[(int)*SIG_CFG_mcs];
        *SIG_CFG_n_bpscs = iv14[(int)*SIG_CFG_mcs];

        /*  Calculate coded/uncoded number of bits per OFDM symbol */
        *SIG_CFG_n_cbps = 48.0 * (double)iv14[(int)*SIG_CFG_mcs];
        *SIG_CFG_n_dbps = *SIG_CFG_n_cbps * (double)iv12[(int)*SIG_CFG_mcs] /
          (double)iv13[(int)*SIG_CFG_mcs];

        /*  Number of OFDM symbols */
        *SIG_CFG_n_sym = std::ceil(((16.0 + 8.0 * *SIG_CFG_length) + 6.0) /
          *SIG_CFG_n_dbps);
      }
    }
  }
}

void sig_rx_free()
{
  if (!vit_obj.matlabCodegenIsDeleted) {
    vit_obj.matlabCodegenIsDeleted = true;
    if (vit_obj.isInitialized == 1) {
      vit_obj.isInitialized = 2;
    }
  }
}

void sig_rx_init()
{
  vit_obj_not_empty = false;
  vit_obj.matlabCodegenIsDeleted = true;
}

/* End of code generation (sig_rx.cpp) */
