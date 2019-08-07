/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx.cpp
 *
 * Code generation for function 'sim_rx'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sim_rx_mex_emxutil.h"
#include "data_tx.h"
#include "data_rx.h"
#include "sig_rx.h"
#include "fft.h"
#include "apply_cfo.h"
#include "cmlri.h"
#include "sum.h"
#include "abs.h"
#include "xcorr.h"
#include "ltf_tx.h"
#include "pdet.h"
#include "sim_rx_mex_rtwutil.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void sim_rx(emxArray_creal_T *rx_wf, double s0_len, double pdet_thold,
            emxArray_real_T *pld_bytes, double *err)
{
  double c_idx;
  double c_cfo_data[1];
  int c_cfo_size[2];
  boolean_T pdet_err;
  int b_err;
  int loop_ub;
  int i12;
  emxArray_real_T *y;
  double r;
  double ar;
  creal_T z_data[1];
  emxArray_creal_T *b_y;
  emxArray_creal_T *c_y;
  int nx;
  int k;
  emxArray_creal_T *f;
  double y_im;
  double f_im;
  creal_T ltf_wf[160];
  creal_T xc_in[64];
  creal_T dcv1[127];
  double xc[127];
  boolean_T exitg1;
  double f_idx;
  creal_T r2[64];
  emxArray_creal_T *b_rx_wf;
  creal_T b_xc_in[64];
  creal_T x;
  creal_T wf_in[128];
  double SIG_CFG_length;
  double SIG_CFG_n_dbps;
  double SIG_CFG_n_sym;
  double r_cfo;
  emxArray_uint32_T *pld_crc32;
  emxArray_boolean_T *rx_out;
  emxArray_boolean_T *b;
  emxArray_real_T *d;
  emxArray_real_T *in;
  double PHY_pilot_idx[4];
  static const double PHY_polarity_sign[127] = { 1.0, 1.0, 1.0, 1.0, -1.0, -1.0,
    -1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0,
    -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0, -1.0, 1.0,
    -1.0, 1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0,
    1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0,
    1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0,
    1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, -1.0, -1.0,
    1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 };

  static const double PHY_data_idx[48] = { 7.0, 8.0, 9.0, 10.0, 11.0, 13.0, 14.0,
    15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 27.0, 28.0,
    29.0, 30.0, 31.0, 32.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 41.0, 42.0, 43.0,
    44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0, 53.0, 55.0, 56.0, 57.0,
    58.0, 59.0 };

  boolean_T rx_out_data[32759];
  unsigned int crc;
  int data_size_idx_1;
  unsigned char data_data[32759];
  unsigned char u1;
  unsigned int mask;
  unsigned int b_mask;
  boolean_T b_x[4];

  /* SIM_RX High-level receiver function */
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
  /*  Needed for code generation */
  /*  Initialize PHY parameters */
  /*  Packet detection / coarse CFO estimation */
  pdet(rx_wf, s0_len, pdet_thold, &c_idx, c_cfo_data, c_cfo_size, &pdet_err);

  /*  If no packet error, proceed with packet decoding */
  b_err = 0;
  pld_bytes->size[0] = 0;
  pld_bytes->size[1] = 0;
  if (pdet_err) {
    b_err = 1;
  } else {
    /*  Coarse CFO correction */
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
    loop_ub = c_cfo_size[0] * c_cfo_size[1];
    for (i12 = 0; i12 < loop_ub; i12++) {
      r = -c_cfo_data[i12] / 5.9E+9 * 1.0E+7;
      ar = r * 0.0 * 5.9E+9;
      r = r * 6.2831853071795862 * 5.9E+9;
      if (r == 0.0) {
        z_data[i12].re = ar / 1.0E+7;
        z_data[i12].im = 0.0;
      } else if (ar == 0.0) {
        z_data[i12].re = 0.0;
        z_data[i12].im = r / 1.0E+7;
      } else {
        z_data[i12].re = rtNaN;
        z_data[i12].im = r / 1.0E+7;
      }
    }

    emxInit_real_T(&y, 2);
    if (rx_wf->size[0] - 1 < 0) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else {
      ar = (double)rx_wf->size[0] - 1.0;
      i12 = y->size[0] * y->size[1];
      y->size[0] = 1;
      loop_ub = (int)ar;
      y->size[1] = loop_ub + 1;
      emxEnsureCapacity_real_T(y, i12);
      for (i12 = 0; i12 <= loop_ub; i12++) {
        y->data[i12] = i12;
      }
    }

    emxInit_creal_T(&b_y, 2);
    i12 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = y->size[1];
    emxEnsureCapacity_creal_T(b_y, i12);
    loop_ub = y->size[0] * y->size[1];
    for (i12 = 0; i12 < loop_ub; i12++) {
      b_y->data[i12].re = y->data[i12];
      b_y->data[i12].im = 0.0;
    }

    emxFree_real_T(&y);
    emxInit_creal_T(&c_y, 2);
    i12 = c_y->size[0] * c_y->size[1];
    c_y->size[0] = c_cfo_size[0];
    c_y->size[1] = b_y->size[1];
    emxEnsureCapacity_creal_T(c_y, i12);
    loop_ub = c_cfo_size[0];
    for (i12 = 0; i12 < loop_ub; i12++) {
      nx = b_y->size[1];
      for (k = 0; k < nx; k++) {
        r = b_y->data[k].re;
        y_im = b_y->data[k].im;
        c_y->data[i12 + c_y->size[0] * k].re = z_data[i12].re * r - z_data[i12].
          im * y_im;
        c_y->data[i12 + c_y->size[0] * k].im = z_data[i12].re * y_im +
          z_data[i12].im * r;
      }
    }

    emxFree_creal_T(&b_y);
    nx = c_y->size[0] * c_y->size[1];
    for (k = 0; k < nx; k++) {
      if (c_y->data[k].im == 0.0) {
        r = c_y->data[k].re;
        c_y->data[k].re = std::exp(r);
        c_y->data[k].im = 0.0;
      } else if (rtIsInf(c_y->data[k].im) && rtIsInf(c_y->data[k].re) &&
                 (c_y->data[k].re < 0.0)) {
        c_y->data[k].re = 0.0;
        c_y->data[k].im = 0.0;
      } else {
        r = std::exp(c_y->data[k].re / 2.0);
        y_im = c_y->data[k].im;
        ar = c_y->data[k].im;
        c_y->data[k].re = r * (r * std::cos(y_im));
        c_y->data[k].im = r * (r * std::sin(ar));
      }
    }

    emxInit_creal_T(&f, 2);
    i12 = f->size[0] * f->size[1];
    f->size[0] = c_y->size[1];
    f->size[1] = c_y->size[0];
    emxEnsureCapacity_creal_T(f, i12);
    loop_ub = c_y->size[0];
    for (i12 = 0; i12 < loop_ub; i12++) {
      nx = c_y->size[1];
      for (k = 0; k < nx; k++) {
        f->data[k + f->size[0] * i12] = c_y->data[i12 + c_y->size[0] * k];
      }
    }

    emxFree_creal_T(&c_y);

    /*  Apply shift */
    i12 = rx_wf->size[0];
    emxEnsureCapacity_creal_T(rx_wf, i12);
    loop_ub = rx_wf->size[0];
    for (i12 = 0; i12 < loop_ub; i12++) {
      r = rx_wf->data[i12].re;
      ar = rx_wf->data[i12].im;
      y_im = f->data[i12].re;
      f_im = f->data[i12].im;
      rx_wf->data[i12].re = r * y_im - ar * f_im;
      rx_wf->data[i12].im = r * f_im + ar * y_im;
    }

    emxFree_creal_T(&f);

    /*  Fine synchronization / fine CFO estimation */
    /* FINE_SYNC Fine synchronization */
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
    /*  Obtain original LTF waveform */
    ltf_tx(0.0, ltf_wf);

    /*  Input LTF signal */
    for (i12 = 0; i12 < 64; i12++) {
      xc_in[i12] = rx_wf->data[(int)(c_idx + (160.0 + (double)i12)) - 1];
    }

    /*  Cross-correlation of input with reference signals */
    xcorr(*(creal_T (*)[64])&ltf_wf[32], xc_in, dcv1);
    b_abs(dcv1, xc);

    /*  Find the maximum value */
    if (!rtIsNaN(xc[0])) {
      nx = 1;
    } else {
      nx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k < 128)) {
        if (!rtIsNaN(xc[k - 1])) {
          nx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (nx == 0) {
      nx = 1;
    } else {
      r = xc[nx - 1];
      i12 = nx + 1;
      for (k = i12; k < 128; k++) {
        ar = xc[k - 1];
        if (r < ar) {
          r = ar;
          nx = k;
        }
      }
    }

    /*  Adjust index */
    f_idx = (216.0 - (double)nx) + c_idx;

    /*  Fine CFO estimation */
    for (i12 = 0; i12 < 64; i12++) {
      xc_in[i12] = rx_wf->data[(int)(f_idx + (double)i12) - 1];
    }

    for (i12 = 0; i12 < 64; i12++) {
      r2[i12] = rx_wf->data[(int)(f_idx + (64.0 + (double)i12)) - 1];
    }

    for (nx = 0; nx < 64; nx++) {
      b_xc_in[nx].re = xc_in[nx].re * r2[nx].re - xc_in[nx].im * -r2[nx].im;
      b_xc_in[nx].im = xc_in[nx].re * -r2[nx].im + xc_in[nx].im * r2[nx].re;
    }

    emxInit_creal_T(&b_rx_wf, 1);
    x = sum(b_xc_in);

    /*  Fine CFO correction */
    i12 = b_rx_wf->size[0];
    b_rx_wf->size[0] = rx_wf->size[0];
    emxEnsureCapacity_creal_T(b_rx_wf, i12);
    loop_ub = rx_wf->size[0];
    for (i12 = 0; i12 < loop_ub; i12++) {
      b_rx_wf->data[i12] = rx_wf->data[i12];
    }

    apply_cfo(b_rx_wf, -(-rt_atan2d_snf(x.im, x.re) / 64.0 / 2.0 /
                         3.1415926535897931) / 5.9E+9 * 1.0E+7, rx_wf);

    /*  Channel estimation */
    emxFree_creal_T(&b_rx_wf);
    for (i12 = 0; i12 < 128; i12++) {
      wf_in[i12] = rx_wf->data[(int)(f_idx + (double)i12) - 1];
    }

    /* CHAN_EST Channel estimation algorithm, using LTF preamble */
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
    /*  LTF f-domain represenation (including DC-subcarrier & guard bands) */
    /*  Average the two t-domain symbols */
    /*  FFT on the averaged sequence */
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
    /*  Least-Squares channel estimation */
    for (i12 = 0; i12 < 64; i12++) {
      ar = wf_in[i12].re + wf_in[64 + i12].re;
      r = wf_in[i12].im + wf_in[64 + i12].im;
      if (r == 0.0) {
        b_xc_in[i12].re = ar / 2.0;
        b_xc_in[i12].im = 0.0;
      } else if (ar == 0.0) {
        b_xc_in[i12].re = 0.0;
        b_xc_in[i12].im = r / 2.0;
      } else {
        b_xc_in[i12].re = ar / 2.0;
        b_xc_in[i12].im = r / 2.0;
      }
    }

    for (i12 = 0; i12 < 64; i12++) {
      r2[i12] = b_xc_in[iv5[i12]];
    }

    r2br_r2dit_trig_impl(r2, dv0, dv3, xc_in);
    for (nx = 0; nx < 64; nx++) {
      ar = xc_in[iv0[nx]].re * 7.2111025509279782;
      r = xc_in[iv0[nx]].im * 7.2111025509279782;
      if (r == 0.0) {
        ar /= 64.0;
        r = 0.0;
      } else if (ar == 0.0) {
        ar = 0.0;
        r /= 64.0;
      } else {
        ar /= 64.0;
        r /= 64.0;
      }

      if (r == 0.0) {
        r2[nx].re = ar / (double)iv1[nx];
        r2[nx].im = 0.0;
      } else if (ar == 0.0) {
        r2[nx].re = 0.0;
        r2[nx].im = r / (double)iv1[nx];
      } else {
        r2[nx].re = ar / (double)iv1[nx];
        r2[nx].im = r / (double)iv1[nx];
      }
    }

    /*  SIG reception */
    for (i12 = 0; i12 < 64; i12++) {
      xc_in[i12] = rx_wf->data[(int)(((f_idx + 128.0) + 16.0) + (double)i12) - 1];
    }

    memcpy(&b_xc_in[0], &r2[0], sizeof(creal_T) << 6);
    sig_rx(xc_in, b_xc_in, &SIG_CFG_length, &r, &pdet_err, &ar, &y_im, &f_im,
           &c_idx, &SIG_CFG_n_dbps, &SIG_CFG_n_sym, &r_cfo);

    /*  Identify SIG errors from parity or length inconsistency and abort or proceed with data processing */
    emxInit_uint32_T(&pld_crc32, 2);
    emxInit_boolean_T(&rx_out, 1);
    emxInit_boolean_T(&b, 2);
    emxInit_real_T(&d, 1);
    emxInit_real_T(&in, 2);
    if (pdet_err || ((SIG_CFG_n_sym * 80.0 + 63.0) + ((f_idx + 128.0) + 16.0) >
                     rx_wf->size[0])) {
      b_err = 2;
    } else {
      /*  Update PHY parameters based on SIG */
      PHY_pilot_idx[0] = 12.0;
      PHY_pilot_idx[1] = 26.0;
      PHY_pilot_idx[2] = 40.0;
      PHY_pilot_idx[3] = 54.0;

      /* UPDATE_PHY_PARAMS Updates PHY layer parameters */
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
      /*  MCS tables for coding rate (numerator / denominator) and bits per modulation symbol */
      /*  Find code rate numerator/denominator & bits per modulation symbol */
      /*  Calculate coded/uncoded number of bits per OFDM symbol */
      /*  Calculate number of OFDM symbols */
      /*  Data processing */
      data_rx(PHY_pilot_idx, PHY_polarity_sign, PHY_data_idx, ar, f_im, c_idx,
              SIG_CFG_n_dbps, SIG_CFG_n_sym, rx_wf, (f_idx + 128.0) + 16.0, r2,
              r_cfo, rx_out);

      /*  Check if payload length is correct */
      if (SIG_CFG_length >= 5.0) {
        /*  Convert to bytes */
        loop_ub = (int)((10.0 + SIG_CFG_length * 8.0) - 1.0) - 10;
        for (i12 = 0; i12 <= loop_ub; i12++) {
          rx_out_data[i12] = rx_out->data[9 + i12];
        }

        i12 = b->size[0] * b->size[1];
        loop_ub = (int)SIG_CFG_length;
        b->size[0] = loop_ub;
        b->size[1] = 8;
        emxEnsureCapacity_boolean_T(b, i12);
        for (i12 = 0; i12 < 8; i12++) {
          for (k = 0; k < loop_ub; k++) {
            b->data[k + b->size[0] * i12] = rx_out_data[i12 + (k << 3)];
          }
        }

        r = 1.0;
        i12 = d->size[0];
        d->size[0] = b->size[0];
        emxEnsureCapacity_real_T(d, i12);
        loop_ub = b->size[0];
        for (i12 = 0; i12 < loop_ub; i12++) {
          d->data[i12] = 0.0;
        }

        for (k = 0; k < 8; k++) {
          i12 = b->size[0];
          for (nx = 0; nx < i12; nx++) {
            ar = d->data[nx];
            if (b->data[nx + b->size[0] * k]) {
              ar = d->data[nx] + r;
            }

            d->data[nx] = ar;
          }

          r *= 2.0;
        }

        i12 = pld_bytes->size[0] * pld_bytes->size[1];
        pld_bytes->size[0] = d->size[0];
        pld_bytes->size[1] = 1;
        emxEnsureCapacity_real_T(pld_bytes, i12);
        loop_ub = d->size[0];
        for (i12 = 0; i12 < loop_ub; i12++) {
          pld_bytes->data[i12] = d->data[i12];
        }

        /*  Calculate CRC-32 */
        nx = (int)(SIG_CFG_length - 4.0);
        i12 = in->size[0] * in->size[1];
        in->size[0] = 1;
        in->size[1] = nx;
        emxEnsureCapacity_real_T(in, i12);
        for (i12 = 0; i12 < nx; i12++) {
          in->data[i12] = d->data[i12];
        }

        /* CRC32 Appends CRC32 on an input bitstream */
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
        /*  Initialize CRC */
        crc = MAX_uint32_T;
        data_size_idx_1 = in->size[1];
        loop_ub = in->size[0] * in->size[1];
        for (i12 = 0; i12 < loop_ub; i12++) {
          ar = rt_roundd_snf(in->data[i12]);
          if (ar < 256.0) {
            if (ar >= 0.0) {
              u1 = (unsigned char)ar;
            } else {
              u1 = 0U;
            }
          } else if (ar >= 256.0) {
            u1 = MAX_uint8_T;
          } else {
            u1 = 0U;
          }

          data_data[i12] = u1;
        }

        /*  Compute CRC-32 value */
        for (nx = 0; nx < data_size_idx_1; nx++) {
          crc ^= data_data[nx];
          for (k = 0; k < 8; k++) {
            mask = ~(crc & 1U);
            if (mask == MAX_uint32_T) {
              b_mask = 0U;
            } else {
              b_mask = mask + 1U;
            }

            crc = crc >> 1U ^ (b_mask & 3988292384U);
          }
        }

        crc = ~crc;

        /*  Output vector */
        i12 = pld_crc32->size[0] * pld_crc32->size[1];
        pld_crc32->size[0] = 1;
        pld_crc32->size[1] = in->size[1] + 4;
        emxEnsureCapacity_uint32_T(pld_crc32, i12);
        loop_ub = in->size[1];
        for (i12 = 0; i12 < loop_ub; i12++) {
          ar = rt_roundd_snf(in->data[i12]);
          if (ar < 4.294967296E+9) {
            if (ar >= 0.0) {
              mask = (unsigned int)ar;
            } else {
              mask = 0U;
            }
          } else if (ar >= 4.294967296E+9) {
            mask = MAX_uint32_T;
          } else {
            mask = 0U;
          }

          pld_crc32->data[i12] = mask;
        }

        pld_crc32->data[in->size[1]] = crc & 255U;
        pld_crc32->data[in->size[1] + 1] = crc >> 8U & 255U;
        pld_crc32->data[in->size[1] + 2] = crc >> 16U & 255U;
        pld_crc32->data[in->size[1] + 3] = crc >> 24U;

        /*  Check CRC for errors */
        nx = (int)(SIG_CFG_length + -3.0) - 1;
        b_x[0] = (pld_crc32->data[nx] != d->data[nx]);
        nx = (int)(SIG_CFG_length + -2.0) - 1;
        b_x[1] = (pld_crc32->data[nx] != d->data[nx]);
        nx = (int)(SIG_CFG_length + -1.0) - 1;
        b_x[2] = (pld_crc32->data[nx] != d->data[nx]);
        nx = (int)SIG_CFG_length - 1;
        b_x[3] = (pld_crc32->data[nx] != d->data[nx]);
        pdet_err = false;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k < 4)) {
          if (b_x[k]) {
            pdet_err = true;
            exitg1 = true;
          } else {
            k++;
          }
        }

        if (pdet_err) {
          b_err = 3;
        }
      } else {
        b_err = 2;
      }
    }

    emxFree_real_T(&in);
    emxFree_real_T(&d);
    emxFree_boolean_T(&b);
    emxFree_boolean_T(&rx_out);
    emxFree_uint32_T(&pld_crc32);
  }

  *err = b_err;
}

/* End of code generation (sim_rx.cpp) */
