/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dot11_ifft.cpp
 *
 * Code generation for function 'dot11_ifft'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "dot11_ifft.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void dot11_ifft(const double in[64], creal_T out[64])
{
  int ix;
  int ju;
  int iy;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double twid_re;
  int iheight;
  double twid_im;
  int istart;
  int temp_re_tmp;
  int j;
  int ihi;

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
  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 0; i < 63; i++) {
    out[iy].re = in[iv0[ix]];
    out[iy].im = 0.0;
    iy = 64;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }

    iy = ju;
    ix++;
  }

  out[iy].re = in[iv0[ix]];
  out[iy].im = 0.0;
  for (i = 0; i <= 62; i += 2) {
    temp_re = out[i + 1].re;
    temp_im = out[i + 1].im;
    twid_re = out[i].re;
    twid_im = out[i].im;
    out[i + 1].re = out[i].re - out[i + 1].re;
    out[i + 1].im = out[i].im - out[i + 1].im;
    twid_re += temp_re;
    twid_im += temp_im;
    out[i].re = twid_re;
    out[i].im = twid_im;
  }

  iy = 2;
  ix = 4;
  ju = 16;
  iheight = 61;
  while (ju > 0) {
    for (i = 0; i < iheight; i += ix) {
      temp_re_tmp = i + iy;
      temp_re = out[temp_re_tmp].re;
      temp_im = out[i + iy].im;
      out[temp_re_tmp].re = out[i].re - out[temp_re_tmp].re;
      out[temp_re_tmp].im = out[i].im - temp_im;
      out[i].re += temp_re;
      out[i].im += temp_im;
    }

    istart = 1;
    for (j = ju; j < 32; j += ju) {
      twid_re = dv0[j];
      twid_im = dv1[j];
      i = istart;
      ihi = istart + iheight;
      while (i < ihi) {
        temp_re_tmp = i + iy;
        temp_re = twid_re * out[temp_re_tmp].re - twid_im * out[i + iy].im;
        temp_im = twid_re * out[i + iy].im + twid_im * out[i + iy].re;
        out[temp_re_tmp].re = out[i].re - temp_re;
        out[temp_re_tmp].im = out[i].im - temp_im;
        out[i].re += temp_re;
        out[i].im += temp_im;
        i += ix;
      }

      istart++;
    }

    ju /= 2;
    iy = ix;
    ix += ix;
    iheight -= iy;
  }

  for (iy = 0; iy < 64; iy++) {
    out[iy].re = 64.0 * (0.015625 * out[iy].re);
    out[iy].im = 64.0 * (0.015625 * out[iy].im);
  }
}

/* End of code generation (dot11_ifft.cpp) */
