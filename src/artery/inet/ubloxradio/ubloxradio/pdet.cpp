/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * pdet.cpp
 *
 * Code generation for function 'pdet'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "pdet.h"
#include "cmlri.h"
#include "sim_rx_mex_emxutil.h"
#include "kaiser.h"
#include "xcorr.h"
#include "sim_rx_mex_rtwutil.h"
#include "sim_rx_mex_data.h"

/* Function Declarations */
static int div_s32_floor(int numerator, int denominator);

/* Function Definitions */
static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator < 0) {
      absNumerator = ~(unsigned int)numerator + 1U;
    } else {
      absNumerator = (unsigned int)numerator;
    }

    if (denominator < 0) {
      absDenominator = ~(unsigned int)denominator + 1U;
    } else {
      absDenominator = (unsigned int)denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }

      quotient = -(int)tempAbsQuotient;
    } else {
      quotient = (int)tempAbsQuotient;
    }
  }

  return quotient;
}

void pdet(const emxArray_creal_T *in, double s0_len, double pdet_thold, double
          *idx, double c_cfo_data[], int c_cfo_size[2], boolean_T *err)
{
  static const creal_T dcv2[64] = { { 0.40824829046386307,/* re */
      0.40824829046386307              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054458             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { 1.2669822230356942,           /* re */
      -0.1122816846564426              /* im */
    }, { 0.81649658092772615,          /* re */
      0.0                              /* im */
    }, { 1.2669822230356946,           /* re */
      -0.11228168465644235             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054586             /* im */
    }, { 0.40824829046386307,          /* re */
      0.40824829046386307              /* im */
    }, { 0.020764353243054586,         /* re */
      -1.1754648916223065              /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { -0.11228168465644235,         /* re */
      1.2669822230356946               /* im */
    }, { 0.0,                          /* re */
      0.81649658092772615              /* im */
    }, { -0.1122816846564426,          /* re */
      1.2669822230356942               /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { 0.020764353243054458,         /* re */
      -1.1754648916223065              /* im */
    }, { 0.40824829046386307,          /* re */
      0.40824829046386307              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054458             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { 1.2669822230356942,           /* re */
      -0.1122816846564426              /* im */
    }, { 0.81649658092772615,          /* re */
      0.0                              /* im */
    }, { 1.2669822230356946,           /* re */
      -0.11228168465644235             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054586             /* im */
    }, { 0.40824829046386307,          /* re */
      0.40824829046386307              /* im */
    }, { 0.020764353243054586,         /* re */
      -1.1754648916223065              /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { -0.11228168465644235,         /* re */
      1.2669822230356946               /* im */
    }, { 0.0,                          /* re */
      0.81649658092772615              /* im */
    }, { -0.1122816846564426,          /* re */
      1.2669822230356942               /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { 0.020764353243054458,         /* re */
      -1.1754648916223065              /* im */
    }, { 0.40824829046386307,          /* re */
      0.40824829046386307              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054458             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { 1.2669822230356942,           /* re */
      -0.1122816846564426              /* im */
    }, { 0.81649658092772615,          /* re */
      0.0                              /* im */
    }, { 1.2669822230356946,           /* re */
      -0.11228168465644235             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054586             /* im */
    }, { 0.40824829046386307,          /* re */
      0.40824829046386307              /* im */
    }, { 0.020764353243054586,         /* re */
      -1.1754648916223065              /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { -0.11228168465644235,         /* re */
      1.2669822230356946               /* im */
    }, { 0.0,                          /* re */
      0.81649658092772615              /* im */
    }, { -0.1122816846564426,          /* re */
      1.2669822230356942               /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { 0.020764353243054458,         /* re */
      -1.1754648916223065              /* im */
    }, { 0.40824829046386307,          /* re */
      0.40824829046386307              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054458             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { 1.2669822230356942,           /* re */
      -0.1122816846564426              /* im */
    }, { 0.81649658092772615,          /* re */
      0.0                              /* im */
    }, { 1.2669822230356946,           /* re */
      -0.11228168465644235             /* im */
    }, { -0.11957315586905011,         /* re */
      -0.6969234250586761              /* im */
    }, { -1.1754648916223065,          /* re */
      0.020764353243054586             /* im */
    }, { 0.40824829046386307,          /* re */
      0.40824829046386307              /* im */
    }, { 0.020764353243054586,         /* re */
      -1.1754648916223065              /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { -0.11228168465644235,         /* re */
      1.2669822230356946               /* im */
    }, { 0.0,                          /* re */
      0.81649658092772615              /* im */
    }, { -0.1122816846564426,          /* re */
      1.2669822230356942               /* im */
    }, { -0.6969234250586761,          /* re */
      -0.11957315586905011             /* im */
    }, { 0.020764353243054458,         /* re */
      -1.1754648916223065              /* im */
    } };

  int loop_ub;
  int maxval;
  int i;
  emxArray_creal_T *b_in;
  creal_T dcv3[64];
  int i13;
  emxArray_creal_T *xc;
  emxArray_creal_T *c1;
  int k;
  emxArray_real_T *xc_crop;
  emxArray_boolean_T *x;
  boolean_T exitg1;
  int ii_data[1];
  unsigned int idx_data[1];
  double re;
  creal_T r1[16];
  double im;
  double b_re;
  double b_im;

  /* PDET Detects start of packet */
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
  /*  Obtain original STF waveform */
  /* STF_TX Generates STF preamble */
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
  /*  Store this as a persistent variable to avoid recalculation */
  if (!stf_wf_base_not_empty) {
    /*  STF f-domain represenation (including DC-subcarrier & guard bands) */
    /*  Apply spectral shaping window */
    /*  Base STF waveform */
    memcpy(&stf_wf_base[0], &dcv2[0], sizeof(creal_T) << 6);
    stf_wf_base_not_empty = true;
  }

  /*  Append CP */
  /*  Input STF signal */
  if (1.0 > s0_len + 80.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)(s0_len + 80.0);
  }

  /*  X-correlation of received waveform with known STF */
  if (64 > loop_ub) {
    maxval = 64;
  } else {
    maxval = loop_ub;
  }

  for (i = 0; i < 32; i++) {
    dcv3[i] = stf_wf_base[i + 32];
    dcv3[i + 32] = stf_wf_base[i];
  }

  emxInit_creal_T(&b_in, 1);
  i13 = b_in->size[0];
  b_in->size[0] = loop_ub;
  emxEnsureCapacity_creal_T(b_in, i13);
  for (i13 = 0; i13 < loop_ub; i13++) {
    b_in->data[i13] = in->data[i13];
  }

  emxInit_creal_T(&xc, 1);
  emxInit_creal_T(&c1, 1);
  crosscorr(dcv3, b_in, (double)maxval - 1.0, c1);
  i13 = xc->size[0];
  xc->size[0] = 2 * (maxval - 1) + 1;
  emxEnsureCapacity_creal_T(xc, i13);
  loop_ub = 2 * (maxval - 1) + 1;
  emxFree_creal_T(&b_in);
  for (i13 = 0; i13 < loop_ub; i13++) {
    xc->data[i13].re = 0.0;
    xc->data[i13].im = 0.0;
  }

  loop_ub = c1->size[0] - 1;
  for (i13 = 0; i13 <= loop_ub; i13++) {
    xc->data[i13] = c1->data[i13];
  }

  if (1.0 > (s0_len + 80.0) + 63.0) {
    i13 = 1;
    i = 1;
    maxval = 0;
  } else {
    i13 = (int)((s0_len + 80.0) + 63.0);
    i = -1;
    maxval = 1;
  }

  k = c1->size[0];
  loop_ub = div_s32_floor(maxval - i13, i);
  maxval = loop_ub + 1;
  c1->size[0] = maxval;
  emxEnsureCapacity_creal_T(c1, k);
  for (k = 0; k <= loop_ub; k++) {
    c1->data[k] = xc->data[(i13 + i * k) - 1];
  }

  emxFree_creal_T(&xc);
  emxInit_real_T(&xc_crop, 1);
  i13 = xc_crop->size[0];
  xc_crop->size[0] = maxval;
  emxEnsureCapacity_real_T(xc_crop, i13);
  for (k = 0; k <= loop_ub; k++) {
    xc_crop->data[k] = rt_hypotd_snf(c1->data[k].re, c1->data[k].im);
  }

  emxFree_creal_T(&c1);
  emxInit_boolean_T(&x, 1);

  /*  Find first occurence of value above threshold */
  i13 = x->size[0];
  x->size[0] = xc_crop->size[0];
  emxEnsureCapacity_boolean_T(x, i13);
  loop_ub = xc_crop->size[0];
  for (i13 = 0; i13 < loop_ub; i13++) {
    x->data[i13] = (xc_crop->data[i13] > pdet_thold);
  }

  emxFree_real_T(&xc_crop);
  k = (1 <= x->size[0]);
  i = 0;
  maxval = 0;
  exitg1 = false;
  while ((!exitg1) && (maxval <= x->size[0] - 1)) {
    if (x->data[maxval]) {
      i = 1;
      ii_data[0] = maxval + 1;
      exitg1 = true;
    } else {
      maxval++;
    }
  }

  emxFree_boolean_T(&x);
  if (k == 1) {
    if (i == 0) {
      k = 0;
    }
  } else {
    k = (1 <= i);
  }

  for (i13 = 0; i13 < k; i13++) {
    idx_data[0] = (unsigned int)ii_data[0];
  }

  /*  Check if index is valid, else declare packet error */
  if ((k == 0) || (idx_data[0] < s0_len)) {
    c_cfo_size[0] = 0;
    c_cfo_size[1] = 0;
    *idx = 0.0;
    *err = true;
  } else {
    *idx = idx_data[0];
    *err = false;

    /*  Coarse CFO estimation */
    for (i13 = 0; i13 < 16; i13++) {
      i = (int)(i13 + idx_data[0]);
      maxval = i - 1;
      re = in->data[maxval].re;
      im = in->data[maxval].im;
      maxval = i + 15;
      b_re = in->data[maxval].re;
      b_im = in->data[maxval].im;
      r1[i13].re = re * b_re - im * -b_im;
      r1[i13].im = re * -b_im + im * b_re;
    }

    re = r1[0].re;
    im = r1[0].im;
    for (k = 0; k < 15; k++) {
      re += r1[k + 1].re;
      im += r1[k + 1].im;
    }

    c_cfo_size[0] = 1;
    c_cfo_size[1] = 1;
    c_cfo_data[0] = -rt_atan2d_snf(im, re) / 16.0 / 2.0 / 3.1415926535897931;
  }
}

/* End of code generation (pdet.cpp) */
