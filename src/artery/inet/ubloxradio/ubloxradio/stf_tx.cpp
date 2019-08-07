/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stf_tx.cpp
 *
 * Code generation for function 'stf_tx'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "stf_tx.h"
#include "fft.h"
#include "kaiser.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void stf_tx(double w_beta, creal_T stf_wf[160])
{
  double dv5[64];
  int i;
  creal_T stf_f[64];
  static const creal_T dcv0[64] = { { 0.0,/* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.70710678118654757,          /* re */
      0.70710678118654757              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { -0.70710678118654757,         /* re */
      -0.70710678118654757             /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.70710678118654757,          /* re */
      0.70710678118654757              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { -0.70710678118654757,         /* re */
      -0.70710678118654757             /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { -0.70710678118654757,         /* re */
      -0.70710678118654757             /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.70710678118654757,          /* re */
      0.70710678118654757              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { -0.70710678118654757,         /* re */
      -0.70710678118654757             /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { -0.70710678118654757,         /* re */
      -0.70710678118654757             /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.70710678118654757,          /* re */
      0.70710678118654757              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.70710678118654757,          /* re */
      0.70710678118654757              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.70710678118654757,          /* re */
      0.70710678118654757              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.70710678118654757,          /* re */
      0.70710678118654757              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    }, { 0.0,                          /* re */
      0.0                              /* im */
    } };

  creal_T b_stf_f[64];

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
    kaiser(w_beta, dv5);
    for (i = 0; i < 64; i++) {
      stf_f[i].re = dv5[i] * dcv0[i].re;
      stf_f[i].im = dv5[i] * dcv0[i].im;
    }

    /*  Base STF waveform */
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
      b_stf_f[i] = stf_f[iv0[i]];
    }

    r2br_r2dit_trig_impl(b_stf_f, dv0, dv1, stf_wf_base);
    for (i = 0; i < 64; i++) {
      stf_wf_base[i].re = 0.28867513459481292 * (64.0 * (0.015625 *
        stf_wf_base[i].re));
      stf_wf_base[i].im = 0.28867513459481292 * (64.0 * (0.015625 *
        stf_wf_base[i].im));
    }

    stf_wf_base_not_empty = true;
  }

  /*  Append CP */
  memcpy(&stf_wf[0], &stf_wf_base[32], sizeof(creal_T) << 5);
  for (i = 0; i < 64; i++) {
    stf_wf[i + 32] = stf_wf_base[i];
    stf_wf[i + 96] = stf_wf_base[i];
  }
}

void stf_wf_base_not_empty_init()
{
  stf_wf_base_not_empty = false;
}

/* End of code generation (stf_tx.cpp) */
