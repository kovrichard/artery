/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ltf_tx.cpp
 *
 * Code generation for function 'ltf_tx'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "ltf_tx.h"
#include "dot11_ifft.h"
#include "kaiser.h"
#include "sim_rx_mex_data.h"

/* Variable Definitions */
static creal_T ltf_wf_base[64];
static boolean_T ltf_wf_base_not_empty;

/* Function Definitions */
void ltf_tx(double w_beta, creal_T ltf_wf[160])
{
  double ltf_f[64];
  int i;

  /* LTF_TX Generates LTF preamble */
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
  if (!ltf_wf_base_not_empty) {
    /*  LTF f-domain represenation (including DC-subcarrier & guard bands) */
    /*  Apply spectral shaping window */
    kaiser(w_beta, ltf_f);
    for (i = 0; i < 64; i++) {
      ltf_f[i] *= (double)iv1[i];
    }

    /*  Base LTF waveform */
    dot11_ifft(ltf_f, ltf_wf_base);
    for (i = 0; i < 64; i++) {
      ltf_wf_base[i].re *= 0.13867504905630729;
      ltf_wf_base[i].im *= 0.13867504905630729;
    }

    ltf_wf_base_not_empty = true;
  }

  /*  Append CP */
  memcpy(&ltf_wf[0], &ltf_wf_base[32], sizeof(creal_T) << 5);
  for (i = 0; i < 64; i++) {
    ltf_wf[i + 32] = ltf_wf_base[i];
    ltf_wf[i + 96] = ltf_wf_base[i];
  }
}

void ltf_wf_base_not_empty_init()
{
  ltf_wf_base_not_empty = false;
}

/* End of code generation (ltf_tx.cpp) */
