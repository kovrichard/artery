/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * deinterleaver.cpp
 *
 * Code generation for function 'deinterleaver'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "deinterleaver.h"
#include "mod.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void deinterleaver(const double in[48], double out[48])
{
  double dv11[48];
  int i18;

  /* DEINTERLEAVER Bit deinterleaver */
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
  /*  s-parameter */
  /*  Initialize output */
  /*  Input index */
  /*  First permutation */
  /*  Second permutation */
  /*  Deinterleaver mapping */
  c_mod(dv2, dv11);
  for (i18 = 0; i18 < 48; i18++) {
    out[i18] = in[(int)(((double)iv4[i18] + dv11[i18]) + 1.0) - 1];
  }
}

/* End of code generation (deinterleaver.cpp) */
