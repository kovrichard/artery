/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * scrambler_tx.cpp
 *
 * Code generation for function 'scrambler_tx'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "scrambler_tx.h"
#include "sim_rx_mex_emxutil.h"

/* Function Definitions */
void scrambler_tx(const emxArray_boolean_T *in, boolean_T pn_state[7],
                  emxArray_boolean_T *out)
{
  int i24;
  int i;
  int ii;
  boolean_T tmp2[6];

  /* SCRAMBLER Bit scrambler */
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
  /*  Initialize output */
  /*  Perform scrambling per bit, use PN = X^7 + X^4 + 1 */
  i24 = in->size[0];
  i = out->size[0];
  out->size[0] = in->size[0];
  emxEnsureCapacity_boolean_T(out, i);
  for (ii = 0; ii < i24; ii++) {
    for (i = 0; i < 6; i++) {
      tmp2[i] = pn_state[i];
    }

    pn_state[0] = ((pn_state[6] ^ pn_state[3]) != 0);
    for (i = 0; i < 6; i++) {
      pn_state[1 + i] = tmp2[i];
    }

    out->data[ii] = ((in->data[ii] ^ pn_state[0]) != 0);
  }
}

/* End of code generation (scrambler_tx.cpp) */
