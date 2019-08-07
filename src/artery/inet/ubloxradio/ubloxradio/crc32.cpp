/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * crc32.cpp
 *
 * Code generation for function 'crc32'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "crc32.h"

/* Function Definitions */
void crc32(const unsigned int in_data[], const int in_size[2], unsigned int
           out_data[], int out_size[2])
{
  unsigned int crc;
  int loop_ub;
  int i1;
  unsigned int mask;
  unsigned char data_data[4096];
  int j;
  unsigned int b_mask;

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
  loop_ub = in_size[0] * in_size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    mask = in_data[i1];
    if (in_data[i1] > 255U) {
      mask = 255U;
    }

    data_data[i1] = (unsigned char)mask;
  }

  /*  Compute CRC-32 value */
  i1 = in_size[1];
  for (loop_ub = 0; loop_ub < i1; loop_ub++) {
    crc ^= data_data[loop_ub];
    for (j = 0; j < 8; j++) {
      mask = ~(crc & 1U);
      if (mask == MAX_uint32_T) {
        b_mask = 0U;
      } else {
        b_mask = mask + 1U;
      }

      crc = crc >> 1U ^ (b_mask & 3988292384U);
    }
  }

  mask = ~crc;

  /*  Output vector */
  out_size[0] = 1;
  out_size[1] = in_size[1] + 4;
  if (0 <= in_size[1] - 1) {
    memcpy(&out_data[0], &in_data[0], (unsigned int)(in_size[1] * (int)sizeof
            (unsigned int)));
  }

  out_data[in_size[1]] = mask & 255U;
  out_data[in_size[1] + 1] = mask >> 8U & 255U;
  out_data[in_size[1] + 2] = mask >> 16U & 255U;
  out_data[in_size[1] + 3] = mask >> 24U;
}

/* End of code generation (crc32.cpp) */
