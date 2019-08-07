/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * tx_phy_params.cpp
 *
 * Code generation for function 'tx_phy_params'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "tx_phy_params.h"
#include "sim_rx_mex_emxutil.h"
#include "de2bi.h"

/* Function Definitions */
void tx_phy_params(double mcs, const unsigned int mac_payload_data[], const int
                   mac_payload_size[2], const boolean_T pn_seq[7], struct0_T
                   *PHY, boolean_T data_msg_data[], int data_msg_size[1])
{
  int i;
  static const signed char iv6[127] = { 1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1,
    -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1,
    -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1,
    1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1,
    -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1, -1, -1,
    -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, 1, -1,
    -1, -1, -1, -1, -1, -1 };

  emxArray_uint32_T *r1;
  static const signed char iv7[48] = { 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 31, 32, 34, 35, 36, 37, 38, 39,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 55, 56, 57, 58, 59 };

  static const signed char iv8[8] = { 1, 3, 1, 3, 1, 3, 2, 3 };

  static const signed char iv9[8] = { 2, 4, 2, 4, 2, 4, 3, 4 };

  static const signed char iv10[8] = { 1, 1, 2, 2, 4, 4, 6, 6 };

  double PHY_tmp;
  int loop_ub;
  int i2;
  int i3;
  boolean_T b_data_msg_data[32800];

  /* TX_PHY_PARAMS Initializes PHY layer parameters */
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
  /*  Store MCS / payload length */
  PHY->mcs = mcs;
  PHY->length = mac_payload_size[1];

  /*  Initialize scrambler with a 7-bit non-allzero PN sequence (random or pre-set) */
  for (i = 0; i < 7; i++) {
    PHY->pn_seq[i] = pn_seq[i];
  }

  /*  Pilot subcarrier indices and values */
  PHY->pilot_idx[0] = 12.0;
  PHY->pilot_val[0] = 1.0;
  PHY->pilot_idx[1] = 26.0;
  PHY->pilot_val[1] = 1.0;
  PHY->pilot_idx[2] = 40.0;
  PHY->pilot_val[2] = 1.0;
  PHY->pilot_idx[3] = 54.0;
  PHY->pilot_val[3] = -1.0;

  /*  Polarity signs to use for pilots */
  for (i = 0; i < 127; i++) {
    PHY->polarity_sign[i] = iv6[i];
  }

  /*  Starting index for pilot polarity index */
  PHY->pilot_offset = 1.0;

  /*  Data subcarrier indices */
  for (i = 0; i < 48; i++) {
    PHY->data_idx[i] = iv7[i];
  }

  emxInit_uint32_T(&r1, 2);

  /*  Number of data subcarriers */
  PHY->n_sd = 48.0;

  /*  MCS tables for coding rate (numerator / denominator) and bits per modulation symbol */
  /*  Find code rate numerator/denominator & bits per modulation symbol */
  i = (int)(mcs + 1.0) - 1;
  PHY->r_num = iv8[i];
  PHY->r_denom = iv9[i];
  PHY->n_bpscs = iv10[i];

  /*  Calculate coded/uncoded number of bits per OFDM symbol */
  PHY->n_cbps = 48.0 * (double)iv10[(int)(mcs + 1.0) - 1];
  PHY_tmp = (double)(48 * iv10[(int)(mcs + 1.0) - 1] * iv8[(int)(mcs + 1.0) - 1])
    / (double)iv9[(int)(mcs + 1.0) - 1];
  PHY->n_dbps = PHY_tmp;

  /*  Calculate number of OFDM symbols */
  PHY->n_sym = std::ceil(((16.0 + 8.0 * (double)mac_payload_size[1]) + 6.0) /
    PHY_tmp);

  /*  Convert byte to binary data */
  de2bi(mac_payload_data, mac_payload_size, r1);
  i = r1->size[0];
  loop_ub = r1->size[0];
  for (i2 = 0; i2 < loop_ub; i2++) {
    for (i3 = 0; i3 < 8; i3++) {
      b_data_msg_data[i3 + (i2 << 3)] = (r1->data[i2 + r1->size[0] * i3] != 0U);
    }
  }

  emxFree_uint32_T(&r1);
  i <<= 3;
  data_msg_size[0] = i;
  if (0 <= i - 1) {
    memcpy(&data_msg_data[0], &b_data_msg_data[0], (unsigned int)(i * (int)
            sizeof(boolean_T)));
  }
}

/* End of code generation (tx_phy_params.cpp) */
