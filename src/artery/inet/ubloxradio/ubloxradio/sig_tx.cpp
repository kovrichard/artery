/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sig_tx.cpp
 *
 * Code generation for function 'sig_tx'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sig_tx.h"
#include "dot11_ifft.h"
#include "kaiser.h"
#include "mod.h"
#include "step.h"
#include "sim_rx_mex_rtwutil.h"
#include "sim_rx_mex_data.h"

/* Variable Definitions */
static comm_ConvolutionalEncoder_0 bcc_obj;
static boolean_T bcc_obj_not_empty;

/* Function Definitions */
void sig_tx(double PHY_mcs, double PHY_length, const double PHY_pilot_idx[4],
            const double PHY_pilot_val[4], const double PHY_data_idx[48], double
            w_beta, creal_T sig_wf[80])
{
  int i;
  signed char sig_rate_idx_0;
  signed char sig_rate_idx_1;
  signed char sig_rate_idx_2;
  signed char sig_rate_idx_3;
  double binary_length[12];
  double tmp;
  double x[16];
  double sig_rate[24];
  boolean_T sig_msg[24];
  int ib;
  unsigned int b_tmp;
  unsigned int minNxtStIdx;
  boolean_T sig_enc[60];
  signed char sig_int[48];
  double dv6[48];
  double sig_sp[64];
  double dv7[64];
  creal_T sig_wf_base[64];

  /* SIG_TX SIG message transmitter/parser */
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
  /*  Store this as a persistent variable to avoid reinitialization */
  /*  Initialize BCC encoder object for SIG */
  if (!bcc_obj_not_empty) {
    /* System object Constructor function: comm.ConvolutionalEncoder */
    bcc_obj.S0_isInitialized = 0;
    for (i = 0; i < 128; i++) {
      bcc_obj.P0_StateVec[i] = (unsigned int)iv2[i];
      bcc_obj.P1_OutputVec[i] = (unsigned int)iv3[i];
    }

    bcc_obj_not_empty = true;
  } else {
    if (bcc_obj.S0_isInitialized == 1) {
      InitializeConditions(&bcc_obj);
    }
  }

  /*  Select datarate binary code according to MCS */
  switch ((int)PHY_mcs) {
   case 0:
    sig_rate_idx_0 = 1;
    sig_rate_idx_1 = 1;
    sig_rate_idx_2 = 0;
    sig_rate_idx_3 = 1;
    break;

   case 1:
    sig_rate_idx_0 = 1;
    sig_rate_idx_1 = 1;
    sig_rate_idx_2 = 1;
    sig_rate_idx_3 = 1;
    break;

   case 2:
    sig_rate_idx_0 = 0;
    sig_rate_idx_1 = 1;
    sig_rate_idx_2 = 0;
    sig_rate_idx_3 = 1;
    break;

   case 3:
    sig_rate_idx_0 = 0;
    sig_rate_idx_1 = 1;
    sig_rate_idx_2 = 1;
    sig_rate_idx_3 = 1;
    break;

   case 4:
    sig_rate_idx_0 = 1;
    sig_rate_idx_1 = 0;
    sig_rate_idx_2 = 0;
    sig_rate_idx_3 = 1;
    break;

   case 5:
    sig_rate_idx_0 = 1;
    sig_rate_idx_1 = 0;
    sig_rate_idx_2 = 1;
    sig_rate_idx_3 = 1;
    break;

   case 6:
    sig_rate_idx_0 = 0;
    sig_rate_idx_1 = 0;
    sig_rate_idx_2 = 0;
    sig_rate_idx_3 = 1;
    break;

   case 7:
    sig_rate_idx_0 = 0;
    sig_rate_idx_1 = 0;
    sig_rate_idx_2 = 1;
    sig_rate_idx_3 = 1;
    break;

   default:
    /*  Needed for code generation */
    sig_rate_idx_0 = 0;
    sig_rate_idx_1 = 0;
    sig_rate_idx_2 = 0;
    sig_rate_idx_3 = 0;
    break;
  }

  /*  Report error if length exceeds maximum allowed value */
  /*  Convert payload length to binary */
  memset(&binary_length[0], 0, 12U * sizeof(double));
  i = 1;
  tmp = PHY_length;
  while ((i <= 12) && (tmp > 0.0)) {
    binary_length[i - 1] = rt_remd_snf(tmp, 2.0);
    tmp /= 2.0;
    tmp = std::floor(tmp);
    i++;
  }

  /*  Calculate even parity */
  x[0] = sig_rate_idx_0;
  x[1] = sig_rate_idx_1;
  x[2] = sig_rate_idx_2;
  x[3] = sig_rate_idx_3;
  memcpy(&x[4], &binary_length[0], 12U * sizeof(double));
  tmp = x[0];
  for (i = 0; i < 15; i++) {
    tmp += x[i + 1];
  }

  /*  Form SIG uncoded message */
  sig_rate[0] = sig_rate_idx_0;
  sig_rate[1] = sig_rate_idx_1;
  sig_rate[2] = sig_rate_idx_2;
  sig_rate[3] = sig_rate_idx_3;
  sig_rate[4] = 0.0;
  memcpy(&sig_rate[5], &binary_length[0], 12U * sizeof(double));
  sig_rate[17] = b_mod(tmp);
  sig_rate[18] = 0.0;
  sig_rate[19] = 0.0;
  sig_rate[20] = 0.0;
  sig_rate[21] = 0.0;
  sig_rate[22] = 0.0;
  sig_rate[23] = 0.0;
  for (i = 0; i < 24; i++) {
    sig_msg[i] = (sig_rate[i] != 0.0);
  }

  /*  Convolutional encoder */
  if (bcc_obj.S0_isInitialized != 1) {
    bcc_obj.S0_isInitialized = 1;

    /* System object Initialization function: comm.ConvolutionalEncoder */
    bcc_obj.W0_currState = 0U;
  }

  /* System object Outputs function: comm.ConvolutionalEncoder */
  for (ib = 0; ib < 24; ib++) {
    i = (int)(bcc_obj.W0_currState + (sig_msg[ib] << 6));
    b_tmp = bcc_obj.P1_OutputVec[i];
    bcc_obj.W0_currState = bcc_obj.P0_StateVec[i];
    i = ib << 1;
    sig_enc[i + 1] = ((b_tmp & 1U) != 0U);
    b_tmp >>= 1U;
    sig_enc[i] = ((b_tmp & 1U) != 0U);
  }

  i = 49;
  for (ib = 0; ib < 6; ib++) {
    if (bcc_obj.P0_StateVec[bcc_obj.W0_currState] <
        bcc_obj.P0_StateVec[bcc_obj.W0_currState + 64U]) {
      minNxtStIdx = bcc_obj.W0_currState;
    } else {
      minNxtStIdx = bcc_obj.W0_currState + 64U;
    }

    b_tmp = bcc_obj.P1_OutputVec[minNxtStIdx];
    bcc_obj.W0_currState = bcc_obj.P0_StateVec[minNxtStIdx];
    sig_enc[i] = ((b_tmp & 1U) != 0U);
    b_tmp >>= 1U;
    sig_enc[i - 1] = ((b_tmp & 1U) != 0U);
    i += 2;
  }

  /*  Interleaver */
  /* INTERLEAVER Bit interleaver */
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
  /*  Interleaver configuration */
  /*  s-parameter */
  /*  Initialize output */
  for (i = 0; i < 48; i++) {
    sig_int[i] = 0;
  }

  /*  Input index */
  /*  First permutation */
  /*  Second permutation */
  /*  Interleaver mapping */
  c_mod(dv2, dv6);
  for (i = 0; i < 48; i++) {
    sig_int[(int)(((double)iv4[i] + dv6[i]) + 1.0) - 1] = (signed char)sig_enc[i];
  }

  /*  BPSK modulation */
  /*  Initialize base f-domain representation */
  memset(&sig_sp[0], 0, sizeof(double) << 6);

  /*  Map modulated symbols on data subcarriers */
  for (i = 0; i < 48; i++) {
    sig_sp[(int)PHY_data_idx[i] - 1] = 2.0 * (double)sig_int[i] - 1.0;
  }

  /*  Append pilots */
  sig_sp[(int)PHY_pilot_idx[0] - 1] = PHY_pilot_val[0];
  sig_sp[(int)PHY_pilot_idx[1] - 1] = PHY_pilot_val[1];
  sig_sp[(int)PHY_pilot_idx[2] - 1] = PHY_pilot_val[2];
  sig_sp[(int)PHY_pilot_idx[3] - 1] = PHY_pilot_val[3];

  /*  Apply spectral shaping window */
  kaiser(w_beta, dv7);
  for (i = 0; i < 64; i++) {
    sig_sp[i] *= dv7[i];
  }

  /*  Time-domain SIG waveform */
  dot11_ifft(sig_sp, sig_wf_base);
  for (i = 0; i < 64; i++) {
    sig_wf_base[i].re *= 0.13867504905630729;
    sig_wf_base[i].im *= 0.13867504905630729;
  }

  /*  Append CP */
  memcpy(&sig_wf[0], &sig_wf_base[48], sizeof(creal_T) << 4);
  memcpy(&sig_wf[16], &sig_wf_base[0], sizeof(creal_T) << 6);
}

void sig_tx_free()
{
  /* System object Destructor function: comm.ConvolutionalEncoder */
  if (bcc_obj.S0_isInitialized == 1) {
    bcc_obj.S0_isInitialized = 2;
  }
}

void sig_tx_init()
{
  bcc_obj_not_empty = false;
  bcc_obj.S0_isInitialized = 0;
}

/* End of code generation (sig_tx.cpp) */
