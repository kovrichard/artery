/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_tx.cpp
 *
 * Code generation for function 'sim_tx'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sim_rx_mex_emxutil.h"
#include "apply_time_window.h"
#include "data_tx.h"
#include "sig_tx.h"
#include "ltf_tx.h"
#include "stf_tx.h"
#include "tx_phy_params.h"
#include "crc32.h"

/* Function Definitions */
void sim_tx(double mcs, const unsigned int mac_payload_data[], const int
            mac_payload_size[2], const boolean_T pn_seq[7], boolean_T window_en,
            double w_beta, emxArray_creal_T *tx_wf, emxArray_creal_T *data_f_mtx,
            boolean_T data_msg_data[], int data_msg_size[1], struct0_T *PHY)
{
  emxArray_boolean_T *r0;
  unsigned int b_mac_payload_data[4100];
  int b_mac_payload_size[2];
  creal_T stf_wf[160];
  creal_T ltf_wf[160];
  creal_T sig_wf[80];
  double pad_len;
  int i0;
  int loop_ub;
  int b_loop_ub;
  emxArray_creal_T *data_wf;
  emxArray_creal_T *b_stf_wf;
  emxInit_boolean_T(&r0, 1);

  /* SIM_RX High-level transmitter function */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    August 2018; Last revision: 19-February-2019 */
  crc32(mac_payload_data, mac_payload_size, b_mac_payload_data,
        b_mac_payload_size);

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
  /*  Create structure with PHY parameters */
  tx_phy_params(mcs, b_mac_payload_data, b_mac_payload_size, pn_seq, PHY,
                data_msg_data, data_msg_size);

  /*  Get STF waveform */
  stf_tx(w_beta, stf_wf);

  /*  Get LTF waveform */
  ltf_tx(w_beta, ltf_wf);

  /*  Get SIG waveform */
  sig_tx(PHY->mcs, PHY->length, PHY->pilot_idx, PHY->pilot_val, PHY->data_idx,
         w_beta, sig_wf);

  /*  Calculate number of required pad bits */
  pad_len = PHY->n_sym * PHY->n_dbps - ((16.0 + 8.0 * PHY->length) + 6.0);

  /*  Add service and zero-padding (pad + tail) */
  /*  Generate data waveform */
  i0 = r0->size[0];
  loop_ub = (int)(pad_len + 6.0);
  r0->size[0] = (data_msg_size[0] + loop_ub) + 16;
  emxEnsureCapacity_boolean_T(r0, i0);
  for (i0 = 0; i0 < 16; i0++) {
    r0->data[i0] = false;
  }

  b_loop_ub = data_msg_size[0];
  for (i0 = 0; i0 < b_loop_ub; i0++) {
    r0->data[i0 + 16] = data_msg_data[i0];
  }

  for (i0 = 0; i0 < loop_ub; i0++) {
    r0->data[(i0 + data_msg_size[0]) + 16] = false;
  }

  emxInit_creal_T(&data_wf, 1);
  emxInit_creal_T(&b_stf_wf, 1);
  data_tx(PHY->length, PHY->pn_seq, PHY->pilot_idx, PHY->pilot_val,
          PHY->polarity_sign, PHY->data_idx, PHY->r_num, PHY->n_bpscs,
          PHY->n_cbps, PHY->n_dbps, PHY->n_sym, pad_len, r0, w_beta, data_wf,
          data_f_mtx);

  /*  Concatenate output waveform */
  /*  Apply time-domain windowing */
  i0 = b_stf_wf->size[0];
  b_stf_wf->size[0] = 400 + data_wf->size[0];
  emxEnsureCapacity_creal_T(b_stf_wf, i0);
  emxFree_boolean_T(&r0);
  for (i0 = 0; i0 < 160; i0++) {
    b_stf_wf->data[i0] = stf_wf[i0];
    b_stf_wf->data[i0 + 160] = ltf_wf[i0];
  }

  for (i0 = 0; i0 < 80; i0++) {
    b_stf_wf->data[i0 + 320] = sig_wf[i0];
  }

  loop_ub = data_wf->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_stf_wf->data[i0 + 400] = data_wf->data[i0];
  }

  emxFree_creal_T(&data_wf);
  apply_time_window(b_stf_wf, window_en, tx_wf);
  emxFree_creal_T(&b_stf_wf);
}

/* End of code generation (sim_tx.cpp) */
