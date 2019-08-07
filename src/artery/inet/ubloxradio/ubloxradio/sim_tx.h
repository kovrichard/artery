/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_tx.h
 *
 * Code generation for function 'sim_tx'
 *
 */

#ifndef SIM_TX_H
#define SIM_TX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void sim_tx(double mcs, const unsigned int mac_payload_data[], const int
                   mac_payload_size[2], const boolean_T pn_seq[7], boolean_T
                   window_en, double w_beta, emxArray_creal_T *tx_wf,
                   emxArray_creal_T *data_f_mtx, boolean_T data_msg_data[], int
                   data_msg_size[1], struct0_T *PHY);

#endif

/* End of code generation (sim_tx.h) */
