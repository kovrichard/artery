/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * data_tx.h
 *
 * Code generation for function 'data_tx'
 *
 */

#ifndef DATA_TX_H
#define DATA_TX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void data_tx(double PHY_length, const boolean_T PHY_pn_seq[7], const
                    double PHY_pilot_idx[4], const double PHY_pilot_val[4],
                    const double PHY_polarity_sign[127], const double
                    PHY_data_idx[48], double PHY_r_num, double PHY_n_bpscs,
                    double PHY_n_cbps, double PHY_n_dbps, double PHY_n_sym,
                    double pad_len, const emxArray_boolean_T *padding_out,
                    double w_beta, emxArray_creal_T *data_wf, emxArray_creal_T
                    *data_f_mtx);
extern void data_tx_free();
extern void data_tx_init();

#endif

/* End of code generation (data_tx.h) */
