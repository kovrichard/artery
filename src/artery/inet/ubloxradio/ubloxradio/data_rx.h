/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * data_rx.h
 *
 * Code generation for function 'data_rx'
 *
 */

#ifndef DATA_RX_H
#define DATA_RX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void data_rx(const double PHY_pilot_idx[4], const double
                    PHY_polarity_sign[127], const double PHY_data_idx[48],
                    double SIG_CFG_r_num, double SIG_CFG_n_bpscs, double
                    SIG_CFG_n_cbps, double SIG_CFG_n_dbps, double SIG_CFG_n_sym,
                    const emxArray_creal_T *rx_wf, double idx, const creal_T
                    h_est[64], double r_cfo, emxArray_boolean_T *descr_msg);

#endif

/* End of code generation (data_rx.h) */
