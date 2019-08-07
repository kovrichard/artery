/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sig_rx.h
 *
 * Code generation for function 'sig_rx'
 *
 */

#ifndef SIG_RX_H
#define SIG_RX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void sig_rx(const creal_T r[64], creal_T h_est[64], double
                   *SIG_CFG_length, double *SIG_CFG_mcs, boolean_T
                   *SIG_CFG_sig_err, double *SIG_CFG_r_num, double
                   *SIG_CFG_r_denom, double *SIG_CFG_n_bpscs, double
                   *SIG_CFG_n_cbps, double *SIG_CFG_n_dbps, double
                   *SIG_CFG_n_sym, double *r_cfo);
extern void sig_rx_free();
extern void sig_rx_init();

#endif

/* End of code generation (sig_rx.h) */
