/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sig_tx.h
 *
 * Code generation for function 'sig_tx'
 *
 */

#ifndef SIG_TX_H
#define SIG_TX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void sig_tx(double PHY_mcs, double PHY_length, const double
                   PHY_pilot_idx[4], const double PHY_pilot_val[4], const double
                   PHY_data_idx[48], double w_beta, creal_T sig_wf[80]);
extern void sig_tx_free();
extern void sig_tx_init();

#endif

/* End of code generation (sig_tx.h) */
