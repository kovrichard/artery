/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx_mex_terminate.cpp
 *
 * Code generation for function 'sim_rx_mex_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sim_rx_mex_terminate.h"
#include "sig_tx.h"
#include "data_tx.h"
#include "sig_rx.h"
#include "bcc_dec.h"

/* Function Definitions */
void sim_rx_mex_terminate()
{
  bcc_dec_free();
  sig_rx_free();
  data_tx_free();
  sig_tx_free();
}

/* End of code generation (sim_rx_mex_terminate.cpp) */
