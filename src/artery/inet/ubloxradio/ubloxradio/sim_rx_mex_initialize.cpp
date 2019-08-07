/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx_mex_initialize.cpp
 *
 * Code generation for function 'sim_rx_mex_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sim_rx_mex_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "bcc_dec.h"
#include "sig_rx.h"
#include "data_tx.h"
#include "sig_tx.h"
#include "stf_tx.h"
#include "ltf_tx.h"

/* Function Definitions */
void sim_rx_mex_initialize()
{
  rt_InitInfAndNaN(8U);
  ltf_wf_base_not_empty_init();
  stf_wf_base_not_empty_init();
  sig_tx_init();
  data_tx_init();
  sig_rx_init();
  bcc_dec_init();
  c_eml_rand_mt19937ar_stateful_i();
}

/* End of code generation (sim_rx_mex_initialize.cpp) */
