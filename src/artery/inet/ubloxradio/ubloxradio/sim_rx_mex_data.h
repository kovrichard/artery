/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx_mex_data.h
 *
 * Code generation for function 'sim_rx_mex_data'
 *
 */

#ifndef SIM_RX_MEX_DATA_H
#define SIM_RX_MEX_DATA_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Variable Declarations */
extern creal_T stf_wf_base[64];
extern boolean_T stf_wf_base_not_empty;
extern unsigned int state[625];
extern commcodegen_ViterbiDecoder_1 vit_dec;
extern boolean_T vit_dec_not_empty;
extern const double dv0[33];
extern const double dv1[33];
extern const signed char iv0[64];
extern const signed char iv1[64];
extern const signed char iv2[128];
extern const signed char iv3[128];
extern const double dv2[48];
extern const signed char iv4[48];
extern const double dv3[33];
extern const signed char iv5[64];
extern const double dv4[65];

#endif

/* End of code generation (sim_rx_mex_data.h) */
