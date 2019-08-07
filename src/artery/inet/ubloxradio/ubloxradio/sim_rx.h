/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx.h
 *
 * Code generation for function 'sim_rx'
 *
 */

#ifndef SIM_RX_H
#define SIM_RX_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void sim_rx(emxArray_creal_T *rx_wf, double s0_len, double pdet_thold,
                   emxArray_real_T *pld_bytes, double *err);

#endif

/* End of code generation (sim_rx.h) */
