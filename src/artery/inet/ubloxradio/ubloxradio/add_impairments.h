/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * add_impairments.h
 *
 * Code generation for function 'add_impairments'
 *
 */

#ifndef ADD_IMPAIRMENTS_H
#define ADD_IMPAIRMENTS_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void add_impairments(const struct1_T *SIM, emxArray_creal_T *tx_wf,
  emxArray_creal_T *rx_wf, double *s0_len);

#endif

/* End of code generation (add_impairments.h) */
