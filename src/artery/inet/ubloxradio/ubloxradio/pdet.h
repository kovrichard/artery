/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * pdet.h
 *
 * Code generation for function 'pdet'
 *
 */

#ifndef PDET_H
#define PDET_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void pdet(const emxArray_creal_T *in, double s0_len, double pdet_thold,
                 double *idx, double c_cfo_data[], int c_cfo_size[2], boolean_T *
                 err);

#endif

/* End of code generation (pdet.h) */
