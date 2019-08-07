/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ifft.h
 *
 * Code generation for function 'ifft'
 *
 */

#ifndef IFFT_H
#define IFFT_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void b_ifft(const creal_T x[128], creal_T y[128]);
extern void ifft(const emxArray_creal_T *x, emxArray_creal_T *y);

#endif

/* End of code generation (ifft.h) */
