/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fft.h
 *
 * Code generation for function 'fft'
 *
 */

#ifndef FFT_H
#define FFT_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void b_r2br_r2dit_trig_impl(const emxArray_creal_T *x, int unsigned_nRows,
  const emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T
  *y);
extern void generate_twiddle_tables(int nRows, boolean_T useRadix2,
  emxArray_real_T *costab, emxArray_real_T *sintab, emxArray_real_T *sintabinv);
extern void get_algo_sizes(int n1, boolean_T useRadix2, int *N2blue, int *nRows);
extern void r2br_r2dit_trig(const emxArray_creal_T *x, int n1_unsigned, const
  emxArray_real_T *costab, const emxArray_real_T *sintab, emxArray_creal_T *y);
extern void r2br_r2dit_trig_impl(const creal_T x[64], const double costab[33],
  const double sintab[33], creal_T y[64]);

#endif

/* End of code generation (fft.h) */
