/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xcorr.h
 *
 * Code generation for function 'xcorr'
 *
 */

#ifndef XCORR_H
#define XCORR_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void crosscorr(const creal_T x[64], const emxArray_creal_T *y, double
                      maxlag, emxArray_creal_T *c);
extern void xcorr(const creal_T x[64], const creal_T varargin_1[64], creal_T c
                  [127]);

#endif

/* End of code generation (xcorr.h) */
