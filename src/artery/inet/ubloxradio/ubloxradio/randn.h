/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * randn.h
 *
 * Code generation for function 'randn'
 *
 */

#ifndef RANDN_H
#define RANDN_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern double genrandu(unsigned int mt[625]);
extern void randn(const double varargin_1[2], emxArray_real_T *r);

#endif

/* End of code generation (randn.h) */
