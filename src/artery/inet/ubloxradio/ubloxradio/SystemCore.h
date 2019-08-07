/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SystemCore.h
 *
 * Code generation for function 'SystemCore'
 *
 */

#ifndef SYSTEMCORE_H
#define SYSTEMCORE_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void SystemCore_reset(commcodegen_ViterbiDecoder_1 *obj);
extern void SystemCore_step(commcodegen_ViterbiDecoder_1 *obj, const double
  varargin_1_data[], const int varargin_1_size[1], double varargout_1_data[],
  int varargout_1_size[2]);
extern void b_SystemCore_step(commcodegen_ViterbiDecoder_1 *obj, const double
  varargin_1_data[], const int varargin_1_size[1], double varargout_1_data[],
  int varargout_1_size[2]);

#endif

/* End of code generation (SystemCore.h) */
