/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * SystemCore.cpp
 *
 * Code generation for function 'SystemCore'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "SystemCore.h"
#include "mpower.h"
#include "ViterbiDecoder.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void SystemCore_reset(commcodegen_ViterbiDecoder_1 *obj)
{
  if (obj->isInitialized == 1) {
    ViterbiDecoder_resetImpl(obj);
  }
}

void SystemCore_step(commcodegen_ViterbiDecoder_1 *obj, const double
                     varargin_1_data[], const int varargin_1_size[1], double
                     varargout_1_data[], int varargout_1_size[2])
{
  comm_ViterbiDecoder_5 *b_obj;
  int varargout_1_size_tmp;
  int ib;
  int ptrIdx;
  int indx1;
  unsigned int minstate;
  double tmpV;
  unsigned int input;
  double b_indx1;
  if (obj->isInitialized != 1) {
    obj->isSetupComplete = false;
    obj->isInitialized = 1;
    obj->isSetupComplete = true;
    ViterbiDecoder_resetImpl(obj);
  }

  b_obj = &obj->cSFunObject;

  /* System object Outputs function: comm.ViterbiDecoder */
  varargout_1_size_tmp = varargin_1_size[0] >> 1;
  varargout_1_size[0] = varargout_1_size_tmp;
  varargout_1_size[1] = 1;
  for (ib = 0; ib < varargout_1_size_tmp; ib++) {
    ptrIdx = ib << 1;

    /* Branch Metric Computation */
    for (indx1 = 0; indx1 < 4; indx1++) {
      b_obj->W0_bMetric[indx1] = 0.0;
      tmpV = varargin_1_data[ptrIdx + 1];
      if ((indx1 & 1) != 0) {
        tmpV++;
      } else {
        tmpV--;
      }

      tmpV = rt_powd_snf(tmpV, 2.0);
      b_obj->W0_bMetric[indx1] += tmpV;
      if ((int)((unsigned int)indx1 >> 1) != 0) {
        b_indx1 = varargin_1_data[ptrIdx] + 1.0;
      } else {
        b_indx1 = varargin_1_data[ptrIdx] - 1.0;
      }

      b_obj->W0_bMetric[indx1] += rt_powd_snf(b_indx1, 2.0);
    }

    /* State Metric Update */
    minstate = (unsigned int)ACS_D_D(64, b_obj->W2_tempMetric, 2,
      b_obj->W0_bMetric, b_obj->W1_stateMetric, b_obj->W3_tbState,
      b_obj->W4_tbInput, (int *)&b_obj->W5_tbPtr, b_obj->P0_StateVec,
      b_obj->P1_OutputVec, 1.7976931348623157E+308);

    /* Traceback decoding */
    ptrIdx = b_obj->W5_tbPtr;
    input = 0U;
    for (indx1 = 0; indx1 < 97; indx1++) {
      minstate += ptrIdx << 6;
      input = b_obj->W4_tbInput[minstate];
      minstate = b_obj->W3_tbState[minstate];
      if (ptrIdx > 0) {
        ptrIdx--;
      } else {
        ptrIdx = 96;
      }
    }

    varargout_1_data[ib] = (int)input & 1;

    /* Increment (mod TbDepth) the traceback index and store */
    if (b_obj->W5_tbPtr < 96) {
      b_obj->W5_tbPtr++;
    } else {
      b_obj->W5_tbPtr = 0;
    }
  }
}

void b_SystemCore_step(commcodegen_ViterbiDecoder_1 *obj, const double
  varargin_1_data[], const int varargin_1_size[1], double varargout_1_data[],
  int varargout_1_size[2])
{
  comm_ViterbiDecoder_5 *b_obj;
  int varargout_1_size_idx_0_tmp;
  int ib;
  int ptrIdx;
  int indx1;
  signed char b_varargout_1_data[216];
  unsigned int minstate;
  double tmpV;
  unsigned int input;
  double b_indx1;
  if (obj->isInitialized != 1) {
    obj->isSetupComplete = false;
    obj->isInitialized = 1;
    obj->isSetupComplete = true;
    ViterbiDecoder_resetImpl(obj);
  }

  b_obj = &obj->cSFunObject;

  /* System object Outputs function: comm.ViterbiDecoder */
  varargout_1_size_idx_0_tmp = varargin_1_size[0] >> 1;
  for (ib = 0; ib < varargout_1_size_idx_0_tmp; ib++) {
    ptrIdx = ib << 1;

    /* Branch Metric Computation */
    for (indx1 = 0; indx1 < 4; indx1++) {
      b_obj->W0_bMetric[indx1] = 0.0;
      tmpV = varargin_1_data[ptrIdx + 1];
      if ((indx1 & 1) != 0) {
        tmpV++;
      } else {
        tmpV--;
      }

      tmpV = rt_powd_snf(tmpV, 2.0);
      b_obj->W0_bMetric[indx1] += tmpV;
      if ((int)((unsigned int)indx1 >> 1) != 0) {
        b_indx1 = varargin_1_data[ptrIdx] + 1.0;
      } else {
        b_indx1 = varargin_1_data[ptrIdx] - 1.0;
      }

      b_obj->W0_bMetric[indx1] += rt_powd_snf(b_indx1, 2.0);
    }

    /* State Metric Update */
    minstate = (unsigned int)ACS_D_D(64, b_obj->W2_tempMetric, 2,
      b_obj->W0_bMetric, b_obj->W1_stateMetric, b_obj->W3_tbState,
      b_obj->W4_tbInput, (int *)&b_obj->W5_tbPtr, b_obj->P0_StateVec,
      b_obj->P1_OutputVec, 1.7976931348623157E+308);

    /* Traceback decoding */
    ptrIdx = b_obj->W5_tbPtr;
    input = 0U;
    for (indx1 = 0; indx1 < 97; indx1++) {
      minstate += ptrIdx << 6;
      input = b_obj->W4_tbInput[minstate];
      minstate = b_obj->W3_tbState[minstate];
      if (ptrIdx > 0) {
        ptrIdx--;
      } else {
        ptrIdx = 96;
      }
    }

    b_varargout_1_data[ib] = (signed char)(input & 1U);

    /* Increment (mod TbDepth) the traceback index and store */
    if (b_obj->W5_tbPtr < 96) {
      b_obj->W5_tbPtr++;
    } else {
      b_obj->W5_tbPtr = 0;
    }
  }

  varargout_1_size[0] = varargout_1_size_idx_0_tmp;
  varargout_1_size[1] = 1;
  for (ptrIdx = 0; ptrIdx < varargout_1_size_idx_0_tmp; ptrIdx++) {
    varargout_1_data[ptrIdx] = b_varargout_1_data[ptrIdx];
  }
}

/* End of code generation (SystemCore.cpp) */
