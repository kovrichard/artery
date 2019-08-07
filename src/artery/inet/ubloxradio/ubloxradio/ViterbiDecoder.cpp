/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ViterbiDecoder.cpp
 *
 * Code generation for function 'ViterbiDecoder'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "ViterbiDecoder.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
commcodegen_ViterbiDecoder *ViterbiDecoder_ViterbiDecoder
  (commcodegen_ViterbiDecoder *obj)
{
  commcodegen_ViterbiDecoder *b_obj;
  int i;
  comm_ViterbiDecoder_4 *c_obj;
  b_obj = obj;
  b_obj->isInitialized = 0;

  /* System object Constructor function: comm.ViterbiDecoder */
  for (i = 0; i < 128; i++) {
    b_obj->cSFunObject.P0_StateVec[i] = (unsigned int)iv2[i];
  }

  for (i = 0; i < 128; i++) {
    b_obj->cSFunObject.P1_OutputVec[i] = (unsigned int)iv3[i];
  }

  b_obj->matlabCodegenIsDeleted = false;
  c_obj = &obj->cSFunObject;

  /* System object Initialization function: comm.ViterbiDecoder */
  /* Set state metric for all zeros state equal to zero and all other state metrics equal to max values */
  obj->cSFunObject.W6_maxVal = 1.7976931348623157E+308;
  obj->cSFunObject.W1_stateMetric[0U] = 0.0;
  for (i = 0; i < 63; i++) {
    c_obj->W1_stateMetric[i + 1] = 1.7976931348623157E+308;
  }

  /* Set traceback memory to zero */
  for (i = 0; i < 1600; i++) {
    c_obj->W4_tbInput[i] = 0U;
    c_obj->W3_tbState[i] = 0U;
  }

  obj->cSFunObject.W5_tbPtr = 0;
  return b_obj;
}

void ViterbiDecoder_resetImpl(commcodegen_ViterbiDecoder_1 *obj)
{
  comm_ViterbiDecoder_5 *b_obj;
  int i;
  b_obj = &obj->cSFunObject;

  /* System object Initialization function: comm.ViterbiDecoder */
  /* Set state metric for all zeros state equal to zero and all other state metrics equal to max values */
  obj->cSFunObject.W6_maxVal = 1.7976931348623157E+308;
  obj->cSFunObject.W1_stateMetric[0U] = 0.0;
  for (i = 0; i < 63; i++) {
    b_obj->W1_stateMetric[i + 1] = 1.7976931348623157E+308;
  }

  /* Set traceback memory to zero */
  for (i = 0; i < 6208; i++) {
    b_obj->W4_tbInput[i] = 0U;
    b_obj->W3_tbState[i] = 0U;
  }

  obj->cSFunObject.W5_tbPtr = 0;
}

commcodegen_ViterbiDecoder_1 *b_ViterbiDecoder_ViterbiDecoder
  (commcodegen_ViterbiDecoder_1 *obj)
{
  commcodegen_ViterbiDecoder_1 *b_obj;
  int i22;
  b_obj = obj;
  b_obj->isInitialized = 0;

  /* System object Constructor function: comm.ViterbiDecoder */
  for (i22 = 0; i22 < 128; i22++) {
    b_obj->cSFunObject.P0_StateVec[i22] = (unsigned int)iv2[i22];
  }

  for (i22 = 0; i22 < 128; i22++) {
    b_obj->cSFunObject.P1_OutputVec[i22] = (unsigned int)iv3[i22];
  }

  b_obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

/* End of code generation (ViterbiDecoder.cpp) */
