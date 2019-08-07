/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * step.cpp
 *
 * Code generation for function 'step'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "step.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
comm_ConvolutionalEncoder_0 *Constructor(comm_ConvolutionalEncoder_0 *obj)
{
  comm_ConvolutionalEncoder_0 *b_obj;
  int i23;

  /* System object Constructor function: comm.ConvolutionalEncoder */
  b_obj = obj;
  b_obj->S0_isInitialized = 0;
  for (i23 = 0; i23 < 128; i23++) {
    b_obj->P0_StateVec[i23] = (unsigned int)iv2[i23];
  }

  for (i23 = 0; i23 < 128; i23++) {
    b_obj->P1_OutputVec[i23] = (unsigned int)iv3[i23];
  }

  return b_obj;
}

void InitializeConditions(comm_ConvolutionalEncoder_0 *obj)
{
  /* System object Initialization function: comm.ConvolutionalEncoder */
  obj->W0_currState = 0U;
}

/* End of code generation (step.cpp) */
