/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx_mex_types.h
 *
 * Code generation for function 'sim_tx'
 *
 */

#ifndef SIM_RX_MEX_TYPES_H
#define SIM_RX_MEX_TYPES_H

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
struct comm_ConvolutionalEncoder_0
{
  int S0_isInitialized;
  unsigned int W0_currState;
  unsigned int P0_StateVec[128];
  unsigned int P1_OutputVec[128];
};

struct comm_ViterbiDecoder_4
{
  int S0_isInitialized;
  double W0_bMetric[4];
  double W1_stateMetric[64];
  double W2_tempMetric[64];
  unsigned int W3_tbState[1600];
  unsigned int W4_tbInput[1600];
  int W5_tbPtr;
  double W6_maxVal;
  unsigned int P0_StateVec[128];
  unsigned int P1_OutputVec[128];
};

struct comm_ViterbiDecoder_5
{
  int S0_isInitialized;
  double W0_bMetric[4];
  double W1_stateMetric[64];
  double W2_tempMetric[64];
  unsigned int W3_tbState[6208];
  unsigned int W4_tbInput[6208];
  int W5_tbPtr;
  double W6_maxVal;
  unsigned int P0_StateVec[128];
  unsigned int P1_OutputVec[128];
};

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int isInitialized;
  boolean_T isSetupComplete;
  comm_ViterbiDecoder_4 cSFunObject;
} commcodegen_ViterbiDecoder;

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int isInitialized;
  boolean_T isSetupComplete;
  comm_ViterbiDecoder_5 cSFunObject;
} commcodegen_ViterbiDecoder_1;

struct emxArray_boolean_T
{
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

typedef struct {
  creal_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
} emxArray_creal_T;

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

struct emxArray_uint32_T
{
  unsigned int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

typedef struct {
  double mcs;
  double length;
  boolean_T pn_seq[7];
  double pilot_idx[4];
  double pilot_val[4];
  double polarity_sign[127];
  double pilot_offset;
  double data_idx[48];
  double n_sd;
  double r_num;
  double r_denom;
  double n_bpscs;
  double n_cbps;
  double n_dbps;
  double n_sym;
} struct0_T;

typedef struct {
  double snr;
  boolean_T use_mex;
  boolean_T apply_cfo;
} struct1_T;

#endif

/* End of code generation (sim_rx_mex_types.h) */
