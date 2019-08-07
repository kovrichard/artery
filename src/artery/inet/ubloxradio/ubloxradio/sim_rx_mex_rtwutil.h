/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx_mex_rtwutil.h
 *
 * Code generation for function 'sim_rx_mex_rtwutil'
 *
 */

#ifndef SIM_RX_MEX_RTWUTIL_H
#define SIM_RX_MEX_RTWUTIL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern int ACS_D_D(int numStates, double pTempMetric[], int alpha, const double
                   pBMet[], double pStateMet[], unsigned int pTbState[],
                   unsigned int pTbInput[], const int pTbPtr[], const unsigned
                   int pNxtSt[], const unsigned int pEncOut[], double maxValue);
extern double rt_atan2d_snf(double u0, double u1);
extern double rt_hypotd_snf(double u0, double u1);
extern double rt_powd_snf(double u0, double u1);
extern double rt_remd_snf(double u0, double u1);
extern double rt_roundd_snf(double u);

#endif

/* End of code generation (sim_rx_mex_rtwutil.h) */
