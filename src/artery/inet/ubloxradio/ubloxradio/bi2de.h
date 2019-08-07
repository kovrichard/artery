/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bi2de.h
 *
 * Code generation for function 'bi2de'
 *
 */

#ifndef BI2DE_H
#define BI2DE_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void UPDATE_DECIMAL(double *d_i, double pp, double b_ij);
extern void bi2de(const double b_data[], const int b_size[2], double dOut_data[],
                  int dOut_size[1]);

#endif

/* End of code generation (bi2de.h) */
