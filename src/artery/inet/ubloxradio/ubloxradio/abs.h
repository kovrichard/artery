/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * abs.h
 *
 * Code generation for function 'abs'
 *
 */

#ifndef ABS_H
#define ABS_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void b_abs(const creal_T x[127], double y[127]);
extern void c_abs(const double x[48], double y[48]);
extern void d_abs(const double x_data[], const int x_size[2], double y_data[],
                  int y_size[2]);

#endif

/* End of code generation (abs.h) */
