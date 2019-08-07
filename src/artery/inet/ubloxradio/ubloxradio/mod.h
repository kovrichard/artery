/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mod.h
 *
 * Code generation for function 'mod'
 *
 */

#ifndef MOD_H
#define MOD_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern double b_mod(double x);
extern void c_mod(const double x[48], double r[48]);
extern void d_mod(const double x_data[], const int x_size[2], double r_data[],
                  int r_size[2]);
extern void e_mod(const double x_data[], const int x_size[2], double y, double
                  r_data[], int r_size[2]);

#endif

/* End of code generation (mod.h) */
