/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * floor1.cpp
 *
 * Code generation for function 'floor1'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "floor1.h"

/* Function Definitions */
void b_floor(double x_data[], int x_size[2])
{
  int nx;
  int k;
  nx = x_size[1];
  for (k = 0; k < nx; k++) {
    x_data[k] = std::floor(x_data[k]);
  }
}

/* End of code generation (floor1.cpp) */
