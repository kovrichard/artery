/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sign.cpp
 *
 * Code generation for function 'sign'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sign.h"

/* Function Definitions */
void b_sign(double x_data[], int x_size[2])
{
  int nx;
  int k;
  double x;
  nx = x_size[1];
  for (k = 0; k < nx; k++) {
    x = x_data[k];
    if (x_data[k] < 0.0) {
      x = -1.0;
    } else if (x_data[k] > 0.0) {
      x = 1.0;
    } else {
      if (x_data[k] == 0.0) {
        x = 0.0;
      }
    }

    x_data[k] = x;
  }
}

/* End of code generation (sign.cpp) */
