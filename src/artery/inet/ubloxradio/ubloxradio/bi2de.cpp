/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bi2de.cpp
 *
 * Code generation for function 'bi2de'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "bi2de.h"

/* Function Definitions */
void UPDATE_DECIMAL(double *d_i, double pp, double b_ij)
{
  if (b_ij != 0.0) {
    *d_i += pp * b_ij;
  }
}

void bi2de(const double b_data[], const int b_size[2], double dOut_data[], int
           dOut_size[1])
{
  double pp;
  int i7;
  int j;
  int b_j;
  int i8;
  int i;
  pp = 1.0;
  dOut_size[0] = b_size[0];
  if (0 <= b_size[0] - 1) {
    memset(&dOut_data[0], 0, (unsigned int)(b_size[0] * (int)sizeof(double)));
  }

  i7 = (int)((1.0 + (-1.0 - (double)b_size[1])) / -1.0);
  for (j = 0; j < i7; j++) {
    b_j = b_size[1] - j;
    i8 = b_size[0];
    for (i = 0; i < i8; i++) {
      UPDATE_DECIMAL(&dOut_data[i], pp, b_data[i + b_size[0] * (b_j - 1)]);
    }

    pp *= 2.0;
  }
}

/* End of code generation (bi2de.cpp) */
