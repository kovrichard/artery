/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * de2bi.cpp
 *
 * Code generation for function 'de2bi'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "de2bi.h"
#include "sig_tx.h"
#include "sim_rx_mex_emxutil.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void de2bi(const unsigned int d_data[], const int d_size[2], emxArray_uint32_T
           *b)
{
  int i4;
  int loop_ub;
  int j;
  double tmp;
  i4 = b->size[0] * b->size[1];
  b->size[0] = d_size[1];
  b->size[1] = 8;
  emxEnsureCapacity_uint32_T(b, i4);
  loop_ub = d_size[1] << 3;
  for (i4 = 0; i4 < loop_ub; i4++) {
    b->data[i4] = 0U;
  }

  i4 = d_size[1];
  for (loop_ub = 0; loop_ub < i4; loop_ub++) {
    j = 1;
    tmp = d_data[loop_ub];
    while ((j <= 8) && (tmp > 0.0)) {
      b->data[loop_ub + b->size[0] * (j - 1)] = (unsigned int)rt_remd_snf(tmp,
        2.0);
      tmp /= 2.0;
      tmp = std::floor(tmp);
      j++;
    }
  }
}

/* End of code generation (de2bi.cpp) */
