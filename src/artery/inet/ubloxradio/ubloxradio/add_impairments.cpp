/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * add_impairments.cpp
 *
 * Code generation for function 'add_impairments'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "sim_rx_mex_emxutil.h"
#include "mpower.h"
#include "randn.h"
#include "rand.h"
#include "apply_cfo.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void add_impairments(const struct1_T *SIM, emxArray_creal_T *tx_wf,
                     emxArray_creal_T *rx_wf, double *s0_len)
{
  double a[2];
  double x;
  emxArray_creal_T *b_tx_wf;
  int i10;
  int loop_ub;
  emxArray_real_T *r2;
  emxArray_real_T *b;
  emxArray_real_T *r3;
  emxArray_creal_T *noise;
  double b_a;
  double re;
  double im;

  /*  Add CFO error, assume [-5, 5] ppm per Tx/Rx device */
  if (SIM->apply_cfo) {
    b_rand(a);
    a[0] = a[0] * 10.0 - 5.0;
    a[1] = a[1] * 10.0 - 5.0;
    emxInit_creal_T(&b_tx_wf, 1);
    i10 = b_tx_wf->size[0];
    b_tx_wf->size[0] = tx_wf->size[0];
    emxEnsureCapacity_creal_T(b_tx_wf, i10);
    loop_ub = tx_wf->size[0];
    for (i10 = 0; i10 < loop_ub; i10++) {
      b_tx_wf->data[i10] = tx_wf->data[i10];
    }

    apply_cfo(b_tx_wf, (a[0] + a[1]) * 1.0E-6, tx_wf);
    emxFree_creal_T(&b_tx_wf);
  }

  /*  Append silence samples at the beginning/end of useful waveform */
  *s0_len = c_rand();
  x = std::floor(*s0_len * 101.0);
  i10 = rx_wf->size[0];
  rx_wf->size[0] = ((int)(100.0 + x) + tx_wf->size[0]) + (int)(300.0 - (100.0 +
    x));
  emxEnsureCapacity_creal_T(rx_wf, i10);
  loop_ub = (int)(100.0 + x);
  for (i10 = 0; i10 < loop_ub; i10++) {
    rx_wf->data[i10].re = 0.0;
    rx_wf->data[i10].im = 0.0;
  }

  loop_ub = tx_wf->size[0];
  for (i10 = 0; i10 < loop_ub; i10++) {
    rx_wf->data[i10 + (int)(100.0 + x)] = tx_wf->data[i10];
  }

  loop_ub = (int)(300.0 - (100.0 + x));
  for (i10 = 0; i10 < loop_ub; i10++) {
    rx_wf->data[(i10 + (int)(100.0 + x)) + tx_wf->size[0]].re = 0.0;
    rx_wf->data[(i10 + (int)(100.0 + x)) + tx_wf->size[0]].im = 0.0;
  }

  emxInit_real_T(&r2, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&r3, 2);

  /*  Add AWGN noise */
  a[0] = rx_wf->size[0];
  a[1] = 1.0;
  randn(a, r2);
  a[0] = rx_wf->size[0];
  a[1] = 1.0;
  randn(a, r3);
  i10 = b->size[0] * b->size[1];
  b->size[0] = r3->size[0];
  b->size[1] = 1;
  emxEnsureCapacity_real_T(b, i10);
  loop_ub = r3->size[0] * r3->size[1];
  for (i10 = 0; i10 < loop_ub; i10++) {
    b->data[i10] = r3->data[i10];
  }

  emxFree_real_T(&r3);
  emxInit_creal_T(&noise, 2);
  b_a = std::sqrt(1.0 / rt_powd_snf(10.0, SIM->snr / 10.0) / 2.0);
  i10 = noise->size[0] * noise->size[1];
  noise->size[0] = r2->size[0];
  noise->size[1] = 1;
  emxEnsureCapacity_creal_T(noise, i10);
  loop_ub = r2->size[0] * r2->size[1];
  for (i10 = 0; i10 < loop_ub; i10++) {
    re = r2->data[i10] + 0.0 * b->data[i10];
    im = b->data[i10];
    noise->data[i10].re = b_a * re;
    noise->data[i10].im = b_a * im;
  }

  emxFree_real_T(&b);
  emxFree_real_T(&r2);
  i10 = rx_wf->size[0];
  emxEnsureCapacity_creal_T(rx_wf, i10);
  loop_ub = rx_wf->size[0];
  for (i10 = 0; i10 < loop_ub; i10++) {
    rx_wf->data[i10].re += noise->data[i10].re;
    rx_wf->data[i10].im += noise->data[i10].im;
  }

  emxFree_creal_T(&noise);
  *s0_len = 100.0 + x;
}

/* End of code generation (add_impairments.cpp) */
