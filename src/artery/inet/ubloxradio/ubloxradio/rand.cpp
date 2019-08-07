/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rand.cpp
 *
 * Code generation for function 'rand'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "rand.h"
#include "randn.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void b_rand(double r[2])
{
  r[0] = genrandu(state);
  r[1] = genrandu(state);
}

double c_rand()
{
  return genrandu(state);
}

/* End of code generation (rand.cpp) */
