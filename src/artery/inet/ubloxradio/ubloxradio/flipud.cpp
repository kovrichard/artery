/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * flipud.cpp
 *
 * Code generation for function 'flipud'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "flipud.h"

/* Function Definitions */
void flipud(boolean_T x[7])
{
  boolean_T xtmp;
  xtmp = x[0];
  x[0] = x[6];
  x[6] = xtmp;
  xtmp = x[1];
  x[1] = x[5];
  x[5] = xtmp;
  xtmp = x[2];
  x[2] = x[4];
  x[4] = xtmp;
}

/* End of code generation (flipud.cpp) */
