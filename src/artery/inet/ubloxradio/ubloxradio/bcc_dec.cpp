/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bcc_dec.cpp
 *
 * Code generation for function 'bcc_dec'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "bcc_dec.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void bcc_dec_free()
{
  if (!vit_dec.matlabCodegenIsDeleted) {
    vit_dec.matlabCodegenIsDeleted = true;
    if (vit_dec.isInitialized == 1) {
      vit_dec.isInitialized = 2;
    }
  }
}

void bcc_dec_init()
{
  vit_dec_not_empty = false;
  vit_dec.matlabCodegenIsDeleted = true;
}

/* End of code generation (bcc_dec.cpp) */
