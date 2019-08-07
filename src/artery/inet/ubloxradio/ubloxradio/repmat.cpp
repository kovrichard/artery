/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.cpp
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "repmat.h"

/* Function Definitions */
void repmat(const boolean_T a_data[], const int a_size[2], double varargin_2,
            boolean_T b_data[], int b_size[2])
{
  int b_size_tmp;
  int ncols;
  int jtilecol;
  int ibtile;
  int jcol;
  b_size[0] = 1;
  b_size_tmp = (int)varargin_2;
  b_size[1] = (short)(a_size[1] * b_size_tmp);
  ncols = a_size[1];
  for (jtilecol = 0; jtilecol < b_size_tmp; jtilecol++) {
    ibtile = jtilecol * ncols;
    for (jcol = 0; jcol < ncols; jcol++) {
      b_data[ibtile + jcol] = a_data[jcol];
    }
  }
}

/* End of code generation (repmat.cpp) */
