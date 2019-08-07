/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cuchk.cpp
 *
 * Code generation for function 'cuchk'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "cuchk.h"

/* Function Definitions */
int cuchk(const creal_T y, double ascle)
{
  int nz;
  double yr;
  double yi;
  double smallpart;
  yr = std::abs(y.re);
  yi = std::abs(y.im);
  if (yr > yi) {
    smallpart = yi;
    yi = yr;
  } else {
    smallpart = yr;
  }

  if ((smallpart <= ascle) && (yi < smallpart / 2.2204460492503131E-16)) {
    nz = 1;
  } else {
    nz = 0;
  }

  return nz;
}

/* End of code generation (cuchk.cpp) */
