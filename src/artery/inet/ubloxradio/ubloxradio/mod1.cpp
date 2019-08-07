/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mod1.cpp
 *
 * Code generation for function 'mod1'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "mod1.h"

/* Function Definitions */
double f_mod(double x, double y)
{
  double r;
  boolean_T rEQ0;
  double q;
  if ((!rtIsInf(x)) && (!rtIsNaN(x)) && ((!rtIsInf(y)) && (!rtIsNaN(y)))) {
    if (x == 0.0) {
      r = 0.0;
    } else {
      r = std::fmod(x, y);
      rEQ0 = (r == 0.0);
      if ((!rEQ0) && (y > std::floor(y))) {
        q = std::abs(x / y);
        rEQ0 = (std::abs(q - std::floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        r = 0.0;
      } else {
        if (x < 0.0) {
          r += y;
        }
      }
    }
  } else {
    r = rtNaN;
  }

  return r;
}

/* End of code generation (mod1.cpp) */
