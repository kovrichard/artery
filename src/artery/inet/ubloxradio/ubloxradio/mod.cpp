/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mod.cpp
 *
 * Code generation for function 'mod'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "mod.h"
#include "mod1.h"

/* Function Definitions */
double b_mod(double x)
{
  double r;
  if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    if (x == 0.0) {
      r = 0.0;
    } else {
      r = std::fmod(x, 2.0);
      if (r == 0.0) {
        r = 0.0;
      } else {
        if (x < 0.0) {
          r += 2.0;
        }
      }
    }
  } else {
    r = rtNaN;
  }

  return r;
}

void c_mod(const double x[48], double r[48])
{
  int k;
  double b_r;
  for (k = 0; k < 48; k++) {
    if ((!rtIsInf(x[k])) && (!rtIsNaN(x[k]))) {
      b_r = std::fmod(x[k], 1.0);
      if (b_r == 0.0) {
        b_r = 0.0;
      }
    } else {
      b_r = rtNaN;
    }

    r[k] = b_r;
  }
}

void d_mod(const double x_data[], const int x_size[2], double r_data[], int
           r_size[2])
{
  int nx;
  int k;
  double r;
  r_size[0] = 1;
  r_size[1] = (short)x_size[1];
  nx = (short)x_size[1];
  for (k = 0; k < nx; k++) {
    if ((!rtIsInf(x_data[k])) && (!rtIsNaN(x_data[k]))) {
      if (x_data[k] == 0.0) {
        r = 0.0;
      } else {
        r = std::fmod(x_data[k], 16.0);
        if (r == 0.0) {
          r = 0.0;
        }
      }
    } else {
      r = rtNaN;
    }

    r_data[k] = r;
  }
}

void e_mod(const double x_data[], const int x_size[2], double y, double r_data[],
           int r_size[2])
{
  int nx;
  int k;
  r_size[0] = 1;
  r_size[1] = (short)x_size[1];
  nx = (short)x_size[1];
  for (k = 0; k < nx; k++) {
    r_data[k] = f_mod(x_data[k], y);
  }
}

/* End of code generation (mod.cpp) */
