/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sim_rx_mex_rtwutil.cpp
 *
 * Code generation for function 'sim_rx_mex_rtwutil'
 *
 */

/* Include files */
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <float.h>
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
int ACS_D_D(int numStates, double pTempMetric[], int alpha, const double pBMet[],
            double pStateMet[], unsigned int pTbState[], unsigned int pTbInput[],
            const int pTbPtr[], const unsigned int pNxtSt[], const unsigned int
            pEncOut[], double maxValue)
{
  int minstate;
  double renorm;
  int stateIdx;
  double tmpVal;
  int inpIdx;
  int offsetIdx;
  int nextStateIdx;
  double currMetPlusBranchMet;
  renorm = maxValue;
  for (stateIdx = 0; stateIdx < numStates; stateIdx++) {
    pTempMetric[stateIdx] = maxValue;
  }

  for (stateIdx = 0; stateIdx < numStates; stateIdx++) {
    tmpVal = pStateMet[stateIdx];
    for (inpIdx = 0; inpIdx < alpha; inpIdx++) {
      offsetIdx = inpIdx * numStates + stateIdx;
      nextStateIdx = (int)pNxtSt[offsetIdx];
      currMetPlusBranchMet = tmpVal + pBMet[pEncOut[offsetIdx]];
      if (currMetPlusBranchMet < pTempMetric[nextStateIdx]) {
        offsetIdx = numStates * pTbPtr[0U] + nextStateIdx;
        pTbState[offsetIdx] = (unsigned int)stateIdx;
        pTbInput[offsetIdx] = (unsigned int)inpIdx;
        pTempMetric[nextStateIdx] = currMetPlusBranchMet;
        if (pTempMetric[nextStateIdx] < renorm) {
          renorm = pTempMetric[nextStateIdx];
        }
      }
    }
  }

  /* Update (and renormalize) state metrics, then find  */
  /*  minimum metric state for start of traceback */
  minstate = 0;
  for (stateIdx = 0; stateIdx < numStates; stateIdx++) {
    tmpVal = pTempMetric[stateIdx] - renorm;
    pStateMet[stateIdx] = tmpVal;
    if (tmpVal == 0.0) {
      minstate = stateIdx;
    }
  }

  return minstate;
}

double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = b * std::sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * std::sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = std::abs(u0);
    d1 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = 1.0;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

double rt_remd_snf(double u0, double u1)
{
  double y;
  double b_u1;
  double q;
  if (rtIsNaN(u0) || rtIsInf(u0) || (rtIsNaN(u1) || rtIsInf(u1))) {
    y = rtNaN;
  } else {
    if (u1 < 0.0) {
      b_u1 = std::ceil(u1);
    } else {
      b_u1 = std::floor(u1);
    }

    if ((u1 != 0.0) && (u1 != b_u1)) {
      q = std::abs(u0 / u1);
      if (std::abs(q - std::floor(q + 0.5)) <= DBL_EPSILON * q) {
        y = 0.0 * u0;
      } else {
        y = std::fmod(u0, u1);
      }
    } else {
      y = std::fmod(u0, u1);
    }
  }

  return y;
}

double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* End of code generation (sim_rx_mex_rtwutil.cpp) */
