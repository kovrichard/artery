/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xcorr.cpp
 *
 * Code generation for function 'xcorr'
 *
 */

/* Include files */
#include <string.h>
#include <cmath>
#include <math.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "xcorr.h"
#include "ifft.h"
#include "fft1.h"
#include "sim_rx_mex_emxutil.h"
#include "fft.h"
#include "bluesteinSetup.h"
#include "mpower.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Declarations */
static void b_crosscorr(const creal_T x[64], const creal_T y[64], creal_T c[127]);

/* Function Definitions */
static void b_crosscorr(const creal_T x[64], const creal_T y[64], creal_T c[127])
{
  creal_T c1[128];
  creal_T dcv4[128];
  int i;
  creal_T b_c1[128];
  fft(x, c1);
  fft(y, dcv4);
  for (i = 0; i < 128; i++) {
    b_c1[i].re = c1[i].re * dcv4[i].re - c1[i].im * -dcv4[i].im;
    b_c1[i].im = c1[i].re * -dcv4[i].im + c1[i].im * dcv4[i].re;
  }

  b_ifft(b_c1, c1);
  memcpy(&c[0], &c1[65], 63U * sizeof(creal_T));
  memcpy(&c[63], &c1[0], sizeof(creal_T) << 6);
}

void crosscorr(const creal_T x[64], const emxArray_creal_T *y, double maxlag,
               emxArray_creal_T *c)
{
  int m;
  double mxl;
  double tdops;
  int ceilLog2;
  double m2;
  int n;
  int c0;
  int Sn;
  int n1_tmp;
  emxArray_creal_T *X;
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *wwc;
  emxArray_creal_T *fv;
  emxArray_creal_T *r4;
  int i14;
  emxArray_creal_T *b_fv;
  boolean_T useRadix2;
  emxArray_creal_T *Y;
  int j;
  int nRowsM2;
  int nRowsD2;
  int nRowsD4;
  double twid_re;
  double twid_im;
  int i;
  double temp_re;
  double temp_im;
  int temp_re_tmp;
  double fv_im;
  double fv_re;
  double b_fv_im;
  m = y->size[0];
  if (64 > m) {
    m = 64;
  }

  mxl = (double)m - 1.0;
  if (maxlag < mxl) {
    mxl = maxlag;
  }

  tdops = frexp((double)(((unsigned int)m << 1) + MAX_uint32_T), &ceilLog2);
  if (tdops == 0.5) {
    ceilLog2--;
  }

  m2 = rt_powd_snf(2.0, (double)ceilLog2);
  m = y->size[0];
  if (64 > m) {
    m = 64;
  }

  n = y->size[0];
  if (64 < n) {
    n = 64;
  }

  c0 = (n << 3) - 2;
  if (mxl <= (double)n - 1.0) {
    Sn = 63 * (c0 - 256);
    if (63 <= m - n) {
      tdops = (c0 + 63 * c0) + Sn;
    } else {
      tdops = (((double)c0 + (double)(m - n) * (double)c0) + (63.0 - (double)(m
                 - n)) * (4.0 * (double)(((unsigned int)m + n) - 63U) - 6.0)) +
        (double)Sn;
    }
  } else if (mxl <= (double)m - 1.0) {
    Sn = (n - 1) * c0 - ((n - 1) << 2) * n;
    if (mxl <= m - n) {
      tdops = ((double)c0 + mxl * (double)c0) + (double)Sn;
    } else {
      tdops = (((double)c0 + (double)(m - n) * (double)c0) + (mxl - (double)(m -
                 n)) * (4.0 * (((double)m - mxl) + (double)n) - 6.0)) + (double)
        Sn;
    }
  } else {
    tdops = 8.0 * (double)m * (double)n - 2.0 * ((double)((unsigned int)m + n) -
      1.0);
  }

  if (tdops < m2 * (15.0 * (double)ceilLog2 + 6.0)) {
    n = y->size[0];
    m = y->size[0];
    if (64 > m) {
      m = 64;
    }

    tdops = (double)m - 1.0;
    if (mxl < tdops) {
      tdops = mxl;
    }

    i14 = c->size[0];
    Sn = (int)(2.0 * tdops + 1.0);
    c->size[0] = Sn;
    emxEnsureCapacity_creal_T(c, i14);
    for (i14 = 0; i14 < Sn; i14++) {
      c->data[i14].re = 0.0;
      c->data[i14].im = 0.0;
    }

    i14 = (int)(tdops + 1.0);
    for (Sn = 0; Sn < i14; Sn++) {
      nRowsM2 = 64 - Sn;
      if (nRowsM2 >= n) {
        nRowsM2 = n;
      }

      twid_re = 0.0;
      twid_im = 0.0;
      for (i = 0; i < nRowsM2; i++) {
        temp_re = x[(int)((unsigned int)Sn + i)].re;
        temp_im = x[(int)((unsigned int)Sn + i)].im;
        twid_re += y->data[i].re * temp_re + y->data[i].im * temp_im;
        twid_im += y->data[i].re * temp_im - y->data[i].im * temp_re;
      }

      m = (int)((tdops + (double)Sn) + 1.0) - 1;
      c->data[m].re = twid_re;
      c->data[m].im = twid_im;
    }

    i14 = (int)tdops;
    for (Sn = 0; Sn < i14; Sn++) {
      m = (n - Sn) - 1;
      if (64 < m) {
        nRowsM2 = 63;
      } else {
        nRowsM2 = m - 1;
      }

      twid_re = 0.0;
      twid_im = 0.0;
      for (i = 0; i <= nRowsM2; i++) {
        temp_re_tmp = (Sn + i) + 1;
        temp_re = y->data[temp_re_tmp].re;
        temp_im = y->data[temp_re_tmp].im;
        twid_re += temp_re * x[i].re + temp_im * x[i].im;
        twid_im += temp_re * x[i].im - temp_im * x[i].re;
      }

      m = (int)((tdops - (1.0 + (double)Sn)) + 1.0) - 1;
      c->data[m].re = twid_re;
      c->data[m].im = twid_im;
    }
  } else {
    n1_tmp = (int)m2;
    emxInit_creal_T(&X, 1);
    emxInit_real_T(&costab, 2);
    emxInit_real_T(&sintab, 2);
    emxInit_real_T(&sintabinv, 2);
    emxInit_creal_T(&wwc, 1);
    emxInit_creal_T(&fv, 1);
    emxInit_creal_T(&r4, 1);
    emxInit_creal_T(&b_fv, 1);
    if (n1_tmp == 0) {
      X->size[0] = 0;
    } else {
      useRadix2 = ((n1_tmp & (n1_tmp - 1)) == 0);
      get_algo_sizes((int)m2, useRadix2, &c0, &m);
      generate_twiddle_tables(m, useRadix2, costab, sintab, sintabinv);
      if (useRadix2) {
        if (64 < (int)m2) {
          j = 62;
        } else {
          j = (int)m2 - 2;
        }

        nRowsM2 = n1_tmp - 2;
        nRowsD2 = n1_tmp / 2;
        nRowsD4 = nRowsD2 / 2;
        i14 = X->size[0];
        X->size[0] = n1_tmp;
        emxEnsureCapacity_creal_T(X, i14);
        if (n1_tmp > 64) {
          Sn = X->size[0];
          i14 = X->size[0];
          X->size[0] = Sn;
          emxEnsureCapacity_creal_T(X, i14);
          for (i14 = 0; i14 < Sn; i14++) {
            X->data[i14].re = 0.0;
            X->data[i14].im = 0.0;
          }
        }

        ceilLog2 = 0;
        c0 = 0;
        m = 0;
        for (i = 0; i <= j; i++) {
          X->data[m] = x[ceilLog2];
          n = n1_tmp;
          useRadix2 = true;
          while (useRadix2) {
            n >>= 1;
            c0 ^= n;
            useRadix2 = ((c0 & n) == 0);
          }

          m = c0;
          ceilLog2++;
        }

        X->data[m] = x[ceilLog2];
        if (n1_tmp > 1) {
          for (i = 0; i <= nRowsM2; i += 2) {
            temp_re = X->data[i + 1].re;
            temp_im = X->data[i + 1].im;
            X->data[i + 1].re = X->data[i].re - X->data[i + 1].re;
            X->data[i + 1].im = X->data[i].im - X->data[i + 1].im;
            X->data[i].re += temp_re;
            X->data[i].im += temp_im;
          }
        }

        m = 2;
        ceilLog2 = 4;
        c0 = 1 + ((nRowsD4 - 1) << 2);
        while (nRowsD4 > 0) {
          for (i = 0; i < c0; i += ceilLog2) {
            temp_re_tmp = i + m;
            temp_re = X->data[temp_re_tmp].re;
            temp_im = X->data[temp_re_tmp].im;
            X->data[temp_re_tmp].re = X->data[i].re - temp_re;
            X->data[temp_re_tmp].im = X->data[i].im - temp_im;
            X->data[i].re += temp_re;
            X->data[i].im += temp_im;
          }

          Sn = 1;
          for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
            twid_re = costab->data[j];
            twid_im = sintab->data[j];
            i = Sn;
            nRowsM2 = Sn + c0;
            while (i < nRowsM2) {
              temp_re_tmp = i + m;
              temp_re = twid_re * X->data[temp_re_tmp].re - twid_im * X->
                data[temp_re_tmp].im;
              temp_im = twid_re * X->data[temp_re_tmp].im + twid_im * X->
                data[temp_re_tmp].re;
              X->data[temp_re_tmp].re = X->data[i].re - temp_re;
              X->data[temp_re_tmp].im = X->data[i].im - temp_im;
              X->data[i].re += temp_re;
              X->data[i].im += temp_im;
              i += ceilLog2;
            }

            Sn++;
          }

          nRowsD4 /= 2;
          m = ceilLog2;
          ceilLog2 += ceilLog2;
          c0 -= m;
        }
      } else {
        bluesteinSetup(n1_tmp, wwc);
        if ((int)m2 < 64) {
          ceilLog2 = (int)m2 - 1;
        } else {
          ceilLog2 = 63;
        }

        i14 = X->size[0];
        X->size[0] = n1_tmp;
        emxEnsureCapacity_creal_T(X, i14);
        if (n1_tmp > 64) {
          Sn = X->size[0];
          i14 = X->size[0];
          X->size[0] = Sn;
          emxEnsureCapacity_creal_T(X, i14);
          for (i14 = 0; i14 < Sn; i14++) {
            X->data[i14].re = 0.0;
            X->data[i14].im = 0.0;
          }
        }

        m = 0;
        for (Sn = 0; Sn <= ceilLog2; Sn++) {
          temp_re_tmp = (n1_tmp + Sn) - 1;
          temp_re = wwc->data[temp_re_tmp].re;
          temp_im = wwc->data[temp_re_tmp].im;
          X->data[Sn].re = temp_re * x[m].re + temp_im * x[m].im;
          X->data[Sn].im = temp_re * x[m].im - temp_im * x[m].re;
          m++;
        }

        i14 = ceilLog2 + 2;
        for (Sn = i14; Sn <= n1_tmp; Sn++) {
          X->data[Sn - 1].re = 0.0;
          X->data[Sn - 1].im = 0.0;
        }

        b_r2br_r2dit_trig_impl(X, c0, costab, sintab, fv);
        b_r2br_r2dit_trig_impl(wwc, c0, costab, sintab, r4);
        i14 = b_fv->size[0];
        b_fv->size[0] = fv->size[0];
        emxEnsureCapacity_creal_T(b_fv, i14);
        Sn = fv->size[0];
        for (i14 = 0; i14 < Sn; i14++) {
          fv_re = fv->data[i14].re;
          b_fv_im = fv->data[i14].im;
          tdops = r4->data[i14].re;
          twid_re = r4->data[i14].im;
          b_fv->data[i14].re = fv_re * tdops - b_fv_im * twid_re;
          b_fv->data[i14].im = fv_re * twid_re + b_fv_im * tdops;
        }

        r2br_r2dit_trig(b_fv, c0, costab, sintabinv, fv);
        m = 0;
        i14 = wwc->size[0];
        for (Sn = n1_tmp; Sn <= i14; Sn++) {
          tdops = wwc->data[Sn - 1].re;
          fv_re = fv->data[Sn - 1].re;
          twid_re = wwc->data[Sn - 1].im;
          b_fv_im = fv->data[Sn - 1].im;
          twid_im = wwc->data[Sn - 1].re;
          fv_im = fv->data[Sn - 1].im;
          temp_re = wwc->data[Sn - 1].im;
          temp_im = fv->data[Sn - 1].re;
          X->data[m].re = tdops * fv_re + twid_re * b_fv_im;
          X->data[m].im = twid_im * fv_im - temp_re * temp_im;
          m++;
        }
      }
    }

    emxInit_creal_T(&Y, 1);
    if ((y->size[0] == 0) || (n1_tmp == 0)) {
      i14 = Y->size[0];
      Y->size[0] = n1_tmp;
      emxEnsureCapacity_creal_T(Y, i14);
      if (n1_tmp > y->size[0]) {
        Sn = Y->size[0];
        i14 = Y->size[0];
        Y->size[0] = Sn;
        emxEnsureCapacity_creal_T(Y, i14);
        for (i14 = 0; i14 < Sn; i14++) {
          Y->data[i14].re = 0.0;
          Y->data[i14].im = 0.0;
        }
      }
    } else {
      useRadix2 = (((int)m2 & ((int)m2 - 1)) == 0);
      get_algo_sizes((int)m2, useRadix2, &c0, &m);
      generate_twiddle_tables(m, useRadix2, costab, sintab, sintabinv);
      if (useRadix2) {
        b_r2br_r2dit_trig_impl(y, n1_tmp, costab, sintab, Y);
      } else {
        bluesteinSetup(n1_tmp, wwc);
        ceilLog2 = y->size[0];
        if (n1_tmp < ceilLog2) {
          ceilLog2 = n1_tmp;
        }

        i14 = Y->size[0];
        Y->size[0] = n1_tmp;
        emxEnsureCapacity_creal_T(Y, i14);
        if (n1_tmp > y->size[0]) {
          Sn = Y->size[0];
          i14 = Y->size[0];
          Y->size[0] = Sn;
          emxEnsureCapacity_creal_T(Y, i14);
          for (i14 = 0; i14 < Sn; i14++) {
            Y->data[i14].re = 0.0;
            Y->data[i14].im = 0.0;
          }
        }

        m = 0;
        for (Sn = 0; Sn < ceilLog2; Sn++) {
          temp_re_tmp = (n1_tmp + Sn) - 1;
          temp_re = wwc->data[temp_re_tmp].re;
          temp_im = wwc->data[temp_re_tmp].im;
          tdops = y->data[m].re;
          fv_im = y->data[m].im;
          twid_re = y->data[m].im;
          twid_im = y->data[m].re;
          Y->data[Sn].re = temp_re * tdops + temp_im * fv_im;
          Y->data[Sn].im = temp_re * twid_re - temp_im * twid_im;
          m++;
        }

        i14 = ceilLog2 + 1;
        for (Sn = i14; Sn <= n1_tmp; Sn++) {
          Y->data[Sn - 1].re = 0.0;
          Y->data[Sn - 1].im = 0.0;
        }

        b_r2br_r2dit_trig_impl(Y, c0, costab, sintab, fv);
        b_r2br_r2dit_trig_impl(wwc, c0, costab, sintab, r4);
        i14 = b_fv->size[0];
        b_fv->size[0] = fv->size[0];
        emxEnsureCapacity_creal_T(b_fv, i14);
        Sn = fv->size[0];
        for (i14 = 0; i14 < Sn; i14++) {
          fv_re = fv->data[i14].re;
          b_fv_im = fv->data[i14].im;
          tdops = r4->data[i14].re;
          twid_re = r4->data[i14].im;
          b_fv->data[i14].re = fv_re * tdops - b_fv_im * twid_re;
          b_fv->data[i14].im = fv_re * twid_re + b_fv_im * tdops;
        }

        r2br_r2dit_trig(b_fv, c0, costab, sintabinv, fv);
        m = 0;
        i14 = wwc->size[0];
        for (Sn = n1_tmp; Sn <= i14; Sn++) {
          tdops = wwc->data[Sn - 1].re;
          fv_re = fv->data[Sn - 1].re;
          twid_re = wwc->data[Sn - 1].im;
          b_fv_im = fv->data[Sn - 1].im;
          twid_im = wwc->data[Sn - 1].re;
          fv_im = fv->data[Sn - 1].im;
          temp_re = wwc->data[Sn - 1].im;
          temp_im = fv->data[Sn - 1].re;
          Y->data[m].re = tdops * fv_re + twid_re * b_fv_im;
          Y->data[m].im = twid_im * fv_im - temp_re * temp_im;
          m++;
        }
      }
    }

    emxFree_creal_T(&b_fv);
    emxFree_creal_T(&r4);
    emxFree_real_T(&sintabinv);
    emxFree_real_T(&sintab);
    i14 = wwc->size[0];
    wwc->size[0] = X->size[0];
    emxEnsureCapacity_creal_T(wwc, i14);
    Sn = X->size[0];
    for (i14 = 0; i14 < Sn; i14++) {
      tdops = Y->data[i14].re;
      twid_re = -Y->data[i14].im;
      twid_im = X->data[i14].re;
      fv_im = X->data[i14].im;
      wwc->data[i14].re = twid_im * tdops - fv_im * twid_re;
      wwc->data[i14].im = twid_im * twid_re + fv_im * tdops;
    }

    emxFree_creal_T(&Y);
    emxFree_creal_T(&X);
    ifft(wwc, fv);
    emxFree_creal_T(&wwc);
    i14 = costab->size[0] * costab->size[1];
    costab->size[0] = 1;
    Sn = (int)std::floor(mxl - 1.0);
    costab->size[1] = Sn + 1;
    emxEnsureCapacity_real_T(costab, i14);
    for (i14 = 0; i14 <= Sn; i14++) {
      costab->data[i14] = 1.0 + (double)i14;
    }

    m2 -= mxl;
    m = (int)(mxl + 1.0);
    i14 = c->size[0];
    c->size[0] = costab->size[1] + m;
    emxEnsureCapacity_creal_T(c, i14);
    Sn = costab->size[1];
    for (i14 = 0; i14 < Sn; i14++) {
      c->data[i14] = fv->data[(int)(m2 + costab->data[i14]) - 1];
    }

    for (i14 = 0; i14 < m; i14++) {
      c->data[i14 + costab->size[1]] = fv->data[i14];
    }

    emxFree_creal_T(&fv);
    emxFree_real_T(&costab);
  }
}

void xcorr(const creal_T x[64], const creal_T varargin_1[64], creal_T c[127])
{
  b_crosscorr(x, varargin_1, c);
}

/* End of code generation (xcorr.cpp) */
