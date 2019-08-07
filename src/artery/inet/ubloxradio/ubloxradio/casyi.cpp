/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * casyi.cpp
 *
 * Code generation for function 'casyi'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "casyi.h"
#include "kaiser.h"
#include "sqrt.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
int casyi(const creal_T z, double fnu, int kode, creal_T *y)
{
  int nz;
  double cz_re;
  double cz_im;
  double r;
  double dnu2;
  creal_T ck;
  double brm;
  double fdn;
  double ak1_re;
  double ak1_im;
  double ez_re;
  double ez_im;
  double aez;
  double p1_re;
  int inu;
  double bk;
  double sqk;
  double sgn;
  double cs1_re;
  double cs1_im;
  double cs2_re;
  double cs2_im;
  double ak;
  double aa;
  double bb;
  boolean_T errflag;
  boolean_T exitg1;
  double ck_re;
  double ck_im;
  nz = 0;
  if (kode == 2) {
    cz_re = 0.0;
    cz_im = z.im;
    r = 0.0;
  } else {
    cz_re = z.re;
    cz_im = z.im;
    r = z.re;
  }

  if (std::abs(r) > 700.92179369444591) {
    nz = -1;
    y->re = rtNaN;
    y->im = 0.0;
  } else {
    dnu2 = fnu + fnu;
    if (cz_im == 0.0) {
      cz_re = std::exp(cz_re);
      cz_im = 0.0;
    } else if (rtIsInf(cz_im) && rtIsInf(cz_re) && (cz_re < 0.0)) {
      cz_re = 0.0;
      cz_im = 0.0;
    } else {
      r = std::exp(cz_re / 2.0);
      cz_re = r * (r * std::cos(cz_im));
      cz_im = r * (r * std::sin(cz_im));
    }

    if (z.im == 0.0) {
      ck.re = 0.15915494309189535 / z.re;
      ck.im = 0.0;
    } else if (z.re == 0.0) {
      ck.re = 0.0;
      ck.im = -(0.15915494309189535 / z.im);
    } else {
      brm = std::abs(z.re);
      fdn = std::abs(z.im);
      if (brm > fdn) {
        fdn = z.im / z.re;
        r = z.re + fdn * z.im;
        ck.re = (0.15915494309189535 + fdn * 0.0) / r;
        ck.im = (0.0 - fdn * 0.15915494309189535) / r;
      } else if (fdn == brm) {
        if (z.re > 0.0) {
          fdn = 0.5;
        } else {
          fdn = -0.5;
        }

        if (z.im > 0.0) {
          r = 0.5;
        } else {
          r = -0.5;
        }

        ck.re = (0.15915494309189535 * fdn + 0.0 * r) / brm;
        ck.im = (0.0 * fdn - 0.15915494309189535 * r) / brm;
      } else {
        fdn = z.re / z.im;
        r = z.im + fdn * z.re;
        ck.re = fdn * 0.15915494309189535 / r;
        ck.im = (fdn * 0.0 - 0.15915494309189535) / r;
      }
    }

    b_sqrt(&ck);
    ak1_re = ck.re * cz_re - ck.im * cz_im;
    ak1_im = ck.re * cz_im + ck.im * cz_re;
    fdn = 0.0;
    if (dnu2 > 4.7170688552396617E-153) {
      fdn = dnu2 * dnu2;
    }

    ez_re = 8.0 * z.re;
    ez_im = 8.0 * z.im;
    aez = 8.0 * rt_hypotd_snf(z.re, z.im);
    if (z.im != 0.0) {
      inu = (int)fnu;
      r = (fnu - (double)inu) * 3.1415926535897931;
      p1_re = std::sin(r);
      bk = std::cos(r);
      if (z.im < 0.0) {
        bk = -bk;
      }

      if (inu != 0) {
        bk = -bk;
      } else {
        p1_re = -p1_re;
      }
    } else {
      p1_re = 0.0;
      bk = 0.0;
    }

    sqk = fdn - 1.0;
    dnu2 = 2.2204460492503131E-16 / aez * std::abs(fdn - 1.0);
    sgn = 1.0;
    cs1_re = 1.0;
    cs1_im = 0.0;
    cs2_re = 1.0;
    cs2_im = 0.0;
    ck.re = 1.0;
    ck.im = 0.0;
    ak = 0.0;
    aa = 1.0;
    bb = aez;
    cz_re = ez_re;
    cz_im = ez_im;
    errflag = true;
    inu = 0;
    exitg1 = false;
    while ((!exitg1) && (inu < 45)) {
      ck.re *= sqk;
      ck.im *= sqk;
      ck_re = ck.re;
      ck_im = ck.im;
      if (cz_im == 0.0) {
        if (ck_im == 0.0) {
          ck.re = ck_re / cz_re;
          ck.im = 0.0;
        } else if (ck_re == 0.0) {
          ck.re = 0.0;
          ck.im = ck_im / cz_re;
        } else {
          ck.re = ck_re / cz_re;
          ck.im = ck_im / cz_re;
        }
      } else if (cz_re == 0.0) {
        if (ck_re == 0.0) {
          ck.re = ck_im / cz_im;
          ck.im = 0.0;
        } else if (ck_im == 0.0) {
          ck.re = 0.0;
          ck.im = -(ck_re / cz_im);
        } else {
          ck.re = ck_im / cz_im;
          ck.im = -(ck_re / cz_im);
        }
      } else {
        brm = std::abs(cz_re);
        fdn = std::abs(cz_im);
        if (brm > fdn) {
          fdn = cz_im / cz_re;
          r = cz_re + fdn * cz_im;
          ck.re = (ck_re + fdn * ck_im) / r;
          ck.im = (ck_im - fdn * ck_re) / r;
        } else if (fdn == brm) {
          if (cz_re > 0.0) {
            fdn = 0.5;
          } else {
            fdn = -0.5;
          }

          if (cz_im > 0.0) {
            r = 0.5;
          } else {
            r = -0.5;
          }

          ck.re = (ck_re * fdn + ck_im * r) / brm;
          ck.im = (ck_im * fdn - ck_re * r) / brm;
        } else {
          fdn = cz_re / cz_im;
          r = cz_im + fdn * cz_re;
          ck.re = (fdn * ck_re + ck_im) / r;
          ck.im = (fdn * ck_im - ck_re) / r;
        }
      }

      cs2_re += ck.re;
      cs2_im += ck.im;
      sgn = -sgn;
      cs1_re += ck.re * sgn;
      cs1_im += ck.im * sgn;
      cz_re += ez_re;
      cz_im += ez_im;
      aa = aa * std::abs(sqk) / bb;
      bb += aez;
      ak += 8.0;
      sqk -= ak;
      if (aa <= dnu2) {
        errflag = false;
        exitg1 = true;
      } else {
        inu++;
      }
    }

    if (errflag) {
      nz = -2;
    } else {
      if (z.re + z.re < 700.92179369444591) {
        cz_re = -2.0 * z.re;
        cz_im = -2.0 * z.im;
        if (cz_im == 0.0) {
          cz_re = std::exp(cz_re);
          cz_im = 0.0;
        } else if (rtIsInf(cz_im) && rtIsInf(cz_re) && (cz_re < 0.0)) {
          cz_re = 0.0;
          cz_im = 0.0;
        } else {
          r = std::exp(cz_re / 2.0);
          cz_re = r * (r * std::cos(cz_im));
          cz_im = r * (r * std::sin(cz_im));
        }

        r = cz_re * cs2_re - cz_im * cs2_im;
        cz_im = cz_re * cs2_im + cz_im * cs2_re;
        cs1_re += r * p1_re - cz_im * bk;
        cs1_im += r * bk + cz_im * p1_re;
      }

      y->re = cs1_re * ak1_re - cs1_im * ak1_im;
      y->im = cs1_re * ak1_im + cs1_im * ak1_re;
    }
  }

  return nz;
}

/* End of code generation (casyi.cpp) */
