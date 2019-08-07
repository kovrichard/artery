/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cmlri.cpp
 *
 * Code generation for function 'cmlri'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "cmlri.h"
#include "kaiser.h"
#include "gammaln.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
int cmlri(const creal_T z, double fnu, int kode, creal_T *y)
{
  int nz;
  double az;
  double flooraz;
  int iaz;
  double ck_re;
  double brm;
  double ck_im;
  double rho2;
  double rz_re;
  double rz_im;
  double ack;
  double p1_re;
  double p1_im;
  double p2_re;
  double p2_im;
  double tst;
  double ak;
  boolean_T earlyExit;
  int icounter;
  int i;
  boolean_T exitg1;
  int kcounter;
  double pt_re;
  boolean_T guard1 = false;
  double pt_im;
  int itime;
  double s_im;
  nz = 0;
  az = rt_hypotd_snf(z.re, z.im);
  flooraz = std::floor(az);
  iaz = (int)flooraz;
  if (z.im == 0.0) {
    ck_re = (flooraz + 1.0) / z.re;
    ck_im = 0.0;
    rz_re = 2.0 / z.re;
    rz_im = 0.0;
  } else {
    if (z.re == 0.0) {
      ck_re = 0.0;
      ck_im = -((flooraz + 1.0) / z.im);
    } else {
      brm = std::abs(z.re);
      rho2 = std::abs(z.im);
      if (brm > rho2) {
        rho2 = z.im / z.re;
        ack = z.re + rho2 * z.im;
        ck_re = ((flooraz + 1.0) + rho2 * 0.0) / ack;
        ck_im = (0.0 - rho2 * (flooraz + 1.0)) / ack;
      } else if (rho2 == brm) {
        if (z.re > 0.0) {
          rho2 = 0.5;
        } else {
          rho2 = -0.5;
        }

        if (z.im > 0.0) {
          ack = 0.5;
        } else {
          ack = -0.5;
        }

        ck_re = ((flooraz + 1.0) * rho2 + 0.0 * ack) / brm;
        ck_im = (0.0 * rho2 - (flooraz + 1.0) * ack) / brm;
      } else {
        rho2 = z.re / z.im;
        ack = z.im + rho2 * z.re;
        ck_re = rho2 * (flooraz + 1.0) / ack;
        ck_im = (rho2 * 0.0 - (flooraz + 1.0)) / ack;
      }
    }

    if (z.re == 0.0) {
      rz_re = 0.0;
      rz_im = -(2.0 / z.im);
    } else {
      brm = std::abs(z.re);
      rho2 = std::abs(z.im);
      if (brm > rho2) {
        rho2 = z.im / z.re;
        ack = z.re + rho2 * z.im;
        rz_re = (2.0 + rho2 * 0.0) / ack;
        rz_im = (0.0 - rho2 * 2.0) / ack;
      } else if (rho2 == brm) {
        if (z.re > 0.0) {
          rho2 = 0.5;
        } else {
          rho2 = -0.5;
        }

        if (z.im > 0.0) {
          ack = 0.5;
        } else {
          ack = -0.5;
        }

        rz_re = (2.0 * rho2 + 0.0 * ack) / brm;
        rz_im = (0.0 * rho2 - 2.0 * ack) / brm;
      } else {
        rho2 = z.re / z.im;
        ack = z.im + rho2 * z.re;
        rz_re = rho2 * 2.0 / ack;
        rz_im = (rho2 * 0.0 - 2.0) / ack;
      }
    }
  }

  p1_re = 0.0;
  p1_im = 0.0;
  p2_re = 1.0;
  p2_im = 0.0;
  ack = ((flooraz + 1.0) + 1.0) / az;
  ack += std::sqrt(ack * ack - 1.0);
  rho2 = ack * ack;
  tst = (rho2 + rho2) / ((rho2 - 1.0) * (ack - 1.0)) / 2.2204460492503131E-16;
  ak = flooraz + 1.0;
  earlyExit = true;
  icounter = 1;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 80)) {
    icounter++;
    pt_re = p2_re;
    pt_im = p2_im;
    flooraz = ck_re * p2_re - ck_im * p2_im;
    rho2 = ck_re * p2_im + ck_im * p2_re;
    p2_re = p1_re - flooraz;
    p2_im = p1_im - rho2;
    p1_re = pt_re;
    p1_im = pt_im;
    ck_re += rz_re;
    ck_im += rz_im;
    if (rt_hypotd_snf(p2_re, p2_im) > tst * ak * ak) {
      earlyExit = false;
      exitg1 = true;
    } else {
      ak++;
      i++;
    }
  }

  if (earlyExit) {
    nz = -2;
  } else {
    kcounter = 1;
    guard1 = false;
    if (0 >= iaz) {
      p1_re = 0.0;
      p1_im = 0.0;
      p2_re = 1.0;
      p2_im = 0.0;
      if (z.im == 0.0) {
        ck_re = 1.0 / z.re;
        ck_im = 0.0;
      } else if (z.re == 0.0) {
        ck_re = 0.0;
        ck_im = -(1.0 / z.im);
      } else {
        brm = std::abs(z.re);
        rho2 = std::abs(z.im);
        if (brm > rho2) {
          rho2 = z.im / z.re;
          ack = z.re + rho2 * z.im;
          ck_re = (1.0 + rho2 * 0.0) / ack;
          ck_im = (0.0 - rho2) / ack;
        } else if (rho2 == brm) {
          if (z.re > 0.0) {
            rho2 = 0.5;
          } else {
            rho2 = -0.5;
          }

          if (z.im > 0.0) {
            ack = 0.5;
          } else {
            ack = -0.5;
          }

          ck_re = (rho2 + 0.0 * ack) / brm;
          ck_im = (0.0 * rho2 - ack) / brm;
        } else {
          rho2 = z.re / z.im;
          ack = z.im + rho2 * z.re;
          ck_re = rho2 / ack;
          ck_im = (rho2 * 0.0 - 1.0) / ack;
        }
      }

      tst = std::sqrt(1.0 / az / 2.2204460492503131E-16);
      itime = 1;
      earlyExit = true;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 80)) {
        kcounter++;
        pt_re = p2_re;
        pt_im = p2_im;
        flooraz = ck_re * p2_re - ck_im * p2_im;
        rho2 = ck_re * p2_im + ck_im * p2_re;
        p2_re = p1_re - flooraz;
        p2_im = p1_im - rho2;
        p1_re = pt_re;
        p1_im = pt_im;
        ck_re += rz_re;
        ck_im += rz_im;
        rho2 = rt_hypotd_snf(p2_re, p2_im);
        if (rho2 >= tst * ak * ak) {
          if (itime == 2) {
            earlyExit = false;
            exitg1 = true;
          } else {
            ack = rt_hypotd_snf(ck_re, ck_im);
            brm = ack + std::sqrt(ack * ack - 1.0);
            ack = rho2 / rt_hypotd_snf(pt_re, pt_im);
            if ((brm < ack) || rtIsNaN(ack)) {
              ack = brm;
            }

            tst *= std::sqrt(ack / (ack * ack - 1.0));
            itime = 2;
            i++;
          }
        } else {
          i++;
        }
      }

      if (earlyExit) {
        nz = -2;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      iaz += icounter;
      if (iaz > kcounter) {
        kcounter = iaz;
      }

      tst = kcounter;
      p1_re = 0.0;
      p1_im = 0.0;
      p2_re = 1.0020841800044864E-289;
      p2_im = 0.0;
      az = fnu + fnu;
      brm = ((double)kcounter + az) + 1.0;
      gammaln(&brm);
      rho2 = (double)kcounter + 1.0;
      gammaln(&rho2);
      ack = az + 1.0;
      gammaln(&ack);
      flooraz = std::exp((brm - rho2) - ack);
      ak = 0.0;
      s_im = 0.0;
      for (i = 0; i < kcounter; i++) {
        pt_re = p2_re;
        pt_im = p2_im;
        rho2 = tst + fnu;
        brm = rho2 * rz_re;
        rho2 *= rz_im;
        ack = brm * p2_re - rho2 * p2_im;
        rho2 = brm * p2_im + rho2 * p2_re;
        p2_re = p1_re + ack;
        p2_im = p1_im + rho2;
        p1_re = pt_re;
        p1_im = pt_im;
        ack = flooraz * (1.0 - az / (tst + az));
        rho2 = ack + flooraz;
        ak += rho2 * pt_re;
        s_im += rho2 * pt_im;
        flooraz = ack;
        tst--;
      }

      y->re = p2_re;
      y->im = p2_im;
      pt_re = z.re;
      pt_im = z.im;
      if (kode == 2) {
        pt_re = z.re - z.re;
        pt_im = z.im;
      }

      if (rz_im == 0.0) {
        if (rz_re < 0.0) {
          ck_re = std::log(std::abs(rz_re));
          ck_im = 3.1415926535897931;
        } else {
          ck_re = std::log(rz_re);
          ck_im = 0.0;
        }
      } else if ((std::abs(rz_re) > 8.9884656743115785E+307) || (std::abs(rz_im)
                  > 8.9884656743115785E+307)) {
        ck_re = std::log(rt_hypotd_snf(rz_re / 2.0, rz_im / 2.0)) +
          0.69314718055994529;
        ck_im = rt_atan2d_snf(rz_im, rz_re);
      } else {
        ck_re = std::log(rt_hypotd_snf(rz_re, rz_im));
        ck_im = rt_atan2d_snf(rz_im, rz_re);
      }

      flooraz = ck_re;
      brm = 1.0 + fnu;
      gammaln(&brm);
      ck_re = ((-fnu * ck_re - -0.0 * ck_im) + pt_re) - brm;
      ck_im = (-fnu * ck_im + -0.0 * flooraz) + pt_im;
      p2_re += ak;
      p2_im += s_im;
      p1_re = 1.0 / rt_hypotd_snf(p2_re, p2_im);
      if (ck_im == 0.0) {
        ck_re = std::exp(ck_re);
        ck_im = 0.0;
      } else if (rtIsInf(ck_im) && rtIsInf(ck_re) && (ck_re < 0.0)) {
        ck_re = 0.0;
        ck_im = 0.0;
      } else {
        rho2 = std::exp(ck_re / 2.0);
        ck_re = rho2 * (rho2 * std::cos(ck_im));
        ck_im = rho2 * (rho2 * std::sin(ck_im));
      }

      flooraz = ck_re * p1_re - ck_im * 0.0;
      ck_im = ck_re * 0.0 + ck_im * p1_re;
      ack = p2_re * p1_re + p2_im * 0.0;
      p2_im = p2_re * 0.0 - p2_im * p1_re;
      ck_re = flooraz * ack - ck_im * p2_im;
      ck_im = flooraz * p2_im + ck_im * ack;
      rho2 = y->re;
      ack = y->im;
      y->re = rho2 * ck_re - ack * ck_im;
      y->im = rho2 * ck_im + ack * ck_re;
    }
  }

  return nz;
}

/* End of code generation (cmlri.cpp) */
