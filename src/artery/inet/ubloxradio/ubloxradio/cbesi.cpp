/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * cbesi.cpp
 *
 * Code generation for function 'cbesi'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "cbesi.h"
#include "cmlri.h"
#include "casyi.h"
#include "cuchk.h"
#include "gammaln.h"
#include "kaiser.h"
#include "sim_rx_mex_rtwutil.h"

/* Function Definitions */
void cbesi(const creal_T z, creal_T *cy, int *nz, int *ierr)
{
  double AZ;
  creal_T zn;
  double az_tmp;
  boolean_T guard1 = false;
  int nw;
  double crsc_re;
  boolean_T iflag;
  creal_T hz;
  double coef_re;
  double cz_re;
  double b_AZ;
  double cz_im;
  double acz;
  double hz_re;
  double aa;
  double hz_im;
  double coef_im;
  double b_atol;
  double s1_re;
  double s1_im;
  double ak;
  double s;
  double rs;
  *ierr = 0;
  AZ = rt_hypotd_snf(z.re, z.im);
  if (AZ > 1.0737418235E+9) {
    *ierr = 4;
  } else {
    if (AZ > 32767.999992370605) {
      *ierr = 3;
    }
  }

  zn = z;
  if (z.re < 0.0) {
    zn.re = -z.re;
    zn.im = -z.im;
  }

  cy->re = 0.0;
  cy->im = 0.0;
  az_tmp = rt_hypotd_snf(zn.re, zn.im);
  guard1 = false;
  if (az_tmp <= 2.0) {
    nw = 0;
    if (az_tmp == 0.0) {
      cy->re = 1.0;
      cy->im = 0.0;
    } else {
      crsc_re = 1.0;
      iflag = false;
      if (az_tmp < 2.2250738585072014E-305) {
        cy->re = 1.0;
        cy->im = 0.0;
      } else {
        hz.re = 0.5 * zn.re;
        hz.im = 0.5 * zn.im;
        if (az_tmp > 4.7170688552396617E-153) {
          cz_re = hz.re * hz.re - hz.im * hz.im;
          cz_im = hz.re * hz.im + hz.im * hz.re;
          acz = rt_hypotd_snf(cz_re, cz_im);
        } else {
          cz_re = 0.0;
          cz_im = 0.0;
          acz = 0.0;
        }

        if (hz.im == 0.0) {
          if (hz.re < 0.0) {
            hz_re = hz.re;
            hz.re = std::log(std::abs(hz_re));
            hz.im = 3.1415926535897931;
          } else {
            hz_re = hz.re;
            hz.re = std::log(hz_re);
            hz.im = 0.0;
          }
        } else if ((std::abs(hz.re) > 8.9884656743115785E+307) || (std::abs
                    (hz.im) > 8.9884656743115785E+307)) {
          hz_re = hz.re;
          hz_im = hz.im;
          AZ = hz.im;
          coef_re = hz.re;
          hz.re = std::log(rt_hypotd_snf(hz_re / 2.0, hz_im / 2.0)) +
            0.69314718055994529;
          hz.im = rt_atan2d_snf(AZ, coef_re);
        } else {
          hz_re = hz.re;
          hz_im = hz.im;
          AZ = hz.im;
          coef_re = hz.re;
          hz.re = std::log(rt_hypotd_snf(hz_re, hz_im));
          hz.im = rt_atan2d_snf(AZ, coef_re);
        }

        AZ = 1.0;
        gammaln(&AZ);
        hz.re = hz.re * 0.0 - AZ;
        hz.im *= 0.0;
        if (hz.re > -700.92179369444591) {
          AZ = 0.0;
          if (hz.re <= -664.87164553371019) {
            iflag = true;
            crsc_re = 2.2204460492503131E-16;
            AZ = 1.0020841800044864E-289;
          }

          aa = std::exp(hz.re);
          if (iflag) {
            aa /= 2.2204460492503131E-16;
          }

          coef_re = aa * std::cos(hz.im);
          coef_im = aa * std::sin(hz.im);
          b_atol = 2.2204460492503131E-16 * acz;
          s1_re = 1.0;
          s1_im = 0.0;
          if (!(acz < 2.2204460492503131E-16)) {
            hz.re = 1.0;
            hz.im = 0.0;
            ak = 3.0;
            s = 1.0;
            aa = 2.0;
            do {
              rs = 1.0 / s;
              hz_re = hz.re;
              hz_im = hz.im;
              hz.re = rs * (hz_re * cz_re - hz_im * cz_im);
              hz.im = rs * (hz_re * cz_im + hz_im * cz_re);
              s1_re += hz.re;
              s1_im += hz.im;
              s += ak;
              ak += 2.0;
              aa = aa * acz * rs;
            } while (!!(aa > b_atol));
          }

          hz.re = s1_re * coef_re - s1_im * coef_im;
          hz.im = s1_re * coef_im + s1_im * coef_re;
          if (iflag && (cuchk(hz, AZ) != 0)) {
          } else {
            cy->re = hz.re * crsc_re - hz.im * 0.0;
            cy->im = hz.re * 0.0 + hz.im * crsc_re;
          }
        } else {
          nw = 1;
          if (acz > 0.0) {
            nw = -1;
          }
        }
      }
    }

    if (nw < 0) {
      *nz = 1;
    } else {
      *nz = nw;
    }

    if ((1 - *nz == 0) || (nw >= 0)) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    if (az_tmp < 21.784271729432426) {
      nw = cmlri(zn, 0.0, 1, cy);
      if (nw < 0) {
        if (nw == -2) {
          *nz = -2;
        } else {
          *nz = -1;
        }
      } else {
        *nz = 0;
      }
    } else {
      nw = casyi(zn, 0.0, 1, cy);
      if (nw < 0) {
        if (nw == -2) {
          *nz = -2;
        } else {
          *nz = -1;
        }
      } else {
        *nz = 0;
      }
    }
  }

  if (*nz < 0) {
    if (*nz == -2) {
      *nz = 0;
      *ierr = 5;
    } else {
      *nz = 0;
      *ierr = 2;
    }
  } else {
    if ((!(z.re >= 0.0)) && (*nz != 1)) {
      AZ = std::abs(cy->re);
      coef_re = std::abs(cy->im);
      if ((AZ > coef_re) || rtIsNaN(coef_re)) {
        b_AZ = AZ;
      } else {
        b_AZ = coef_re;
      }

      if (b_AZ <= 1.0020841800044864E-289) {
        cy->re *= 4.503599627370496E+15;
        cy->im *= 4.503599627370496E+15;
        AZ = 2.2204460492503131E-16;
      } else {
        AZ = 1.0;
      }

      coef_re = cy->re;
      aa = cy->im;
      cy->re = coef_re - aa * 0.0;
      cy->im = coef_re * 0.0 + aa;
      cy->re *= AZ;
      cy->im *= AZ;
    }
  }
}

/* End of code generation (cbesi.cpp) */
