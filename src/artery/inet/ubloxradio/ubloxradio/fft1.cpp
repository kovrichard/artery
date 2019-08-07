/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fft1.cpp
 *
 * Code generation for function 'fft1'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "fft1.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void fft(const creal_T x[64], creal_T y[128])
{
  int i;
  int ix;
  int ju;
  int iy;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double twid_re;
  int iheight;
  double twid_im;
  int istart;
  int temp_re_tmp;
  int j;
  static const double dv9[65] = { 0.0, -0.049067674327418015,
    -0.0980171403295606, -0.14673047445536175, -0.19509032201612825,
    -0.24298017990326387, -0.29028467725446233, -0.33688985339222005,
    -0.38268343236508978, -0.42755509343028208, -0.47139673682599764,
    -0.51410274419322166, -0.55557023301960218, -0.59569930449243336,
    -0.63439328416364549, -0.67155895484701833, -0.70710678118654757,
    -0.74095112535495922, -0.773010453362737, -0.80320753148064494,
    -0.83146961230254524, -0.85772861000027212, -0.881921264348355,
    -0.90398929312344334, -0.92387953251128674, -0.94154406518302081,
    -0.95694033573220882, -0.970031253194544, -0.98078528040323043,
    -0.989176509964781, -0.99518472667219693, -0.99879545620517241, -1.0,
    -0.99879545620517241, -0.99518472667219693, -0.989176509964781,
    -0.98078528040323043, -0.970031253194544, -0.95694033573220882,
    -0.94154406518302081, -0.92387953251128674, -0.90398929312344334,
    -0.881921264348355, -0.85772861000027212, -0.83146961230254524,
    -0.80320753148064494, -0.773010453362737, -0.74095112535495922,
    -0.70710678118654757, -0.67155895484701833, -0.63439328416364549,
    -0.59569930449243336, -0.55557023301960218, -0.51410274419322166,
    -0.47139673682599764, -0.42755509343028208, -0.38268343236508978,
    -0.33688985339222005, -0.29028467725446233, -0.24298017990326387,
    -0.19509032201612825, -0.14673047445536175, -0.0980171403295606,
    -0.049067674327418015, -0.0 };

  int ihi;
  for (i = 0; i < 128; i++) {
    y[i].re = 0.0;
    y[i].im = 0.0;
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 0; i < 63; i++) {
    y[iy] = x[ix];
    iy = 128;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }

    iy = ju;
    ix++;
  }

  y[iy] = x[ix];
  for (i = 0; i <= 126; i += 2) {
    temp_re = y[i + 1].re;
    temp_im = y[i + 1].im;
    twid_re = y[i].re;
    twid_im = y[i].im;
    y[i + 1].re = y[i].re - y[i + 1].re;
    y[i + 1].im = y[i].im - y[i + 1].im;
    twid_re += temp_re;
    twid_im += temp_im;
    y[i].re = twid_re;
    y[i].im = twid_im;
  }

  iy = 2;
  ix = 4;
  ju = 32;
  iheight = 125;
  while (ju > 0) {
    for (i = 0; i < iheight; i += ix) {
      temp_re_tmp = i + iy;
      temp_re = y[temp_re_tmp].re;
      temp_im = y[i + iy].im;
      y[temp_re_tmp].re = y[i].re - y[temp_re_tmp].re;
      y[temp_re_tmp].im = y[i].im - temp_im;
      y[i].re += temp_re;
      y[i].im += temp_im;
    }

    istart = 1;
    for (j = ju; j < 64; j += ju) {
      twid_re = dv4[j];
      twid_im = dv9[j];
      i = istart;
      ihi = istart + iheight;
      while (i < ihi) {
        temp_re_tmp = i + iy;
        temp_re = twid_re * y[temp_re_tmp].re - twid_im * y[i + iy].im;
        temp_im = twid_re * y[i + iy].im + twid_im * y[i + iy].re;
        y[temp_re_tmp].re = y[i].re - temp_re;
        y[temp_re_tmp].im = y[i].im - temp_im;
        y[i].re += temp_re;
        y[i].im += temp_im;
        i += ix;
      }

      istart++;
    }

    ju /= 2;
    iy = ix;
    ix += ix;
    iheight -= iy;
  }
}

/* End of code generation (fft1.cpp) */
