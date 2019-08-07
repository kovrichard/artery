/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ifft.cpp
 *
 * Code generation for function 'ifft'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "ifft.h"
#include "sim_rx_mex_emxutil.h"
#include "fft.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void b_ifft(const creal_T x[128], creal_T y[128])
{
  int ix;
  int ju;
  int iy;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double twid_re;
  int iheight;
  double twid_im;
  int istart;
  int temp_re_tmp;
  int j;
  static const double dv10[65] = { 0.0, 0.049067674327418015, 0.0980171403295606,
    0.14673047445536175, 0.19509032201612825, 0.24298017990326387,
    0.29028467725446233, 0.33688985339222005, 0.38268343236508978,
    0.42755509343028208, 0.47139673682599764, 0.51410274419322166,
    0.55557023301960218, 0.59569930449243336, 0.63439328416364549,
    0.67155895484701833, 0.70710678118654757, 0.74095112535495922,
    0.773010453362737, 0.80320753148064494, 0.83146961230254524,
    0.85772861000027212, 0.881921264348355, 0.90398929312344334,
    0.92387953251128674, 0.94154406518302081, 0.95694033573220882,
    0.970031253194544, 0.98078528040323043, 0.989176509964781,
    0.99518472667219693, 0.99879545620517241, 1.0, 0.99879545620517241,
    0.99518472667219693, 0.989176509964781, 0.98078528040323043,
    0.970031253194544, 0.95694033573220882, 0.94154406518302081,
    0.92387953251128674, 0.90398929312344334, 0.881921264348355,
    0.85772861000027212, 0.83146961230254524, 0.80320753148064494,
    0.773010453362737, 0.74095112535495922, 0.70710678118654757,
    0.67155895484701833, 0.63439328416364549, 0.59569930449243336,
    0.55557023301960218, 0.51410274419322166, 0.47139673682599764,
    0.42755509343028208, 0.38268343236508978, 0.33688985339222005,
    0.29028467725446233, 0.24298017990326387, 0.19509032201612825,
    0.14673047445536175, 0.0980171403295606, 0.049067674327418015, 0.0 };

  int ihi;
  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 0; i < 127; i++) {
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
      twid_im = dv10[j];
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

  for (iy = 0; iy < 128; iy++) {
    y[iy].re *= 0.0078125;
    y[iy].im *= 0.0078125;
  }
}

void ifft(const emxArray_creal_T *x, emxArray_creal_T *y)
{
  int n1;
  emxArray_real_T *costab1q;
  boolean_T useRadix2;
  int N2blue;
  int nd2;
  double nt_im;
  int rt;
  int i17;
  int k;
  emxArray_real_T *costab;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  emxArray_creal_T *wwc;
  int idx;
  int nInt2;
  int b_y;
  double nt_re;
  emxArray_creal_T *fv;
  double x_re;
  emxArray_creal_T *r5;
  double x_im;
  emxArray_creal_T *b_fv;
  double b_x_im;
  double b_x_re;
  double fv_re;
  double fv_im;
  double b_fv_re;
  n1 = x->size[0];
  if (x->size[0] == 0) {
    y->size[0] = 0;
  } else {
    emxInit_real_T(&costab1q, 2);
    useRadix2 = ((x->size[0] & (x->size[0] - 1)) == 0);
    get_algo_sizes(x->size[0], useRadix2, &N2blue, &nd2);
    nt_im = 6.2831853071795862 / (double)nd2;
    rt = nd2 / 2 / 2;
    i17 = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = rt + 1;
    emxEnsureCapacity_real_T(costab1q, i17);
    costab1q->data[0] = 1.0;
    nd2 = rt / 2 - 1;
    for (k = 0; k <= nd2; k++) {
      costab1q->data[1 + k] = std::cos(nt_im * (1.0 + (double)k));
    }

    i17 = nd2 + 2;
    nd2 = rt - 1;
    for (k = i17; k <= nd2; k++) {
      costab1q->data[k] = std::sin(nt_im * (double)(rt - k));
    }

    costab1q->data[rt] = 0.0;
    emxInit_real_T(&costab, 2);
    emxInit_real_T(&sintab, 2);
    emxInit_real_T(&sintabinv, 2);
    if (!useRadix2) {
      rt = costab1q->size[1] - 1;
      nd2 = (costab1q->size[1] - 1) << 1;
      i17 = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = nd2 + 1;
      emxEnsureCapacity_real_T(costab, i17);
      i17 = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = nd2 + 1;
      emxEnsureCapacity_real_T(sintab, i17);
      costab->data[0] = 1.0;
      sintab->data[0] = 0.0;
      i17 = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = nd2 + 1;
      emxEnsureCapacity_real_T(sintabinv, i17);
      for (k = 0; k < rt; k++) {
        sintabinv->data[1 + k] = costab1q->data[(rt - k) - 1];
      }

      i17 = costab1q->size[1];
      for (k = i17; k <= nd2; k++) {
        sintabinv->data[k] = costab1q->data[k - rt];
      }

      for (k = 0; k < rt; k++) {
        costab->data[1 + k] = costab1q->data[1 + k];
        sintab->data[1 + k] = -costab1q->data[(rt - k) - 1];
      }

      i17 = costab1q->size[1];
      for (k = i17; k <= nd2; k++) {
        costab->data[k] = -costab1q->data[nd2 - k];
        sintab->data[k] = -costab1q->data[k - rt];
      }
    } else {
      rt = costab1q->size[1] - 1;
      nd2 = (costab1q->size[1] - 1) << 1;
      i17 = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = nd2 + 1;
      emxEnsureCapacity_real_T(costab, i17);
      i17 = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = nd2 + 1;
      emxEnsureCapacity_real_T(sintab, i17);
      costab->data[0] = 1.0;
      sintab->data[0] = 0.0;
      for (k = 0; k < rt; k++) {
        costab->data[1 + k] = costab1q->data[1 + k];
        sintab->data[1 + k] = costab1q->data[(rt - k) - 1];
      }

      i17 = costab1q->size[1];
      for (k = i17; k <= nd2; k++) {
        costab->data[k] = -costab1q->data[nd2 - k];
        sintab->data[k] = costab1q->data[k - rt];
      }

      sintabinv->size[0] = 1;
      sintabinv->size[1] = 0;
    }

    emxFree_real_T(&costab1q);
    if (useRadix2) {
      r2br_r2dit_trig(x, x->size[0], costab, sintab, y);
    } else {
      emxInit_creal_T(&wwc, 1);
      nd2 = (x->size[0] + x->size[0]) - 1;
      i17 = wwc->size[0];
      wwc->size[0] = nd2;
      emxEnsureCapacity_creal_T(wwc, i17);
      idx = x->size[0];
      rt = 0;
      wwc->data[x->size[0] - 1].re = 1.0;
      wwc->data[x->size[0] - 1].im = 0.0;
      nInt2 = x->size[0] << 1;
      i17 = x->size[0];
      for (k = 0; k <= i17 - 2; k++) {
        b_y = ((1 + k) << 1) - 1;
        if (nInt2 - rt <= b_y) {
          rt += b_y - nInt2;
        } else {
          rt += b_y;
        }

        nt_im = 3.1415926535897931 * (double)rt / (double)n1;
        if (nt_im == 0.0) {
          nt_re = 1.0;
          nt_im = 0.0;
        } else {
          nt_re = std::cos(nt_im);
          nt_im = std::sin(nt_im);
        }

        wwc->data[idx - 2].re = nt_re;
        wwc->data[idx - 2].im = -nt_im;
        idx--;
      }

      idx = 0;
      i17 = nd2 - 1;
      for (k = i17; k >= n1; k--) {
        wwc->data[k] = wwc->data[idx];
        idx++;
      }

      nInt2 = x->size[0] - 1;
      nd2 = x->size[0];
      i17 = y->size[0];
      y->size[0] = nd2;
      emxEnsureCapacity_creal_T(y, i17);
      nd2 = 0;
      for (k = 0; k <= nInt2; k++) {
        rt = (n1 + k) - 1;
        nt_re = wwc->data[rt].re;
        nt_im = wwc->data[rt].im;
        x_re = x->data[nd2].re;
        x_im = x->data[nd2].im;
        b_x_im = x->data[nd2].im;
        b_x_re = x->data[nd2].re;
        y->data[k].re = nt_re * x_re + nt_im * x_im;
        y->data[k].im = nt_re * b_x_im - nt_im * b_x_re;
        nd2++;
      }

      i17 = nInt2 + 2;
      for (k = i17; k <= n1; k++) {
        y->data[k - 1].re = 0.0;
        y->data[k - 1].im = 0.0;
      }

      emxInit_creal_T(&fv, 1);
      emxInit_creal_T(&r5, 1);
      emxInit_creal_T(&b_fv, 1);
      b_r2br_r2dit_trig_impl(y, N2blue, costab, sintab, fv);
      b_r2br_r2dit_trig_impl(wwc, N2blue, costab, sintab, r5);
      i17 = b_fv->size[0];
      b_fv->size[0] = fv->size[0];
      emxEnsureCapacity_creal_T(b_fv, i17);
      nd2 = fv->size[0];
      for (i17 = 0; i17 < nd2; i17++) {
        fv_re = fv->data[i17].re;
        fv_im = fv->data[i17].im;
        nt_im = r5->data[i17].re;
        x_re = r5->data[i17].im;
        b_fv->data[i17].re = fv_re * nt_im - fv_im * x_re;
        b_fv->data[i17].im = fv_re * x_re + fv_im * nt_im;
      }

      emxFree_creal_T(&r5);
      r2br_r2dit_trig(b_fv, N2blue, costab, sintabinv, fv);
      idx = 0;
      nt_re = x->size[0];
      i17 = x->size[0];
      nd2 = wwc->size[0];
      emxFree_creal_T(&b_fv);
      for (k = i17; k <= nd2; k++) {
        nt_im = wwc->data[k - 1].re;
        fv_re = fv->data[k - 1].re;
        x_re = wwc->data[k - 1].im;
        fv_im = fv->data[k - 1].im;
        x_im = wwc->data[k - 1].re;
        b_x_im = fv->data[k - 1].im;
        b_x_re = wwc->data[k - 1].im;
        b_fv_re = fv->data[k - 1].re;
        y->data[idx].re = nt_im * fv_re + x_re * fv_im;
        y->data[idx].im = x_im * b_x_im - b_x_re * b_fv_re;
        nt_im = wwc->data[k - 1].re;
        fv_re = fv->data[k - 1].re;
        x_re = wwc->data[k - 1].im;
        fv_im = fv->data[k - 1].im;
        x_im = wwc->data[k - 1].re;
        b_x_im = fv->data[k - 1].im;
        b_x_re = wwc->data[k - 1].im;
        b_fv_re = fv->data[k - 1].re;
        y->data[idx].re = nt_im * fv_re + x_re * fv_im;
        y->data[idx].im = x_im * b_x_im - b_x_re * b_fv_re;
        nt_im = y->data[idx].re;
        x_re = y->data[idx].im;
        if (x_re == 0.0) {
          y->data[idx].re = nt_im / (double)(int)nt_re;
          y->data[idx].im = 0.0;
        } else if (nt_im == 0.0) {
          y->data[idx].re = 0.0;
          y->data[idx].im = x_re / (double)(int)nt_re;
        } else {
          y->data[idx].re = nt_im / (double)(int)nt_re;
          y->data[idx].im = x_re / (double)(int)nt_re;
        }

        idx++;
      }

      emxFree_creal_T(&fv);
      emxFree_creal_T(&wwc);
    }

    emxFree_real_T(&sintabinv);
    emxFree_real_T(&sintab);
    emxFree_real_T(&costab);
  }
}

/* End of code generation (ifft.cpp) */
