/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * data_rx.cpp
 *
 * Code generation for function 'data_rx'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "add_impairments.h"
#include "sim_rx.h"
#include "sim_tx.h"
#include "data_rx.h"
#include "mod1.h"
#include "sim_rx_mex_emxutil.h"
#include "scrambler_tx.h"
#include "SystemCore.h"
#include "data_tx.h"
#include "repmat.h"
#include "ViterbiDecoder.h"
#include "mod.h"
#include "floor1.h"
#include "qam64_demap.h"
#include "qam16_demap.h"
#include "mpower.h"
#include "kaiser.h"
#include "cmlri.h"
#include "fft.h"
#include "sim_rx_mex_rtwutil.h"
#include "sim_rx_mex_data.h"

/* Function Definitions */
void data_rx(const double PHY_pilot_idx[4], const double PHY_polarity_sign[127],
             const double PHY_data_idx[48], double SIG_CFG_r_num, double
             SIG_CFG_n_bpscs, double SIG_CFG_n_cbps, double SIG_CFG_n_dbps,
             double SIG_CFG_n_sym, const emxArray_creal_T *rx_wf, double idx,
             const creal_T h_est[64], double r_cfo, emxArray_boolean_T
             *descr_msg)
{
  emxArray_real_T *data_out_vec;
  int i19;
  int ar_tmp;
  int i20;
  int i;
  emxArray_real_T *y;
  int llr_in_size_idx_0_tmp;
  int llr_in_dep_size;
  int b_llr_in_dep_size[1];
  int loop_ub_tmp;
  int i_sym;
  double s;
  double a;
  creal_T wf_in[64];
  double b_a;
  int punct_pat_size[2];
  creal_T b_wf_in[64];
  creal_T dcv5[64];
  boolean_T punct_pat_data[6];
  double r;
  static const boolean_T bv0[6] = { true, true, true, false, false, true };

  double ar;
  double y_re;
  double ai;
  boolean_T idx_data[2592];
  int ii_size[2];
  double y_im;
  boolean_T b_idx_data[1728];
  double llr_in_dep_data[288];
  double bi;
  double brm;
  double a_re;
  emxArray_real_T *data_out;
  double bits_out_data[144];
  double b_s;
  double b_y[4];
  emxArray_boolean_T *in;
  creal_T sym_out[48];
  emxArray_boolean_T *b_in;
  boolean_T pn_seq[7];
  double snr[48];
  double llr_in_data[288];
  double b_sym_out[48];
  double dv14[96];
  int llr_in_data_tmp;
  double x_data_data[288];
  double ii_data[288];
  int tmp_size[2];
  double tmp_data[288];
  short b_tmp_data[288];
  int b_tmp_size[2];
  double c_tmp_data[288];
  int b_ii_size[2];
  double b_ii_data[288];
  int c_llr_in_dep_size[1];
  double b_llr_in_dep_data[432];
  double b_bits_out_data[216];
  short i21;
  short d_tmp_data[432];
  emxInit_real_T(&data_out_vec, 2);

  /* DATA_RX Receiver processing of all DATA OFDM symbols */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    August 2018; Last revision: 11-February-2019 */
  /*  Copyright (C) u-blox */
  /*  */
  /*  All rights reserved. */
  /*  */
  /*  Permission to use, copy, modify, and distribute this software for any */
  /*  purpose without fee is hereby granted, provided that this entire notice */
  /*  is included in all copies of any software which is or includes a copy */
  /*  or modification of this software and in all copies of the supporting */
  /*  documentation for such software. */
  /*  */
  /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
  /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
  /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
  /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
  /*  */
  /*  Project: ubx-v2x */
  /*  Purpose: V2X baseband simulation model */
  /*  Needed for code generation */
  /*  Loop for all OFDM symbols */
  i19 = data_out_vec->size[0] * data_out_vec->size[1];
  ar_tmp = (int)SIG_CFG_n_dbps;
  data_out_vec->size[0] = ar_tmp;
  i20 = (int)SIG_CFG_n_sym;
  data_out_vec->size[1] = i20;
  emxEnsureCapacity_real_T(data_out_vec, i19);
  i = ar_tmp * i20;
  for (i19 = 0; i19 < i; i19++) {
    data_out_vec->data[i19] = 0.0;
  }

  emxInit_real_T(&y, 2);
  if (0 <= i20 - 1) {
    llr_in_size_idx_0_tmp = (int)SIG_CFG_n_bpscs;
    loop_ub_tmp = llr_in_size_idx_0_tmp * 48;
    s = SIG_CFG_n_bpscs / 2.0;
    if (!(s > 1.0)) {
      s = 1.0;
    }

    a = SIG_CFG_n_cbps / 16.0;
    b_llr_in_dep_size[0] = loop_ub_tmp;
  }

  llr_in_dep_size = b_llr_in_dep_size[0];
  for (i_sym = 0; i_sym < i20; i_sym++) {
    /*  Get waveform for current OFDM symbol */
    idx += 80.0;
    for (i19 = 0; i19 < 64; i19++) {
      wf_in[i19] = rx_wf->data[(int)(idx + (double)i19) - 1];
    }

    /*  Find polarity sign for pilot subcarriers */
    b_a = PHY_polarity_sign[(int)(f_mod(((1.0 + (double)i_sym) + 1.0) - 1.0,
      127.0) + 1.0) - 1];

    /*  Perform FFT */
    /* DOT11_FFT 802.11 FFT */
    /*  */
    /*    Author: Ioannis Sarris, u-blox */
    /*    email: ioannis.sarris@u-blox.com */
    /*    August 2018; Last revision: 30-August-2018 */
    /*  Copyright (C) u-blox */
    /*  */
    /*  All rights reserved. */
    /*  */
    /*  Permission to use, copy, modify, and distribute this software for any */
    /*  purpose without fee is hereby granted, provided that this entire notice */
    /*  is included in all copies of any software which is or includes a copy */
    /*  or modification of this software and in all copies of the supporting */
    /*  documentation for such software. */
    /*  */
    /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
    /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
    /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
    /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
    /*  */
    /*  Project: ubx-v2x */
    /*  Purpose: V2X baseband simulation model */
    /*  FFT with shifting and normalization */
    for (i = 0; i < 64; i++) {
      b_wf_in[i] = wf_in[iv5[i]];
    }

    r2br_r2dit_trig_impl(b_wf_in, dv0, dv3, dcv5);
    for (i = 0; i < 64; i++) {
      ar = dcv5[iv0[i]].re * 7.2111025509279782;
      ai = dcv5[iv0[i]].im * 7.2111025509279782;
      if (ai == 0.0) {
        wf_in[i].re = ar / 64.0;
        wf_in[i].im = 0.0;
      } else if (ar == 0.0) {
        wf_in[i].re = 0.0;
        wf_in[i].im = ai / 64.0;
      } else {
        wf_in[i].re = ar / 64.0;
        wf_in[i].im = ai / 64.0;
      }
    }

    /*  Pilot equalization */
    y_re = r_cfo * 0.0;
    if (-r_cfo == 0.0) {
      y_re = std::exp(y_re);
      y_im = 0.0;
    } else {
      r = std::exp(y_re / 2.0);
      y_re = r * (r * std::cos(-r_cfo));
      y_im = r * (r * std::sin(-r_cfo));
    }

    /*  Residual CFO estimation */
    ar_tmp = (int)PHY_pilot_idx[0] - 1;
    ar = b_a * wf_in[ar_tmp].re;
    ai = b_a * wf_in[(int)PHY_pilot_idx[0] - 1].im;
    bi = h_est[(int)PHY_pilot_idx[0] - 1].im;
    if (bi == 0.0) {
      if (ai == 0.0) {
        a_re = ar / h_est[ar_tmp].re;
        r = 0.0;
      } else if (ar == 0.0) {
        a_re = 0.0;
        r = ai / h_est[ar_tmp].re;
      } else {
        a_re = ar / h_est[ar_tmp].re;
        r = ai / h_est[ar_tmp].re;
      }
    } else if (h_est[ar_tmp].re == 0.0) {
      if (ar == 0.0) {
        a_re = ai / bi;
        r = 0.0;
      } else if (ai == 0.0) {
        a_re = 0.0;
        r = -(ar / bi);
      } else {
        a_re = ai / bi;
        r = -(ar / bi);
      }
    } else {
      brm = std::abs(h_est[ar_tmp].re);
      r = std::abs(bi);
      if (brm > r) {
        b_s = bi / h_est[ar_tmp].re;
        r = h_est[ar_tmp].re + b_s * bi;
        a_re = (ar + b_s * ai) / r;
        r = (ai - b_s * ar) / r;
      } else if (r == brm) {
        if (h_est[ar_tmp].re > 0.0) {
          b_s = 0.5;
        } else {
          b_s = -0.5;
        }

        if (bi > 0.0) {
          r = 0.5;
        } else {
          r = -0.5;
        }

        a_re = (ar * b_s + ai * r) / brm;
        r = (ai * b_s - ar * r) / brm;
      } else {
        b_s = h_est[ar_tmp].re / bi;
        r = bi + b_s * h_est[ar_tmp].re;
        a_re = (b_s * ar + ai) / r;
        r = (b_s * ai - ar) / r;
      }
    }

    b_y[0] = rt_atan2d_snf(a_re * y_im + r * y_re, a_re * y_re - r * y_im);
    ar_tmp = (int)PHY_pilot_idx[1] - 1;
    ar = b_a * wf_in[ar_tmp].re;
    ai = b_a * wf_in[(int)PHY_pilot_idx[1] - 1].im;
    bi = h_est[(int)PHY_pilot_idx[1] - 1].im;
    if (bi == 0.0) {
      if (ai == 0.0) {
        a_re = ar / h_est[ar_tmp].re;
        r = 0.0;
      } else if (ar == 0.0) {
        a_re = 0.0;
        r = ai / h_est[ar_tmp].re;
      } else {
        a_re = ar / h_est[ar_tmp].re;
        r = ai / h_est[ar_tmp].re;
      }
    } else if (h_est[ar_tmp].re == 0.0) {
      if (ar == 0.0) {
        a_re = ai / bi;
        r = 0.0;
      } else if (ai == 0.0) {
        a_re = 0.0;
        r = -(ar / bi);
      } else {
        a_re = ai / bi;
        r = -(ar / bi);
      }
    } else {
      brm = std::abs(h_est[ar_tmp].re);
      r = std::abs(bi);
      if (brm > r) {
        b_s = bi / h_est[ar_tmp].re;
        r = h_est[ar_tmp].re + b_s * bi;
        a_re = (ar + b_s * ai) / r;
        r = (ai - b_s * ar) / r;
      } else if (r == brm) {
        if (h_est[ar_tmp].re > 0.0) {
          b_s = 0.5;
        } else {
          b_s = -0.5;
        }

        if (bi > 0.0) {
          r = 0.5;
        } else {
          r = -0.5;
        }

        a_re = (ar * b_s + ai * r) / brm;
        r = (ai * b_s - ar * r) / brm;
      } else {
        b_s = h_est[ar_tmp].re / bi;
        r = bi + b_s * h_est[ar_tmp].re;
        a_re = (b_s * ar + ai) / r;
        r = (b_s * ai - ar) / r;
      }
    }

    b_y[1] = rt_atan2d_snf(a_re * y_im + r * y_re, a_re * y_re - r * y_im);
    ar_tmp = (int)PHY_pilot_idx[2] - 1;
    ar = b_a * wf_in[ar_tmp].re;
    ai = b_a * wf_in[(int)PHY_pilot_idx[2] - 1].im;
    bi = h_est[(int)PHY_pilot_idx[2] - 1].im;
    if (bi == 0.0) {
      if (ai == 0.0) {
        a_re = ar / h_est[ar_tmp].re;
        r = 0.0;
      } else if (ar == 0.0) {
        a_re = 0.0;
        r = ai / h_est[ar_tmp].re;
      } else {
        a_re = ar / h_est[ar_tmp].re;
        r = ai / h_est[ar_tmp].re;
      }
    } else if (h_est[ar_tmp].re == 0.0) {
      if (ar == 0.0) {
        a_re = ai / bi;
        r = 0.0;
      } else if (ai == 0.0) {
        a_re = 0.0;
        r = -(ar / bi);
      } else {
        a_re = ai / bi;
        r = -(ar / bi);
      }
    } else {
      brm = std::abs(h_est[ar_tmp].re);
      r = std::abs(bi);
      if (brm > r) {
        b_s = bi / h_est[ar_tmp].re;
        r = h_est[ar_tmp].re + b_s * bi;
        a_re = (ar + b_s * ai) / r;
        r = (ai - b_s * ar) / r;
      } else if (r == brm) {
        if (h_est[ar_tmp].re > 0.0) {
          b_s = 0.5;
        } else {
          b_s = -0.5;
        }

        if (bi > 0.0) {
          r = 0.5;
        } else {
          r = -0.5;
        }

        a_re = (ar * b_s + ai * r) / brm;
        r = (ai * b_s - ar * r) / brm;
      } else {
        b_s = h_est[ar_tmp].re / bi;
        r = bi + b_s * h_est[ar_tmp].re;
        a_re = (b_s * ar + ai) / r;
        r = (b_s * ai - ar) / r;
      }
    }

    b_y[2] = rt_atan2d_snf(a_re * y_im + r * y_re, a_re * y_re - r * y_im);
    ar_tmp = (int)PHY_pilot_idx[3] - 1;
    ar = b_a * wf_in[ar_tmp].re;
    ai = b_a * wf_in[(int)PHY_pilot_idx[3] - 1].im;
    bi = h_est[(int)PHY_pilot_idx[3] - 1].im;
    if (bi == 0.0) {
      if (ai == 0.0) {
        a_re = ar / h_est[ar_tmp].re;
        r = 0.0;
      } else if (ar == 0.0) {
        a_re = 0.0;
        r = ai / h_est[ar_tmp].re;
      } else {
        a_re = ar / h_est[ar_tmp].re;
        r = ai / h_est[ar_tmp].re;
      }
    } else if (h_est[ar_tmp].re == 0.0) {
      if (ar == 0.0) {
        a_re = ai / bi;
        r = 0.0;
      } else if (ai == 0.0) {
        a_re = 0.0;
        r = -(ar / bi);
      } else {
        a_re = ai / bi;
        r = -(ar / bi);
      }
    } else {
      brm = std::abs(h_est[ar_tmp].re);
      r = std::abs(bi);
      if (brm > r) {
        b_s = bi / h_est[ar_tmp].re;
        r = h_est[ar_tmp].re + b_s * bi;
        a_re = (ar + b_s * ai) / r;
        r = (ai - b_s * ar) / r;
      } else if (r == brm) {
        if (h_est[ar_tmp].re > 0.0) {
          b_s = 0.5;
        } else {
          b_s = -0.5;
        }

        if (bi > 0.0) {
          r = 0.5;
        } else {
          r = -0.5;
        }

        a_re = (ar * b_s + ai * r) / brm;
        r = (ai * b_s - ar * r) / brm;
      } else {
        b_s = h_est[ar_tmp].re / bi;
        r = bi + b_s * h_est[ar_tmp].re;
        a_re = (b_s * ar + ai) / r;
        r = (b_s * ai - ar) / r;
      }
    }

    r_cfo = (r_cfo + (((b_y[0] + b_y[1]) + b_y[2]) + rt_atan2d_snf(-a_re * y_im
               + -r * y_re, -a_re * y_re - -r * y_im)) / 4.0) / 2.0;

    /*  Data equalization with CFO compensation */
    y_re = r_cfo * 0.0;
    if (-r_cfo == 0.0) {
      y_re = std::exp(y_re);
      y_im = 0.0;
    } else {
      r = std::exp(y_re / 2.0);
      y_re = r * (r * std::cos(-r_cfo));
      y_im = r * (r * std::sin(-r_cfo));
    }

    /*  SNR input to Viterbi */
    for (i = 0; i < 48; i++) {
      ar_tmp = (int)PHY_data_idx[i] - 1;
      ai = wf_in[(int)PHY_data_idx[i] - 1].im;
      bi = h_est[(int)PHY_data_idx[i] - 1].im;
      if (bi == 0.0) {
        if (ai == 0.0) {
          ar = wf_in[ar_tmp].re / h_est[ar_tmp].re;
          r = 0.0;
        } else if (wf_in[ar_tmp].re == 0.0) {
          ar = 0.0;
          r = ai / h_est[ar_tmp].re;
        } else {
          ar = wf_in[ar_tmp].re / h_est[ar_tmp].re;
          r = ai / h_est[ar_tmp].re;
        }
      } else if (h_est[ar_tmp].re == 0.0) {
        if (wf_in[ar_tmp].re == 0.0) {
          ar = ai / bi;
          r = 0.0;
        } else if (ai == 0.0) {
          ar = 0.0;
          r = -(wf_in[ar_tmp].re / bi);
        } else {
          ar = ai / bi;
          r = -(wf_in[ar_tmp].re / bi);
        }
      } else {
        brm = std::abs(h_est[ar_tmp].re);
        r = std::abs(bi);
        if (brm > r) {
          b_s = bi / h_est[ar_tmp].re;
          r = h_est[ar_tmp].re + b_s * bi;
          ar = (wf_in[ar_tmp].re + b_s * ai) / r;
          r = (ai - b_s * wf_in[ar_tmp].re) / r;
        } else if (r == brm) {
          if (h_est[ar_tmp].re > 0.0) {
            b_s = 0.5;
          } else {
            b_s = -0.5;
          }

          if (bi > 0.0) {
            r = 0.5;
          } else {
            r = -0.5;
          }

          ar = (wf_in[ar_tmp].re * b_s + ai * r) / brm;
          r = (ai * b_s - wf_in[ar_tmp].re * r) / brm;
        } else {
          b_s = h_est[ar_tmp].re / bi;
          r = bi + b_s * h_est[ar_tmp].re;
          ar = (b_s * wf_in[ar_tmp].re + ai) / r;
          r = (b_s * ai - wf_in[ar_tmp].re) / r;
        }
      }

      sym_out[i].re = ar * y_re - r * y_im;
      sym_out[i].im = ar * y_im + r * y_re;
      snr[i] = rt_powd_snf(rt_hypotd_snf(h_est[(int)PHY_data_idx[i] - 1].re,
        h_est[(int)PHY_data_idx[i] - 1].im), 2.0);
    }

    /*  LLR demapping */
    /* LLR_DEMAP LLR demapping */
    /*  */
    /*    Author: Ioannis Sarris, u-blox */
    /*    email: ioannis.sarris@u-blox.com */
    /*    August 2018; Last revision: 30-August-2018 */
    /*  Copyright (C) u-blox */
    /*  */
    /*  All rights reserved. */
    /*  */
    /*  Permission to use, copy, modify, and distribute this software for any */
    /*  purpose without fee is hereby granted, provided that this entire notice */
    /*  is included in all copies of any software which is or includes a copy */
    /*  or modification of this software and in all copies of the supporting */
    /*  documentation for such software. */
    /*  */
    /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
    /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
    /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
    /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
    /*  */
    /*  Project: ubx-v2x */
    /*  Purpose: V2X baseband simulation model */
    /*  Initialize llr_out matrix */
    if (0 <= loop_ub_tmp - 1) {
      memset(&llr_in_data[0], 0, (unsigned int)(loop_ub_tmp * (int)sizeof(double)));
    }

    switch ((int)SIG_CFG_n_bpscs) {
     case 1:
      /*  BPSK */
      for (i19 = 0; i19 < 48; i19++) {
        ar_tmp = (int)PHY_data_idx[i19] - 1;
        ai = wf_in[(int)PHY_data_idx[i19] - 1].im;
        bi = h_est[(int)PHY_data_idx[i19] - 1].im;
        if (bi == 0.0) {
          if (ai == 0.0) {
            ar = wf_in[ar_tmp].re / h_est[ar_tmp].re;
            r = 0.0;
          } else if (wf_in[ar_tmp].re == 0.0) {
            ar = 0.0;
            r = ai / h_est[ar_tmp].re;
          } else {
            ar = wf_in[ar_tmp].re / h_est[ar_tmp].re;
            r = ai / h_est[ar_tmp].re;
          }
        } else if (h_est[ar_tmp].re == 0.0) {
          if (wf_in[ar_tmp].re == 0.0) {
            ar = ai / bi;
            r = 0.0;
          } else if (ai == 0.0) {
            ar = 0.0;
            r = -(wf_in[ar_tmp].re / bi);
          } else {
            ar = ai / bi;
            r = -(wf_in[ar_tmp].re / bi);
          }
        } else {
          brm = std::abs(h_est[ar_tmp].re);
          r = std::abs(bi);
          if (brm > r) {
            b_s = bi / h_est[ar_tmp].re;
            r = h_est[ar_tmp].re + b_s * bi;
            ar = (wf_in[ar_tmp].re + b_s * ai) / r;
            r = (ai - b_s * wf_in[ar_tmp].re) / r;
          } else if (r == brm) {
            if (h_est[ar_tmp].re > 0.0) {
              b_s = 0.5;
            } else {
              b_s = -0.5;
            }

            if (bi > 0.0) {
              r = 0.5;
            } else {
              r = -0.5;
            }

            ar = (wf_in[ar_tmp].re * b_s + ai * r) / brm;
            r = (ai * b_s - wf_in[ar_tmp].re * r) / brm;
          } else {
            b_s = h_est[ar_tmp].re / bi;
            r = bi + b_s * h_est[ar_tmp].re;
            ar = (b_s * wf_in[ar_tmp].re + ai) / r;
            r = (b_s * ai - wf_in[ar_tmp].re) / r;
          }
        }

        llr_in_data[llr_in_size_idx_0_tmp * i19] = 4.0 * (ar * y_re - r * y_im);
      }
      break;

     case 2:
      /*  QPSK */
      for (i19 = 0; i19 < 48; i19++) {
        llr_in_data[llr_in_size_idx_0_tmp * i19] = 2.0 * (sym_out[i19].re *
          1.4142135623730951);
      }

      for (i19 = 0; i19 < 48; i19++) {
        llr_in_data[1 + llr_in_size_idx_0_tmp * i19] = 2.0 * (sym_out[i19].im *
          1.4142135623730951);
      }
      break;

     case 4:
      /*  16-QAM */
      for (i19 = 0; i19 < 48; i19++) {
        b_sym_out[i19] = sym_out[i19].re * 3.1622776601683795;
      }

      qam16_demap(b_sym_out, dv14);
      for (i19 = 0; i19 < 48; i19++) {
        llr_in_data_tmp = i19 << 1;
        i = llr_in_size_idx_0_tmp * i19;
        llr_in_data[i] = dv14[llr_in_data_tmp] / 10.0;
        llr_in_data[1 + i] = dv14[1 + llr_in_data_tmp] / 10.0;
        b_sym_out[i19] = sym_out[i19].im * 3.1622776601683795;
      }

      qam16_demap(b_sym_out, dv14);
      for (i19 = 0; i19 < 48; i19++) {
        llr_in_data_tmp = i19 << 1;
        i = llr_in_size_idx_0_tmp * i19;
        llr_in_data[i + 2] = dv14[llr_in_data_tmp] / 10.0;
        llr_in_data[i + 3] = dv14[1 + llr_in_data_tmp] / 10.0;
      }
      break;

     case 6:
      /*  64-QAM */
      for (i19 = 0; i19 < 48; i19++) {
        b_sym_out[i19] = sym_out[i19].re * 6.48074069840786;
      }

      qam64_demap(b_sym_out, bits_out_data);
      for (i19 = 0; i19 < 48; i19++) {
        llr_in_data_tmp = llr_in_size_idx_0_tmp * i19;
        llr_in_data[llr_in_data_tmp] = bits_out_data[3 * i19] / 42.0;
        llr_in_data[1 + llr_in_data_tmp] = bits_out_data[1 + 3 * i19] / 42.0;
        llr_in_data[2 + llr_in_data_tmp] = bits_out_data[2 + 3 * i19] / 42.0;
        b_sym_out[i19] = sym_out[i19].im * 6.48074069840786;
      }

      qam64_demap(b_sym_out, bits_out_data);
      for (i19 = 0; i19 < 48; i19++) {
        llr_in_data_tmp = llr_in_size_idx_0_tmp * i19;
        llr_in_data[llr_in_data_tmp + 3] = bits_out_data[3 * i19] / 42.0;
        llr_in_data[llr_in_data_tmp + 4] = bits_out_data[1 + 3 * i19] / 42.0;
        llr_in_data[llr_in_data_tmp + 5] = bits_out_data[2 + 3 * i19] / 42.0;
      }
      break;
    }

    /*  Negative LLR values */
    for (i19 = 0; i19 < 48; i19++) {
      for (ar_tmp = 0; ar_tmp < llr_in_size_idx_0_tmp; ar_tmp++) {
        llr_in_data_tmp = ar_tmp + llr_in_size_idx_0_tmp * i19;
        llr_in_data[llr_in_data_tmp] = -llr_in_data[llr_in_data_tmp] * snr[i19];
      }
    }

    /*  Deinterleaving */
    /* DEINTERLEAVER Bit deinterleaver */
    /*  */
    /*    Author: Ioannis Sarris, u-blox */
    /*    email: ioannis.sarris@u-blox.com */
    /*    August 2018; Last revision: 30-August-2018 */
    /*  Copyright (C) u-blox */
    /*  */
    /*  All rights reserved. */
    /*  */
    /*  Permission to use, copy, modify, and distribute this software for any */
    /*  purpose without fee is hereby granted, provided that this entire notice */
    /*  is included in all copies of any software which is or includes a copy */
    /*  or modification of this software and in all copies of the supporting */
    /*  documentation for such software. */
    /*  */
    /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
    /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
    /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
    /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
    /*  */
    /*  Project: ubx-v2x */
    /*  Purpose: V2X baseband simulation model */
    /*  s-parameter */
    /*  Initialize output */
    if (0 <= loop_ub_tmp - 1) {
      memset(&x_data_data[0], 0, (unsigned int)(loop_ub_tmp * (int)sizeof(double)));
    }

    /*  Input index */
    if (rtIsNaN(SIG_CFG_n_cbps - 1.0)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i19);
      y->data[0] = rtNaN;
    } else if (SIG_CFG_n_cbps - 1.0 < 0.0) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else if (rtIsInf(SIG_CFG_n_cbps - 1.0) && (0.0 == SIG_CFG_n_cbps - 1.0)) {
      i19 = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = 1;
      emxEnsureCapacity_real_T(y, i19);
      y->data[0] = rtNaN;
    } else {
      i19 = y->size[0] * y->size[1];
      y->size[0] = 1;
      i = (int)std::floor(SIG_CFG_n_cbps - 1.0);
      y->size[1] = i + 1;
      emxEnsureCapacity_real_T(y, i19);
      for (i19 = 0; i19 <= i; i19++) {
        y->data[i19] = i19;
      }
    }

    /*  First permutation */
    d_mod(y->data, y->size, ii_data, ii_size);
    tmp_size[0] = 1;
    tmp_size[1] = y->size[1];
    i = y->size[0] * y->size[1];
    for (i19 = 0; i19 < i; i19++) {
      tmp_data[i19] = y->data[i19] / 16.0;
    }

    b_floor(tmp_data, tmp_size);
    i = ii_size[0] * ii_size[1] - 1;
    for (i19 = 0; i19 <= i; i19++) {
      ii_data[i19] = a * ii_data[i19] + tmp_data[i19];
    }

    /*  Second permutation */
    /*  Deinterleaver mapping */
    i = y->size[0] * y->size[1];
    for (i19 = 0; i19 < i; i19++) {
      b_tmp_data[i19] = (short)(y->data[i19] + 1.0);
    }

    tmp_size[0] = 1;
    tmp_size[1] = ii_size[1];
    i = ii_size[1];
    for (i19 = 0; i19 < i; i19++) {
      tmp_data[i19] = ii_data[i19] / s;
    }

    b_floor(tmp_data, tmp_size);
    b_tmp_size[0] = 1;
    b_tmp_size[1] = ii_size[1];
    i = ii_size[1];
    for (i19 = 0; i19 < i; i19++) {
      c_tmp_data[i19] = 16.0 * ii_data[i19] / SIG_CFG_n_cbps;
    }

    b_floor(c_tmp_data, b_tmp_size);
    b_ii_size[0] = 1;
    b_ii_size[1] = ii_size[1];
    i = ii_size[1];
    for (i19 = 0; i19 < i; i19++) {
      b_ii_data[i19] = (ii_data[i19] + SIG_CFG_n_cbps) - c_tmp_data[i19];
    }

    e_mod(b_ii_data, b_ii_size, s, c_tmp_data, b_tmp_size);
    i = tmp_size[0] * tmp_size[1];
    for (i19 = 0; i19 < i; i19++) {
      x_data_data[b_tmp_data[i19] - 1] = llr_in_data[(int)((s * tmp_data[i19] +
        c_tmp_data[i19]) + 1.0) - 1];
    }

    /*  Store output binary data per OFDM symbol */
    if (0 <= loop_ub_tmp - 1) {
      memcpy(&llr_in_dep_data[0], &x_data_data[0], (unsigned int)(loop_ub_tmp *
              (int)sizeof(double)));
    }

    /* BCC_DEC Decodes BCC encoded LLR stream */
    /*  */
    /*    Author: Ioannis Sarris, u-blox */
    /*    email: ioannis.sarris@u-blox.com */
    /*    August 2018; Last revision: 30-August-2018 */
    /*  Copyright (C) u-blox */
    /*  */
    /*  All rights reserved. */
    /*  */
    /*  Permission to use, copy, modify, and distribute this software for any */
    /*  purpose without fee is hereby granted, provided that this entire notice */
    /*  is included in all copies of any software which is or includes a copy */
    /*  or modification of this software and in all copies of the supporting */
    /*  documentation for such software. */
    /*  */
    /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
    /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
    /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
    /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
    /*  */
    /*  Project: ubx-v2x */
    /*  Purpose: V2X baseband simulation model */
    /*  Store this as a persistent variable to avoid reinitialization */
    /*  Needed for code generation */
    /*  Create or reset system object */
    if (!vit_dec_not_empty) {
      b_ViterbiDecoder_ViterbiDecoder(&vit_dec);
      vit_dec_not_empty = true;
    } else {
      if (1U + i_sym == 1U) {
        SystemCore_reset(&vit_dec);
      }
    }

    /*  Select Viterbi decoder depuncturing pattern according to code-rate */
    switch ((int)SIG_CFG_r_num) {
     case 2:
      punct_pat_size[0] = 1;
      punct_pat_size[1] = 4;
      punct_pat_data[0] = true;
      punct_pat_data[1] = true;
      punct_pat_data[2] = true;
      punct_pat_data[3] = false;
      r = 1.3333333333333333;
      break;

     case 3:
      punct_pat_size[0] = 1;
      punct_pat_size[1] = 6;
      for (i19 = 0; i19 < 6; i19++) {
        punct_pat_data[i19] = bv0[i19];
      }

      r = 1.5;
      break;

     default:
      punct_pat_size[0] = 1;
      punct_pat_size[1] = 1;
      punct_pat_data[0] = true;
      r = 1.0;
      break;
    }

    /*  Input codeword length */
    /*  Repeat depuncturing pattern as many times as needed to cover whole output codeword */
    repmat(punct_pat_data, punct_pat_size, (double)llr_in_dep_size * r / (double)
           punct_pat_size[1], idx_data, ii_size);

    /*  Depuncturing */
    i = (int)rt_roundd_snf((double)llr_in_dep_size * r / 2.0) << 1;
    c_llr_in_dep_size[0] = i;
    if (0 <= i - 1) {
      memset(&b_llr_in_dep_data[0], 0, (unsigned int)(i * (int)sizeof(double)));
    }

    ar_tmp = ii_size[1];
    llr_in_data_tmp = 0;
    for (i = 0; i < ar_tmp; i++) {
      if (idx_data[i]) {
        b_llr_in_dep_data[i] = llr_in_dep_data[llr_in_data_tmp];
        llr_in_data_tmp++;
      }
    }

    /*  Viterbi decoder rate 1/2 */
    SystemCore_step(&vit_dec, b_llr_in_dep_data, c_llr_in_dep_size,
                    b_bits_out_data, ii_size);
    i21 = (short)data_out_vec->size[0];
    i = i21 - 1;
    for (i19 = 0; i19 <= i; i19++) {
      d_tmp_data[i19] = (short)i19;
    }

    i = i21;
    for (i19 = 0; i19 < i; i19++) {
      data_out_vec->data[d_tmp_data[i19] + data_out_vec->size[0] * i_sym] =
        b_bits_out_data[i19];
    }
  }

  emxFree_real_T(&y);

  /*  Last pass of Viterbi decoder */
  /* BCC_DEC Decodes BCC encoded LLR stream */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    August 2018; Last revision: 30-August-2018 */
  /*  Copyright (C) u-blox */
  /*  */
  /*  All rights reserved. */
  /*  */
  /*  Permission to use, copy, modify, and distribute this software for any */
  /*  purpose without fee is hereby granted, provided that this entire notice */
  /*  is included in all copies of any software which is or includes a copy */
  /*  or modification of this software and in all copies of the supporting */
  /*  documentation for such software. */
  /*  */
  /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
  /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
  /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
  /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
  /*  */
  /*  Project: ubx-v2x */
  /*  Purpose: V2X baseband simulation model */
  /*  Store this as a persistent variable to avoid reinitialization */
  /*  Needed for code generation */
  /*  Create or reset system object */
  if (!vit_dec_not_empty) {
    b_ViterbiDecoder_ViterbiDecoder(&vit_dec);
    vit_dec_not_empty = true;
  }

  /*  Select Viterbi decoder depuncturing pattern according to code-rate */
  switch ((int)SIG_CFG_r_num) {
   case 2:
    punct_pat_size[0] = 1;
    punct_pat_size[1] = 4;
    punct_pat_data[0] = true;
    punct_pat_data[1] = true;
    punct_pat_data[2] = true;
    punct_pat_data[3] = false;
    r = 1.3333333333333333;
    break;

   case 3:
    punct_pat_size[0] = 1;
    punct_pat_size[1] = 6;
    for (i19 = 0; i19 < 6; i19++) {
      punct_pat_data[i19] = bv0[i19];
    }

    r = 1.5;
    break;

   default:
    punct_pat_size[0] = 1;
    punct_pat_size[1] = 1;
    punct_pat_data[0] = true;
    r = 1.0;
    break;
  }

  /*  Input codeword length */
  /*  Repeat depuncturing pattern as many times as needed to cover whole output codeword */
  repmat(punct_pat_data, punct_pat_size, 192.0 * r / (double)punct_pat_size[1],
         idx_data, ii_size);
  i = ii_size[0] * ii_size[1];
  if (0 <= i - 1) {
    memcpy(&b_idx_data[0], &idx_data[0], (unsigned int)(i * (int)sizeof
            (boolean_T)));
  }

  /*  Depuncturing */
  i = (int)rt_roundd_snf(192.0 * r / 2.0) << 1;
  b_llr_in_dep_size[0] = i;
  memset(&llr_in_dep_data[0], 0, (unsigned int)(i * (int)sizeof(double)));
  ar_tmp = ii_size[1];
  for (i = 0; i < ar_tmp; i++) {
    if (b_idx_data[i]) {
      llr_in_dep_data[i] = 0.0;
    }
  }

  emxInit_real_T(&data_out, 1);

  /*  Viterbi decoder rate 1/2 */
  b_SystemCore_step(&vit_dec, llr_in_dep_data, b_llr_in_dep_size, bits_out_data,
                    ii_size);
  i19 = data_out->size[0];
  data_out->size[0] = data_out_vec->size[0] * data_out_vec->size[1] + ii_size[0];
  emxEnsureCapacity_real_T(data_out, i19);
  i = data_out_vec->size[0] * data_out_vec->size[1];
  for (i19 = 0; i19 < i; i19++) {
    data_out->data[i19] = data_out_vec->data[i19];
  }

  i = ii_size[0];
  for (i19 = 0; i19 < i; i19++) {
    data_out->data[i19 + data_out_vec->size[0] * data_out_vec->size[1]] =
      bits_out_data[i19];
  }

  emxFree_real_T(&data_out_vec);

  /*  Descrambling */
  if (97 > data_out->size[0]) {
    i19 = 0;
    ar_tmp = 0;
  } else {
    i19 = 96;
    ar_tmp = data_out->size[0];
  }

  emxInit_boolean_T(&in, 1);
  i20 = in->size[0];
  i = ar_tmp - i19;
  in->size[0] = i;
  emxEnsureCapacity_boolean_T(in, i20);
  for (ar_tmp = 0; ar_tmp < i; ar_tmp++) {
    in->data[ar_tmp] = (data_out->data[i19 + ar_tmp] != 0.0);
  }

  emxFree_real_T(&data_out);

  /* DESCRAMBLER Bit descrambler */
  /*  */
  /*    Author: Ioannis Sarris, u-blox */
  /*    email: ioannis.sarris@u-blox.com */
  /*    August 2018; Last revision: 30-August-2018 */
  /*  Copyright (C) u-blox */
  /*  */
  /*  All rights reserved. */
  /*  */
  /*  Permission to use, copy, modify, and distribute this software for any */
  /*  purpose without fee is hereby granted, provided that this entire notice */
  /*  is included in all copies of any software which is or includes a copy */
  /*  or modification of this software and in all copies of the supporting */
  /*  documentation for such software. */
  /*  */
  /*  THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED */
  /*  WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY */
  /*  REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY */
  /*  OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. */
  /*  */
  /*  Project: ubx-v2x */
  /*  Purpose: V2X baseband simulation model */
  /*  Initialize descrambler if not initialized */
  if (8 > in->size[0]) {
    i19 = 0;
    ar_tmp = 0;
  } else {
    i19 = 7;
    ar_tmp = in->size[0];
  }

  /*  Perform scrambling (identical to descrabling) */
  for (i20 = 0; i20 < 7; i20++) {
    pn_seq[i20] = in->data[6 - i20];
  }

  emxInit_boolean_T(&b_in, 1);
  i20 = b_in->size[0];
  i = ar_tmp - i19;
  b_in->size[0] = i;
  emxEnsureCapacity_boolean_T(b_in, i20);
  for (ar_tmp = 0; ar_tmp < i; ar_tmp++) {
    b_in->data[ar_tmp] = in->data[i19 + ar_tmp];
  }

  emxFree_boolean_T(&in);
  scrambler_tx(b_in, pn_seq, descr_msg);
  emxFree_boolean_T(&b_in);
}

/* End of code generation (data_rx.cpp) */
