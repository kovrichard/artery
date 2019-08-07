/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * tx_phy_params.h
 *
 * Code generation for function 'tx_phy_params'
 *
 */

#ifndef TX_PHY_PARAMS_H
#define TX_PHY_PARAMS_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "sim_rx_mex_types.h"

/* Function Declarations */
extern void tx_phy_params(double mcs, const unsigned int mac_payload_data[],
  const int mac_payload_size[2], const boolean_T pn_seq[7], struct0_T *PHY,
  boolean_T data_msg_data[], int data_msg_size[1]);

#endif

/* End of code generation (tx_phy_params.h) */
