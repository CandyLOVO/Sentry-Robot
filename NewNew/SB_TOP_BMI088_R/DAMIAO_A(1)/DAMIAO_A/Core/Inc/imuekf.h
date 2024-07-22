/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: imuekf.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Mar-2024 14:45:12
 */

#ifndef IMUEKF_H
#define IMUEKF_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void imuekf(double gx, double gy, double gz, double ax, double ay,
                   double az, double p[36], const double xhat_o[6],double dt, double *Yaw,
                   double *Pitch, double *Roll, double xhat[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for imuekf.h
 *
 * [EOF]
 */
