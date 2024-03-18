/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: imuekf.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 06-Mar-2024 19:47:09
 */

/* Include Files */
#include "imuekf.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "arm_math.h"

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2(i, i1);
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
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/*
 * 定义各卡尔曼矩阵
 *
 * Arguments    : double gx
 *                double gy
 *                double gz
 *                double ax
 *                double ay
 *                double az
 *                double p[36]
 *                const double xhat_o[6]
 *                double dt
 *                double *Yaw
 *                double *Pitch
 *                double *Roll
 *                double xhat[6]
 * Return Type  : void
 */
void imuekf(double gx, double gy, double gz, double ax, double ay, double az,
            double p[36], const double xhat_o[6], double dt, double *Yaw,
            double *Pitch, double *Roll, double xhat[6])
{
  static const double Q[36] = {
      0.02, 0.0, 0.0,   0.0, 0.0,    0.0, 0.0, 0.02, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.02, 0.0, 0.0,    0.0, 0.0, 0.0,   0.0, 0.02, 0.0, 0.0,
      0.0,   0.0, 0.0,   0.0, 1.0E-5, 0.0, 0.0, 0.0,   0.0, 0.0,   0.0, 1.0E-5};
  static const long long R[9] = {200000000, 0, 0, 0, 200000000, 0, 0, 0, 200000000000};
  static const signed char a[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv[6] = {0, 0, 0, 0, 1, 0};
  static const signed char iv1[6] = {0, 0, 0, 0, 0, 1};
  double A[36];
  double b_A[36];
  double H[18];
  double b_y_tmp[18];
  double y_tmp[18];
  double dv[16];
  double c_A[9];
  double Y[3];
  double ek[3];
  double a21;
  double maxval;
  double xhat_o_idx_2;
  double xhat_o_idx_3;
  int H_tmp;
  int b_H_tmp;
  int k;
  int r1;
  int r2;
  int r3;
  int rtemp;
  /*  p = [100000, 0.1, 0.1, 0.1, 0.1, 0.1; */
  /*                                   0.1, 100000, 0.1, 0.1, 0.1, 0.1; */
  /*                                   0.1, 0.1, 100000, 0.1, 0.1, 0.1; */
  /*                                   0.1, 0.1, 0.1, 100000, 0.1, 0.1; */
  /*                                   0.1, 0.1, 0.1, 0.1, 10000, 0.1; */
  /*                                   0.1, 0.1, 0.1, 0.1, 0.1, 10000]; */
  /* 先验估计 */
  /*  xhat = [0; 0; 0; 0; 0; 0]; */
  /*  xhat(1,1) = 1; */
  /*  xhat(2,1)= 0; */
  /*  xhat(3,1) = 0; */
  /*  xhat(4,1) = 0; */
  /*  inct = (xhat(1,1)^2 + xhat(2,1)^2 + xhat(3,1)^2 + xhat(4,1)^2)^(1/2); */
  /*  xhat(1,1) = xhat(1,1)*1/inct; */
  /*  xhat(2,1)= xhat(2,1)*1/inct; */
  /*  xhat(3,1) = xhat(3,1)*1/inct; */
  /*  xhat(4,1) = xhat(4,1)*1/inct; */
  /* xhat = [xhat(1,1) ;xhat(2,1); xhat(3,1); xhat(4,1); bx; by]; */
  /* 假设陀螺仪数据输入 */
  /*  gx = 0; */
  /*  gy = 0; */
  /*  gz = 0; */
  /* 陀螺仪数据传入 */
  dv[0] = 0.0;
  maxval = 0.5 * -gx;
  dv[4] = maxval;
  a21 = 0.5 * -gy;
  dv[8] = a21;
  xhat_o_idx_2 = 0.5 * -gz;
  dv[12] = xhat_o_idx_2;
  dv[1] = 0.5 * gx;
  dv[5] = 0.0;
  dv[9] = 0.5 * gz;
  dv[13] = a21;
  dv[2] = 0.5 * gy;
  dv[6] = xhat_o_idx_2;
  dv[10] = 0.0;
  dv[14] = 0.5 * gx;
  dv[3] = 0.5 * gz;
  dv[7] = 0.5 * gy;
  dv[11] = maxval;
  dv[15] = 0.0;
  maxval = xhat_o[0];
  a21 = xhat_o[1];
  xhat_o_idx_2 = xhat_o[2];
  xhat_o_idx_3 = xhat_o[3];
  for (H_tmp = 0; H_tmp < 4; H_tmp++) {
    xhat[H_tmp] = xhat_o[H_tmp] + (((dv[H_tmp] * maxval + dv[H_tmp + 4] * a21) +
                                    dv[H_tmp + 8] * xhat_o_idx_2) +
                                   dv[H_tmp + 12] * xhat_o_idx_3) *
                                      dt;
  }
  double A_tmp;
  double b_A_tmp;
  double c_A_tmp;
  double d_A_tmp;
  double e_A_tmp;
  xhat[4] = xhat_o[4];
  xhat[5] = xhat_o[5];
  /* xhat = [xhat(1,1) + dt*(xhat(2,1)*(bx/2 - gx/2) - (gz*xhat(4,1))/2 +
   * xhat(3,1)*(by/2 - gy/2)); xhat(2,1)+ dt*((gz*xhat(3,1))/2 - xhat(1,1)*(bx/2
   * - gx/2) + xhat(4,1)*(by/2 - gy/2)); xhat(3,1) - dt*((gz*xhat(2,1))/2 +
   * xhat(4,1)*(bx/2 - gx/2) + xhat(1,1)*(by/2 - gy/2)); xhat(4,1) +
   * dt*((gz*xhat(1,1))/2 + xhat(3,1)*(bx/2 - gx/2) - xhat(2,1)*(by/2 - gy/2));
   * bx; by]; */
  /* 雅可比矩阵A */
  A[0] = 1.0;
  maxval = dt * (0.0 - gx / 2.0);
  A[6] = maxval;
  a21 = dt * (0.0 - gy / 2.0);
  A[12] = a21;
  xhat_o_idx_2 = dt * gz;
  xhat_o_idx_3 = -xhat_o_idx_2 / 2.0;
  A[18] = xhat_o_idx_3;
  A_tmp = dt * xhat[1];
  A[24] = A_tmp / 2.0;
  b_A_tmp = dt * xhat[2] / 2.0;
  A[30] = b_A_tmp;
  c_A_tmp = -(dt * (0.0 - gx)) / 2.0;
  A[1] = c_A_tmp;
  A[7] = 1.0;
  xhat_o_idx_2 /= 2.0;
  A[13] = xhat_o_idx_2;
  A[19] = a21;
  a21 = -(dt * xhat[0]) / 2.0;
  A[25] = a21;
  d_A_tmp = dt * xhat[3];
  A[31] = d_A_tmp / 2.0;
  e_A_tmp = -(dt * (0.0 - gy)) / 2.0;
  A[2] = e_A_tmp;
  A[8] = xhat_o_idx_3;
  A[14] = 1.0;
  A[20] = c_A_tmp;
  A[26] = -d_A_tmp / 2.0;
  A[32] = a21;
  A[3] = xhat_o_idx_2;
  A[9] = e_A_tmp;
  A[15] = maxval;
  A[21] = 1.0;
  A[27] = b_A_tmp;
  A[33] = -A_tmp / 2.0;
  for (H_tmp = 0; H_tmp < 6; H_tmp++) {
    A[6 * H_tmp + 4] = iv[H_tmp];
    A[6 * H_tmp + 5] = iv1[H_tmp];
  }
  /* 雅可比矩阵H */
  H[0] = -2.0 * xhat[2];
  H[3] = 2.0 * xhat[3];
  H[6] = -2.0 * xhat[0];
  H[9] = 2.0 * xhat[1];
  H[12] = 0.0;
  H[15] = 0.0;
  H[1] = 2.0 * xhat[1];
  H[4] = 2.0 * xhat[0];
  H[7] = 2.0 * xhat[3];
  H[10] = 2.0 * xhat[2];
  H[13] = 0.0;
  H[16] = 0.0;
  H[2] = 2.0 * xhat[0];
  H[5] = -2.0 * xhat[1];
  H[8] = -2.0 * xhat[2];
  H[11] = 2.0 * xhat[3];
  H[14] = 0.0;
  H[17] = 0.0;
  for (H_tmp = 0; H_tmp < 6; H_tmp++) {
    for (r1 = 0; r1 < 6; r1++) {
      maxval = 0.0;
      for (b_H_tmp = 0; b_H_tmp < 6; b_H_tmp++) {
        maxval += A[H_tmp + 6 * b_H_tmp] * p[b_H_tmp + 6 * r1];
      }
      b_A[H_tmp + 6 * r1] = maxval;
    }
  }
  for (H_tmp = 0; H_tmp < 6; H_tmp++) {
    for (r1 = 0; r1 < 6; r1++) {
      maxval = 0.0;
      for (b_H_tmp = 0; b_H_tmp < 6; b_H_tmp++) {
        maxval += b_A[H_tmp + 6 * b_H_tmp] * A[r1 + 6 * b_H_tmp];
      }
      b_H_tmp = H_tmp + 6 * r1;
      p[b_H_tmp] = maxval + Q[b_H_tmp];
    }
  }
  /* 后验估计 */
  for (H_tmp = 0; H_tmp < 3; H_tmp++) {
    for (r1 = 0; r1 < 6; r1++) {
      rtemp = H_tmp + 3 * r1;
      y_tmp[r1 + 6 * H_tmp] = H[rtemp];
      maxval = 0.0;
      for (b_H_tmp = 0; b_H_tmp < 6; b_H_tmp++) {
        maxval += H[H_tmp + 3 * b_H_tmp] * p[b_H_tmp + 6 * r1];
      }
      b_y_tmp[rtemp] = maxval;
    }
  }
  /* 模拟陀螺仪输入 */
  /*  ax = 0; */
  /*  ay = 0; */
  /*  az = 10; */
  /* 记得归一化 四元数归一化 加速度归一化 */
  maxval = 1/invSqrt((ax * ax + ay * ay) + az * az);
  ek[0] = ax / maxval - 2.0 * (xhat[1] * xhat[3] - xhat[0] * xhat[2]);
  ek[1] = ay / maxval - 2.0 * (xhat[2] * xhat[3] + xhat[0] * xhat[1]);
  ek[2] = az / maxval - (1.0 - 2.0 * (xhat[1] * xhat[1] + xhat[2] * xhat[2]));
  for (H_tmp = 0; H_tmp < 3; H_tmp++) {
    for (r1 = 0; r1 < 3; r1++) {
      maxval = 0.0;
      for (b_H_tmp = 0; b_H_tmp < 6; b_H_tmp++) {
        maxval += b_y_tmp[H_tmp + 3 * b_H_tmp] * y_tmp[b_H_tmp + 6 * r1];
      }
      rtemp = H_tmp + 3 * r1;
      c_A[rtemp] = maxval + (double)R[rtemp];
    }
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(c_A[0]);
  a21 = fabs(c_A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabs(c_A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  c_A[r2] /= c_A[r1];
  c_A[r3] /= c_A[r1];
  c_A[r2 + 3] -= c_A[r2] * c_A[r1 + 3];
  c_A[r3 + 3] -= c_A[r3] * c_A[r1 + 3];
  c_A[r2 + 6] -= c_A[r2] * c_A[r1 + 6];
  c_A[r3 + 6] -= c_A[r3] * c_A[r1 + 6];
  if (fabs(c_A[r3 + 3]) > fabs(c_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  c_A[r3 + 3] /= c_A[r2 + 3];
  c_A[r3 + 6] -= c_A[r3 + 3] * c_A[r2 + 6];
  Y[r1] = ek[0] / c_A[r1];
  Y[r2] = ek[1] - Y[r1] * c_A[r1 + 3];
  Y[r3] = ek[2] - Y[r1] * c_A[r1 + 6];
  Y[r2] /= c_A[r2 + 3];
  Y[r3] -= Y[r2] * c_A[r2 + 6];
  Y[r3] /= c_A[r3 + 6];
  Y[r2] -= Y[r3] * c_A[r3 + 3];
  Y[r1] -= Y[r3] * c_A[r3];
  Y[r1] -= Y[r2] * c_A[r2];
      double gyro_norm = 1.0f / invSqrt(gx * gx +
                                        gy * gy +
                                        gz * gz);
    double accelInvNorm = invSqrt(ax *ax + ay * ay + az * az);
    
    double accl_norm = 1.0f / accelInvNorm;
  if (((Y[0] * ek[0] + Y[1] * ek[1]) + Y[2] * ek[2] < 0.000000001)&&(gyro_norm < 0.3f && accl_norm > 9.8f - 0.5f && accl_norm < 9.8f + 0.5f)) {
    double d_A[18];
    for (H_tmp = 0; H_tmp < 6; H_tmp++) {
      for (r1 = 0; r1 < 3; r1++) {
        maxval = 0.0;
        for (b_H_tmp = 0; b_H_tmp < 6; b_H_tmp++) {
          maxval += p[H_tmp + 6 * b_H_tmp] * y_tmp[b_H_tmp + 6 * r1];
        }
        d_A[H_tmp + 6 * r1] = maxval;
      }
    }
    for (H_tmp = 0; H_tmp < 3; H_tmp++) {
      for (r1 = 0; r1 < 3; r1++) {
        maxval = 0.0;
        for (b_H_tmp = 0; b_H_tmp < 6; b_H_tmp++) {
          maxval += b_y_tmp[H_tmp + 3 * b_H_tmp] * y_tmp[b_H_tmp + 6 * r1];
        }
        rtemp = H_tmp + 3 * r1;
        c_A[rtemp] = maxval + (double)R[rtemp];
      }
    }
    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = fabs(c_A[0]);
    a21 = fabs(c_A[1]);
    if (a21 > maxval) {
      maxval = a21;
      r1 = 1;
      r2 = 0;
    }
    if (fabs(c_A[2]) > maxval) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }
    c_A[r2] /= c_A[r1];
    c_A[r3] /= c_A[r1];
    c_A[r2 + 3] -= c_A[r2] * c_A[r1 + 3];
    c_A[r3 + 3] -= c_A[r3] * c_A[r1 + 3];
    c_A[r2 + 6] -= c_A[r2] * c_A[r1 + 6];
    c_A[r3 + 6] -= c_A[r3] * c_A[r1 + 6];
    if (fabs(c_A[r3 + 3]) > fabs(c_A[r2 + 3])) {
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }
    c_A[r3 + 3] /= c_A[r2 + 3];
    c_A[r3 + 6] -= c_A[r3 + 3] * c_A[r2 + 6];
    for (k = 0; k < 6; k++) {
      rtemp = k + 6 * r1;
      H[rtemp] = d_A[k] / c_A[r1];
      b_H_tmp = k + 6 * r2;
      H[b_H_tmp] = d_A[k + 6] - H[rtemp] * c_A[r1 + 3];
      H_tmp = k + 6 * r3;
      H[H_tmp] = d_A[k + 12] - H[rtemp] * c_A[r1 + 6];
      H[b_H_tmp] /= c_A[r2 + 3];
      H[H_tmp] -= H[b_H_tmp] * c_A[r2 + 6];
      H[H_tmp] /= c_A[r3 + 6];
      H[b_H_tmp] -= H[H_tmp] * c_A[r3 + 3];
      H[rtemp] -= H[H_tmp] * c_A[r3];
      H[rtemp] -= H[b_H_tmp] * c_A[r2];
    }
    for (H_tmp = 0; H_tmp < 6; H_tmp++) {
      maxval = 0.0;
      for (r1 = 0; r1 < 3; r1++) {
        a21 = 0.0;
        for (b_H_tmp = 0; b_H_tmp < 6; b_H_tmp++) {
          a21 += (double)a[H_tmp + 6 * b_H_tmp] * H[b_H_tmp + 6 * r1];
        }
        maxval += a21 * ek[r1];
      }
      xhat[H_tmp] += maxval;
    }
  }
  maxval = 1/invSqrt(((xhat[0] * xhat[0] + xhat[1] * xhat[1]) + xhat[2] * xhat[2]) +
                xhat[3] * xhat[3]);
  xhat[0] /= maxval;
  xhat[1] /= maxval;
  xhat[2] /= maxval;
  xhat[3] /= maxval;
  maxval = xhat[0] * xhat[0];
  *Yaw = atan2f(2.0 * (xhat[0] * xhat[3] + xhat[1] * xhat[2]),
                       2.0 * (maxval + xhat[1] * xhat[1]) - 1.0) *
         57.29577951;
  *Pitch = atan2f(2.0 * (xhat[0] * xhat[1] + xhat[2] * xhat[3]),
                         2.0 * (maxval + xhat[3] * xhat[3]) - 1.0) *
           57.295779513;
  *Roll = asinf(-2.0 * (xhat[1] * xhat[3] - xhat[0] * xhat[2])) * 57.295779513;
}

/*
 * File trailer for imuekf.c
 *
 * [EOF]
 */
