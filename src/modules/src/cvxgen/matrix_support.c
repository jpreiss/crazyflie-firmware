/* Produced by CVXGEN, 2017-08-04 21:41:33 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(float *lhs, float *rhs) {
}
void multbymAT(float *lhs, float *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
}
void multbymG(float *lhs, float *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[4]*(-1);
  lhs[5] = -rhs[5]*(-1);
  lhs[6] = -rhs[0]*(1);
  lhs[7] = -rhs[1]*(1);
  lhs[8] = -rhs[2]*(1);
  lhs[9] = -rhs[3]*(1);
  lhs[10] = -rhs[4]*(1);
  lhs[11] = -rhs[5]*(1);
}
void multbymGT(float *lhs, float *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[6]*(1);
  lhs[1] = -rhs[1]*(-1)-rhs[7]*(1);
  lhs[2] = -rhs[2]*(-1)-rhs[8]*(1);
  lhs[3] = -rhs[3]*(-1)-rhs[9]*(1);
  lhs[4] = -rhs[4]*(-1)-rhs[10]*(1);
  lhs[5] = -rhs[5]*(-1)-rhs[11]*(1);
}
void multbyP(float *lhs, float *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*work.quad_601932361728[0])+rhs[1]*(2*work.quad_601932361728[6])+rhs[2]*(2*work.quad_601932361728[12])+rhs[3]*(2*work.quad_601932361728[18])+rhs[4]*(2*work.quad_601932361728[24])+rhs[5]*(2*work.quad_601932361728[30]);
  lhs[1] = rhs[0]*(2*work.quad_601932361728[1])+rhs[1]*(2*work.quad_601932361728[7])+rhs[2]*(2*work.quad_601932361728[13])+rhs[3]*(2*work.quad_601932361728[19])+rhs[4]*(2*work.quad_601932361728[25])+rhs[5]*(2*work.quad_601932361728[31]);
  lhs[2] = rhs[0]*(2*work.quad_601932361728[2])+rhs[1]*(2*work.quad_601932361728[8])+rhs[2]*(2*work.quad_601932361728[14])+rhs[3]*(2*work.quad_601932361728[20])+rhs[4]*(2*work.quad_601932361728[26])+rhs[5]*(2*work.quad_601932361728[32]);
  lhs[3] = rhs[0]*(2*work.quad_601932361728[3])+rhs[1]*(2*work.quad_601932361728[9])+rhs[2]*(2*work.quad_601932361728[15])+rhs[3]*(2*work.quad_601932361728[21])+rhs[4]*(2*work.quad_601932361728[27])+rhs[5]*(2*work.quad_601932361728[33]);
  lhs[4] = rhs[0]*(2*work.quad_601932361728[4])+rhs[1]*(2*work.quad_601932361728[10])+rhs[2]*(2*work.quad_601932361728[16])+rhs[3]*(2*work.quad_601932361728[22])+rhs[4]*(2*work.quad_601932361728[28])+rhs[5]*(2*work.quad_601932361728[34]);
  lhs[5] = rhs[0]*(2*work.quad_601932361728[5])+rhs[1]*(2*work.quad_601932361728[11])+rhs[2]*(2*work.quad_601932361728[17])+rhs[3]*(2*work.quad_601932361728[23])+rhs[4]*(2*work.quad_601932361728[29])+rhs[5]*(2*work.quad_601932361728[35]);
}
void fillq(void) {
  work.q[0] = -2*(params.A[0]*params.b[0]+params.A[1]*params.b[1]+params.A[2]*params.b[2]+params.A[3]*params.b[3]+params.A[4]*params.b[4]+params.A[5]*params.b[5]);
  work.q[1] = -2*(params.A[6]*params.b[0]+params.A[7]*params.b[1]+params.A[8]*params.b[2]+params.A[9]*params.b[3]+params.A[10]*params.b[4]+params.A[11]*params.b[5]);
  work.q[2] = -2*(params.A[12]*params.b[0]+params.A[13]*params.b[1]+params.A[14]*params.b[2]+params.A[15]*params.b[3]+params.A[16]*params.b[4]+params.A[17]*params.b[5]);
  work.q[3] = -2*(params.A[18]*params.b[0]+params.A[19]*params.b[1]+params.A[20]*params.b[2]+params.A[21]*params.b[3]+params.A[22]*params.b[4]+params.A[23]*params.b[5]);
  work.q[4] = -2*(params.A[24]*params.b[0]+params.A[25]*params.b[1]+params.A[26]*params.b[2]+params.A[27]*params.b[3]+params.A[28]*params.b[4]+params.A[29]*params.b[5]);
  work.q[5] = -2*(params.A[30]*params.b[0]+params.A[31]*params.b[1]+params.A[32]*params.b[2]+params.A[33]*params.b[3]+params.A[34]*params.b[4]+params.A[35]*params.b[5]);
}
void fillh(void) {
  work.h[0] = 0;
  work.h[1] = 0;
  work.h[2] = 0;
  work.h[3] = 0;
  work.h[4] = 0;
  work.h[5] = 0;
  work.h[6] = 1;
  work.h[7] = 1;
  work.h[8] = 1;
  work.h[9] = 1;
  work.h[10] = 1;
  work.h[11] = 1;
}
void fillb(void) {
}
void pre_ops(void) {
  work.quad_601932361728[0] = params.A[0]*params.A[0]+params.A[1]*params.A[1]+params.A[2]*params.A[2]+params.A[3]*params.A[3]+params.A[4]*params.A[4]+params.A[5]*params.A[5];
  work.quad_601932361728[6] = params.A[0]*params.A[6]+params.A[1]*params.A[7]+params.A[2]*params.A[8]+params.A[3]*params.A[9]+params.A[4]*params.A[10]+params.A[5]*params.A[11];
  work.quad_601932361728[12] = params.A[0]*params.A[12]+params.A[1]*params.A[13]+params.A[2]*params.A[14]+params.A[3]*params.A[15]+params.A[4]*params.A[16]+params.A[5]*params.A[17];
  work.quad_601932361728[18] = params.A[0]*params.A[18]+params.A[1]*params.A[19]+params.A[2]*params.A[20]+params.A[3]*params.A[21]+params.A[4]*params.A[22]+params.A[5]*params.A[23];
  work.quad_601932361728[24] = params.A[0]*params.A[24]+params.A[1]*params.A[25]+params.A[2]*params.A[26]+params.A[3]*params.A[27]+params.A[4]*params.A[28]+params.A[5]*params.A[29];
  work.quad_601932361728[30] = params.A[0]*params.A[30]+params.A[1]*params.A[31]+params.A[2]*params.A[32]+params.A[3]*params.A[33]+params.A[4]*params.A[34]+params.A[5]*params.A[35];
  work.quad_601932361728[1] = params.A[6]*params.A[0]+params.A[7]*params.A[1]+params.A[8]*params.A[2]+params.A[9]*params.A[3]+params.A[10]*params.A[4]+params.A[11]*params.A[5];
  work.quad_601932361728[7] = params.A[6]*params.A[6]+params.A[7]*params.A[7]+params.A[8]*params.A[8]+params.A[9]*params.A[9]+params.A[10]*params.A[10]+params.A[11]*params.A[11];
  work.quad_601932361728[13] = params.A[6]*params.A[12]+params.A[7]*params.A[13]+params.A[8]*params.A[14]+params.A[9]*params.A[15]+params.A[10]*params.A[16]+params.A[11]*params.A[17];
  work.quad_601932361728[19] = params.A[6]*params.A[18]+params.A[7]*params.A[19]+params.A[8]*params.A[20]+params.A[9]*params.A[21]+params.A[10]*params.A[22]+params.A[11]*params.A[23];
  work.quad_601932361728[25] = params.A[6]*params.A[24]+params.A[7]*params.A[25]+params.A[8]*params.A[26]+params.A[9]*params.A[27]+params.A[10]*params.A[28]+params.A[11]*params.A[29];
  work.quad_601932361728[31] = params.A[6]*params.A[30]+params.A[7]*params.A[31]+params.A[8]*params.A[32]+params.A[9]*params.A[33]+params.A[10]*params.A[34]+params.A[11]*params.A[35];
  work.quad_601932361728[2] = params.A[12]*params.A[0]+params.A[13]*params.A[1]+params.A[14]*params.A[2]+params.A[15]*params.A[3]+params.A[16]*params.A[4]+params.A[17]*params.A[5];
  work.quad_601932361728[8] = params.A[12]*params.A[6]+params.A[13]*params.A[7]+params.A[14]*params.A[8]+params.A[15]*params.A[9]+params.A[16]*params.A[10]+params.A[17]*params.A[11];
  work.quad_601932361728[14] = params.A[12]*params.A[12]+params.A[13]*params.A[13]+params.A[14]*params.A[14]+params.A[15]*params.A[15]+params.A[16]*params.A[16]+params.A[17]*params.A[17];
  work.quad_601932361728[20] = params.A[12]*params.A[18]+params.A[13]*params.A[19]+params.A[14]*params.A[20]+params.A[15]*params.A[21]+params.A[16]*params.A[22]+params.A[17]*params.A[23];
  work.quad_601932361728[26] = params.A[12]*params.A[24]+params.A[13]*params.A[25]+params.A[14]*params.A[26]+params.A[15]*params.A[27]+params.A[16]*params.A[28]+params.A[17]*params.A[29];
  work.quad_601932361728[32] = params.A[12]*params.A[30]+params.A[13]*params.A[31]+params.A[14]*params.A[32]+params.A[15]*params.A[33]+params.A[16]*params.A[34]+params.A[17]*params.A[35];
  work.quad_601932361728[3] = params.A[18]*params.A[0]+params.A[19]*params.A[1]+params.A[20]*params.A[2]+params.A[21]*params.A[3]+params.A[22]*params.A[4]+params.A[23]*params.A[5];
  work.quad_601932361728[9] = params.A[18]*params.A[6]+params.A[19]*params.A[7]+params.A[20]*params.A[8]+params.A[21]*params.A[9]+params.A[22]*params.A[10]+params.A[23]*params.A[11];
  work.quad_601932361728[15] = params.A[18]*params.A[12]+params.A[19]*params.A[13]+params.A[20]*params.A[14]+params.A[21]*params.A[15]+params.A[22]*params.A[16]+params.A[23]*params.A[17];
  work.quad_601932361728[21] = params.A[18]*params.A[18]+params.A[19]*params.A[19]+params.A[20]*params.A[20]+params.A[21]*params.A[21]+params.A[22]*params.A[22]+params.A[23]*params.A[23];
  work.quad_601932361728[27] = params.A[18]*params.A[24]+params.A[19]*params.A[25]+params.A[20]*params.A[26]+params.A[21]*params.A[27]+params.A[22]*params.A[28]+params.A[23]*params.A[29];
  work.quad_601932361728[33] = params.A[18]*params.A[30]+params.A[19]*params.A[31]+params.A[20]*params.A[32]+params.A[21]*params.A[33]+params.A[22]*params.A[34]+params.A[23]*params.A[35];
  work.quad_601932361728[4] = params.A[24]*params.A[0]+params.A[25]*params.A[1]+params.A[26]*params.A[2]+params.A[27]*params.A[3]+params.A[28]*params.A[4]+params.A[29]*params.A[5];
  work.quad_601932361728[10] = params.A[24]*params.A[6]+params.A[25]*params.A[7]+params.A[26]*params.A[8]+params.A[27]*params.A[9]+params.A[28]*params.A[10]+params.A[29]*params.A[11];
  work.quad_601932361728[16] = params.A[24]*params.A[12]+params.A[25]*params.A[13]+params.A[26]*params.A[14]+params.A[27]*params.A[15]+params.A[28]*params.A[16]+params.A[29]*params.A[17];
  work.quad_601932361728[22] = params.A[24]*params.A[18]+params.A[25]*params.A[19]+params.A[26]*params.A[20]+params.A[27]*params.A[21]+params.A[28]*params.A[22]+params.A[29]*params.A[23];
  work.quad_601932361728[28] = params.A[24]*params.A[24]+params.A[25]*params.A[25]+params.A[26]*params.A[26]+params.A[27]*params.A[27]+params.A[28]*params.A[28]+params.A[29]*params.A[29];
  work.quad_601932361728[34] = params.A[24]*params.A[30]+params.A[25]*params.A[31]+params.A[26]*params.A[32]+params.A[27]*params.A[33]+params.A[28]*params.A[34]+params.A[29]*params.A[35];
  work.quad_601932361728[5] = params.A[30]*params.A[0]+params.A[31]*params.A[1]+params.A[32]*params.A[2]+params.A[33]*params.A[3]+params.A[34]*params.A[4]+params.A[35]*params.A[5];
  work.quad_601932361728[11] = params.A[30]*params.A[6]+params.A[31]*params.A[7]+params.A[32]*params.A[8]+params.A[33]*params.A[9]+params.A[34]*params.A[10]+params.A[35]*params.A[11];
  work.quad_601932361728[17] = params.A[30]*params.A[12]+params.A[31]*params.A[13]+params.A[32]*params.A[14]+params.A[33]*params.A[15]+params.A[34]*params.A[16]+params.A[35]*params.A[17];
  work.quad_601932361728[23] = params.A[30]*params.A[18]+params.A[31]*params.A[19]+params.A[32]*params.A[20]+params.A[33]*params.A[21]+params.A[34]*params.A[22]+params.A[35]*params.A[23];
  work.quad_601932361728[29] = params.A[30]*params.A[24]+params.A[31]*params.A[25]+params.A[32]*params.A[26]+params.A[33]*params.A[27]+params.A[34]*params.A[28]+params.A[35]*params.A[29];
  work.quad_601932361728[35] = params.A[30]*params.A[30]+params.A[31]*params.A[31]+params.A[32]*params.A[32]+params.A[33]*params.A[33]+params.A[34]*params.A[34]+params.A[35]*params.A[35];
  work.quad_384992550912[0] = params.b[0]*params.b[0]+params.b[1]*params.b[1]+params.b[2]*params.b[2]+params.b[3]*params.b[3]+params.b[4]*params.b[4]+params.b[5]*params.b[5];
}
