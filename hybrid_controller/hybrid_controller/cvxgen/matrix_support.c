/* Produced by CVXGEN, 2018-12-05 06:56:51 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-(params.dbx[0]*params.g[0]+params.dbx[1]*params.g[1]+params.dbx[2]*params.g[2]))-rhs[1]*(-(params.dbx[0]*params.g[3]+params.dbx[1]*params.g[4]+params.dbx[2]*params.g[5]))-rhs[2]*(-(params.dbx[0]*params.g[6]+params.dbx[1]*params.g[7]+params.dbx[2]*params.g[8]));
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-(params.dbx[0]*params.g[0]+params.dbx[1]*params.g[1]+params.dbx[2]*params.g[2]));
  lhs[1] = -rhs[0]*(-(params.dbx[0]*params.g[3]+params.dbx[1]*params.g[4]+params.dbx[2]*params.g[5]));
  lhs[2] = -rhs[0]*(-(params.dbx[0]*params.g[6]+params.dbx[1]*params.g[7]+params.dbx[2]*params.g[8]));
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.Q[0])+rhs[1]*(2*params.Q[3])+rhs[2]*(2*params.Q[6]);
  lhs[1] = rhs[0]*(2*params.Q[1])+rhs[1]*(2*params.Q[4])+rhs[2]*(2*params.Q[7]);
  lhs[2] = rhs[0]*(2*params.Q[2])+rhs[1]*(2*params.Q[5])+rhs[2]*(2*params.Q[8]);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
}
void fillh(void) {
  work.h[0] = -(params.norm_dbx[0]*(params.W[0]+params.L[0])-params.N[0]*(params.dbt[0]+params.alpha[0])-(params.dbx[0]*params.f[0]+params.dbx[1]*params.f[1]+params.dbx[2]*params.f[2]));
}
void fillb(void) {
}
void pre_ops(void) {
}
