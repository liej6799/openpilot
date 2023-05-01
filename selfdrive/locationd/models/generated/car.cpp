#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9008082079438066370) {
   out_9008082079438066370[0] = delta_x[0] + nom_x[0];
   out_9008082079438066370[1] = delta_x[1] + nom_x[1];
   out_9008082079438066370[2] = delta_x[2] + nom_x[2];
   out_9008082079438066370[3] = delta_x[3] + nom_x[3];
   out_9008082079438066370[4] = delta_x[4] + nom_x[4];
   out_9008082079438066370[5] = delta_x[5] + nom_x[5];
   out_9008082079438066370[6] = delta_x[6] + nom_x[6];
   out_9008082079438066370[7] = delta_x[7] + nom_x[7];
   out_9008082079438066370[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7705390806256602072) {
   out_7705390806256602072[0] = -nom_x[0] + true_x[0];
   out_7705390806256602072[1] = -nom_x[1] + true_x[1];
   out_7705390806256602072[2] = -nom_x[2] + true_x[2];
   out_7705390806256602072[3] = -nom_x[3] + true_x[3];
   out_7705390806256602072[4] = -nom_x[4] + true_x[4];
   out_7705390806256602072[5] = -nom_x[5] + true_x[5];
   out_7705390806256602072[6] = -nom_x[6] + true_x[6];
   out_7705390806256602072[7] = -nom_x[7] + true_x[7];
   out_7705390806256602072[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4044238657628669408) {
   out_4044238657628669408[0] = 1.0;
   out_4044238657628669408[1] = 0;
   out_4044238657628669408[2] = 0;
   out_4044238657628669408[3] = 0;
   out_4044238657628669408[4] = 0;
   out_4044238657628669408[5] = 0;
   out_4044238657628669408[6] = 0;
   out_4044238657628669408[7] = 0;
   out_4044238657628669408[8] = 0;
   out_4044238657628669408[9] = 0;
   out_4044238657628669408[10] = 1.0;
   out_4044238657628669408[11] = 0;
   out_4044238657628669408[12] = 0;
   out_4044238657628669408[13] = 0;
   out_4044238657628669408[14] = 0;
   out_4044238657628669408[15] = 0;
   out_4044238657628669408[16] = 0;
   out_4044238657628669408[17] = 0;
   out_4044238657628669408[18] = 0;
   out_4044238657628669408[19] = 0;
   out_4044238657628669408[20] = 1.0;
   out_4044238657628669408[21] = 0;
   out_4044238657628669408[22] = 0;
   out_4044238657628669408[23] = 0;
   out_4044238657628669408[24] = 0;
   out_4044238657628669408[25] = 0;
   out_4044238657628669408[26] = 0;
   out_4044238657628669408[27] = 0;
   out_4044238657628669408[28] = 0;
   out_4044238657628669408[29] = 0;
   out_4044238657628669408[30] = 1.0;
   out_4044238657628669408[31] = 0;
   out_4044238657628669408[32] = 0;
   out_4044238657628669408[33] = 0;
   out_4044238657628669408[34] = 0;
   out_4044238657628669408[35] = 0;
   out_4044238657628669408[36] = 0;
   out_4044238657628669408[37] = 0;
   out_4044238657628669408[38] = 0;
   out_4044238657628669408[39] = 0;
   out_4044238657628669408[40] = 1.0;
   out_4044238657628669408[41] = 0;
   out_4044238657628669408[42] = 0;
   out_4044238657628669408[43] = 0;
   out_4044238657628669408[44] = 0;
   out_4044238657628669408[45] = 0;
   out_4044238657628669408[46] = 0;
   out_4044238657628669408[47] = 0;
   out_4044238657628669408[48] = 0;
   out_4044238657628669408[49] = 0;
   out_4044238657628669408[50] = 1.0;
   out_4044238657628669408[51] = 0;
   out_4044238657628669408[52] = 0;
   out_4044238657628669408[53] = 0;
   out_4044238657628669408[54] = 0;
   out_4044238657628669408[55] = 0;
   out_4044238657628669408[56] = 0;
   out_4044238657628669408[57] = 0;
   out_4044238657628669408[58] = 0;
   out_4044238657628669408[59] = 0;
   out_4044238657628669408[60] = 1.0;
   out_4044238657628669408[61] = 0;
   out_4044238657628669408[62] = 0;
   out_4044238657628669408[63] = 0;
   out_4044238657628669408[64] = 0;
   out_4044238657628669408[65] = 0;
   out_4044238657628669408[66] = 0;
   out_4044238657628669408[67] = 0;
   out_4044238657628669408[68] = 0;
   out_4044238657628669408[69] = 0;
   out_4044238657628669408[70] = 1.0;
   out_4044238657628669408[71] = 0;
   out_4044238657628669408[72] = 0;
   out_4044238657628669408[73] = 0;
   out_4044238657628669408[74] = 0;
   out_4044238657628669408[75] = 0;
   out_4044238657628669408[76] = 0;
   out_4044238657628669408[77] = 0;
   out_4044238657628669408[78] = 0;
   out_4044238657628669408[79] = 0;
   out_4044238657628669408[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6587028471138044914) {
   out_6587028471138044914[0] = state[0];
   out_6587028471138044914[1] = state[1];
   out_6587028471138044914[2] = state[2];
   out_6587028471138044914[3] = state[3];
   out_6587028471138044914[4] = state[4];
   out_6587028471138044914[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6587028471138044914[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6587028471138044914[7] = state[7];
   out_6587028471138044914[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7531965082275181187) {
   out_7531965082275181187[0] = 1;
   out_7531965082275181187[1] = 0;
   out_7531965082275181187[2] = 0;
   out_7531965082275181187[3] = 0;
   out_7531965082275181187[4] = 0;
   out_7531965082275181187[5] = 0;
   out_7531965082275181187[6] = 0;
   out_7531965082275181187[7] = 0;
   out_7531965082275181187[8] = 0;
   out_7531965082275181187[9] = 0;
   out_7531965082275181187[10] = 1;
   out_7531965082275181187[11] = 0;
   out_7531965082275181187[12] = 0;
   out_7531965082275181187[13] = 0;
   out_7531965082275181187[14] = 0;
   out_7531965082275181187[15] = 0;
   out_7531965082275181187[16] = 0;
   out_7531965082275181187[17] = 0;
   out_7531965082275181187[18] = 0;
   out_7531965082275181187[19] = 0;
   out_7531965082275181187[20] = 1;
   out_7531965082275181187[21] = 0;
   out_7531965082275181187[22] = 0;
   out_7531965082275181187[23] = 0;
   out_7531965082275181187[24] = 0;
   out_7531965082275181187[25] = 0;
   out_7531965082275181187[26] = 0;
   out_7531965082275181187[27] = 0;
   out_7531965082275181187[28] = 0;
   out_7531965082275181187[29] = 0;
   out_7531965082275181187[30] = 1;
   out_7531965082275181187[31] = 0;
   out_7531965082275181187[32] = 0;
   out_7531965082275181187[33] = 0;
   out_7531965082275181187[34] = 0;
   out_7531965082275181187[35] = 0;
   out_7531965082275181187[36] = 0;
   out_7531965082275181187[37] = 0;
   out_7531965082275181187[38] = 0;
   out_7531965082275181187[39] = 0;
   out_7531965082275181187[40] = 1;
   out_7531965082275181187[41] = 0;
   out_7531965082275181187[42] = 0;
   out_7531965082275181187[43] = 0;
   out_7531965082275181187[44] = 0;
   out_7531965082275181187[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7531965082275181187[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7531965082275181187[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7531965082275181187[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7531965082275181187[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7531965082275181187[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7531965082275181187[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7531965082275181187[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7531965082275181187[53] = -9.8000000000000007*dt;
   out_7531965082275181187[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7531965082275181187[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7531965082275181187[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7531965082275181187[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7531965082275181187[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7531965082275181187[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7531965082275181187[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7531965082275181187[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7531965082275181187[62] = 0;
   out_7531965082275181187[63] = 0;
   out_7531965082275181187[64] = 0;
   out_7531965082275181187[65] = 0;
   out_7531965082275181187[66] = 0;
   out_7531965082275181187[67] = 0;
   out_7531965082275181187[68] = 0;
   out_7531965082275181187[69] = 0;
   out_7531965082275181187[70] = 1;
   out_7531965082275181187[71] = 0;
   out_7531965082275181187[72] = 0;
   out_7531965082275181187[73] = 0;
   out_7531965082275181187[74] = 0;
   out_7531965082275181187[75] = 0;
   out_7531965082275181187[76] = 0;
   out_7531965082275181187[77] = 0;
   out_7531965082275181187[78] = 0;
   out_7531965082275181187[79] = 0;
   out_7531965082275181187[80] = 1;
}
void h_25(double *state, double *unused, double *out_7956124124599872411) {
   out_7956124124599872411[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1152149855673758191) {
   out_1152149855673758191[0] = 0;
   out_1152149855673758191[1] = 0;
   out_1152149855673758191[2] = 0;
   out_1152149855673758191[3] = 0;
   out_1152149855673758191[4] = 0;
   out_1152149855673758191[5] = 0;
   out_1152149855673758191[6] = 1;
   out_1152149855673758191[7] = 0;
   out_1152149855673758191[8] = 0;
}
void h_24(double *state, double *unused, double *out_8831563141601233713) {
   out_8831563141601233713[0] = state[4];
   out_8831563141601233713[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1020499743331741375) {
   out_1020499743331741375[0] = 0;
   out_1020499743331741375[1] = 0;
   out_1020499743331741375[2] = 0;
   out_1020499743331741375[3] = 0;
   out_1020499743331741375[4] = 1;
   out_1020499743331741375[5] = 0;
   out_1020499743331741375[6] = 0;
   out_1020499743331741375[7] = 0;
   out_1020499743331741375[8] = 0;
   out_1020499743331741375[9] = 0;
   out_1020499743331741375[10] = 0;
   out_1020499743331741375[11] = 0;
   out_1020499743331741375[12] = 0;
   out_1020499743331741375[13] = 0;
   out_1020499743331741375[14] = 1;
   out_1020499743331741375[15] = 0;
   out_1020499743331741375[16] = 0;
   out_1020499743331741375[17] = 0;
}
void h_30(double *state, double *unused, double *out_6628936472681177553) {
   out_6628936472681177553[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3375546474453850007) {
   out_3375546474453850007[0] = 0;
   out_3375546474453850007[1] = 0;
   out_3375546474453850007[2] = 0;
   out_3375546474453850007[3] = 0;
   out_3375546474453850007[4] = 1;
   out_3375546474453850007[5] = 0;
   out_3375546474453850007[6] = 0;
   out_3375546474453850007[7] = 0;
   out_3375546474453850007[8] = 0;
}
void h_26(double *state, double *unused, double *out_3816384369411324170) {
   out_3816384369411324170[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2589353463200298033) {
   out_2589353463200298033[0] = 0;
   out_2589353463200298033[1] = 0;
   out_2589353463200298033[2] = 0;
   out_2589353463200298033[3] = 0;
   out_2589353463200298033[4] = 0;
   out_2589353463200298033[5] = 0;
   out_2589353463200298033[6] = 0;
   out_2589353463200298033[7] = 1;
   out_2589353463200298033[8] = 0;
}
void h_27(double *state, double *unused, double *out_7012614271698209976) {
   out_7012614271698209976[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5550309786254274918) {
   out_5550309786254274918[0] = 0;
   out_5550309786254274918[1] = 0;
   out_5550309786254274918[2] = 0;
   out_5550309786254274918[3] = 1;
   out_5550309786254274918[4] = 0;
   out_5550309786254274918[5] = 0;
   out_5550309786254274918[6] = 0;
   out_5550309786254274918[7] = 0;
   out_5550309786254274918[8] = 0;
}
void h_29(double *state, double *unused, double *out_1627263205020331908) {
   out_1627263205020331908[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2865315130139457823) {
   out_2865315130139457823[0] = 0;
   out_2865315130139457823[1] = 1;
   out_2865315130139457823[2] = 0;
   out_2865315130139457823[3] = 0;
   out_2865315130139457823[4] = 0;
   out_2865315130139457823[5] = 0;
   out_2865315130139457823[6] = 0;
   out_2865315130139457823[7] = 0;
   out_2865315130139457823[8] = 0;
}
void h_28(double *state, double *unused, double *out_7994698365033508088) {
   out_7994698365033508088[0] = state[0];
}
void H_28(double *state, double *unused, double *out_901684858574131572) {
   out_901684858574131572[0] = 1;
   out_901684858574131572[1] = 0;
   out_901684858574131572[2] = 0;
   out_901684858574131572[3] = 0;
   out_901684858574131572[4] = 0;
   out_901684858574131572[5] = 0;
   out_901684858574131572[6] = 0;
   out_901684858574131572[7] = 0;
   out_901684858574131572[8] = 0;
}
void h_31(double *state, double *unused, double *out_988808959918839917) {
   out_988808959918839917[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3215561565433649509) {
   out_3215561565433649509[0] = 0;
   out_3215561565433649509[1] = 0;
   out_3215561565433649509[2] = 0;
   out_3215561565433649509[3] = 0;
   out_3215561565433649509[4] = 0;
   out_3215561565433649509[5] = 0;
   out_3215561565433649509[6] = 0;
   out_3215561565433649509[7] = 0;
   out_3215561565433649509[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_9008082079438066370) {
  err_fun(nom_x, delta_x, out_9008082079438066370);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7705390806256602072) {
  inv_err_fun(nom_x, true_x, out_7705390806256602072);
}
void car_H_mod_fun(double *state, double *out_4044238657628669408) {
  H_mod_fun(state, out_4044238657628669408);
}
void car_f_fun(double *state, double dt, double *out_6587028471138044914) {
  f_fun(state,  dt, out_6587028471138044914);
}
void car_F_fun(double *state, double dt, double *out_7531965082275181187) {
  F_fun(state,  dt, out_7531965082275181187);
}
void car_h_25(double *state, double *unused, double *out_7956124124599872411) {
  h_25(state, unused, out_7956124124599872411);
}
void car_H_25(double *state, double *unused, double *out_1152149855673758191) {
  H_25(state, unused, out_1152149855673758191);
}
void car_h_24(double *state, double *unused, double *out_8831563141601233713) {
  h_24(state, unused, out_8831563141601233713);
}
void car_H_24(double *state, double *unused, double *out_1020499743331741375) {
  H_24(state, unused, out_1020499743331741375);
}
void car_h_30(double *state, double *unused, double *out_6628936472681177553) {
  h_30(state, unused, out_6628936472681177553);
}
void car_H_30(double *state, double *unused, double *out_3375546474453850007) {
  H_30(state, unused, out_3375546474453850007);
}
void car_h_26(double *state, double *unused, double *out_3816384369411324170) {
  h_26(state, unused, out_3816384369411324170);
}
void car_H_26(double *state, double *unused, double *out_2589353463200298033) {
  H_26(state, unused, out_2589353463200298033);
}
void car_h_27(double *state, double *unused, double *out_7012614271698209976) {
  h_27(state, unused, out_7012614271698209976);
}
void car_H_27(double *state, double *unused, double *out_5550309786254274918) {
  H_27(state, unused, out_5550309786254274918);
}
void car_h_29(double *state, double *unused, double *out_1627263205020331908) {
  h_29(state, unused, out_1627263205020331908);
}
void car_H_29(double *state, double *unused, double *out_2865315130139457823) {
  H_29(state, unused, out_2865315130139457823);
}
void car_h_28(double *state, double *unused, double *out_7994698365033508088) {
  h_28(state, unused, out_7994698365033508088);
}
void car_H_28(double *state, double *unused, double *out_901684858574131572) {
  H_28(state, unused, out_901684858574131572);
}
void car_h_31(double *state, double *unused, double *out_988808959918839917) {
  h_31(state, unused, out_988808959918839917);
}
void car_H_31(double *state, double *unused, double *out_3215561565433649509) {
  H_31(state, unused, out_3215561565433649509);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
