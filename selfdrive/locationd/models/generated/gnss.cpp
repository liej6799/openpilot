#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9093778295671839447) {
   out_9093778295671839447[0] = delta_x[0] + nom_x[0];
   out_9093778295671839447[1] = delta_x[1] + nom_x[1];
   out_9093778295671839447[2] = delta_x[2] + nom_x[2];
   out_9093778295671839447[3] = delta_x[3] + nom_x[3];
   out_9093778295671839447[4] = delta_x[4] + nom_x[4];
   out_9093778295671839447[5] = delta_x[5] + nom_x[5];
   out_9093778295671839447[6] = delta_x[6] + nom_x[6];
   out_9093778295671839447[7] = delta_x[7] + nom_x[7];
   out_9093778295671839447[8] = delta_x[8] + nom_x[8];
   out_9093778295671839447[9] = delta_x[9] + nom_x[9];
   out_9093778295671839447[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9126699124174503695) {
   out_9126699124174503695[0] = -nom_x[0] + true_x[0];
   out_9126699124174503695[1] = -nom_x[1] + true_x[1];
   out_9126699124174503695[2] = -nom_x[2] + true_x[2];
   out_9126699124174503695[3] = -nom_x[3] + true_x[3];
   out_9126699124174503695[4] = -nom_x[4] + true_x[4];
   out_9126699124174503695[5] = -nom_x[5] + true_x[5];
   out_9126699124174503695[6] = -nom_x[6] + true_x[6];
   out_9126699124174503695[7] = -nom_x[7] + true_x[7];
   out_9126699124174503695[8] = -nom_x[8] + true_x[8];
   out_9126699124174503695[9] = -nom_x[9] + true_x[9];
   out_9126699124174503695[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_854507982997558257) {
   out_854507982997558257[0] = 1.0;
   out_854507982997558257[1] = 0;
   out_854507982997558257[2] = 0;
   out_854507982997558257[3] = 0;
   out_854507982997558257[4] = 0;
   out_854507982997558257[5] = 0;
   out_854507982997558257[6] = 0;
   out_854507982997558257[7] = 0;
   out_854507982997558257[8] = 0;
   out_854507982997558257[9] = 0;
   out_854507982997558257[10] = 0;
   out_854507982997558257[11] = 0;
   out_854507982997558257[12] = 1.0;
   out_854507982997558257[13] = 0;
   out_854507982997558257[14] = 0;
   out_854507982997558257[15] = 0;
   out_854507982997558257[16] = 0;
   out_854507982997558257[17] = 0;
   out_854507982997558257[18] = 0;
   out_854507982997558257[19] = 0;
   out_854507982997558257[20] = 0;
   out_854507982997558257[21] = 0;
   out_854507982997558257[22] = 0;
   out_854507982997558257[23] = 0;
   out_854507982997558257[24] = 1.0;
   out_854507982997558257[25] = 0;
   out_854507982997558257[26] = 0;
   out_854507982997558257[27] = 0;
   out_854507982997558257[28] = 0;
   out_854507982997558257[29] = 0;
   out_854507982997558257[30] = 0;
   out_854507982997558257[31] = 0;
   out_854507982997558257[32] = 0;
   out_854507982997558257[33] = 0;
   out_854507982997558257[34] = 0;
   out_854507982997558257[35] = 0;
   out_854507982997558257[36] = 1.0;
   out_854507982997558257[37] = 0;
   out_854507982997558257[38] = 0;
   out_854507982997558257[39] = 0;
   out_854507982997558257[40] = 0;
   out_854507982997558257[41] = 0;
   out_854507982997558257[42] = 0;
   out_854507982997558257[43] = 0;
   out_854507982997558257[44] = 0;
   out_854507982997558257[45] = 0;
   out_854507982997558257[46] = 0;
   out_854507982997558257[47] = 0;
   out_854507982997558257[48] = 1.0;
   out_854507982997558257[49] = 0;
   out_854507982997558257[50] = 0;
   out_854507982997558257[51] = 0;
   out_854507982997558257[52] = 0;
   out_854507982997558257[53] = 0;
   out_854507982997558257[54] = 0;
   out_854507982997558257[55] = 0;
   out_854507982997558257[56] = 0;
   out_854507982997558257[57] = 0;
   out_854507982997558257[58] = 0;
   out_854507982997558257[59] = 0;
   out_854507982997558257[60] = 1.0;
   out_854507982997558257[61] = 0;
   out_854507982997558257[62] = 0;
   out_854507982997558257[63] = 0;
   out_854507982997558257[64] = 0;
   out_854507982997558257[65] = 0;
   out_854507982997558257[66] = 0;
   out_854507982997558257[67] = 0;
   out_854507982997558257[68] = 0;
   out_854507982997558257[69] = 0;
   out_854507982997558257[70] = 0;
   out_854507982997558257[71] = 0;
   out_854507982997558257[72] = 1.0;
   out_854507982997558257[73] = 0;
   out_854507982997558257[74] = 0;
   out_854507982997558257[75] = 0;
   out_854507982997558257[76] = 0;
   out_854507982997558257[77] = 0;
   out_854507982997558257[78] = 0;
   out_854507982997558257[79] = 0;
   out_854507982997558257[80] = 0;
   out_854507982997558257[81] = 0;
   out_854507982997558257[82] = 0;
   out_854507982997558257[83] = 0;
   out_854507982997558257[84] = 1.0;
   out_854507982997558257[85] = 0;
   out_854507982997558257[86] = 0;
   out_854507982997558257[87] = 0;
   out_854507982997558257[88] = 0;
   out_854507982997558257[89] = 0;
   out_854507982997558257[90] = 0;
   out_854507982997558257[91] = 0;
   out_854507982997558257[92] = 0;
   out_854507982997558257[93] = 0;
   out_854507982997558257[94] = 0;
   out_854507982997558257[95] = 0;
   out_854507982997558257[96] = 1.0;
   out_854507982997558257[97] = 0;
   out_854507982997558257[98] = 0;
   out_854507982997558257[99] = 0;
   out_854507982997558257[100] = 0;
   out_854507982997558257[101] = 0;
   out_854507982997558257[102] = 0;
   out_854507982997558257[103] = 0;
   out_854507982997558257[104] = 0;
   out_854507982997558257[105] = 0;
   out_854507982997558257[106] = 0;
   out_854507982997558257[107] = 0;
   out_854507982997558257[108] = 1.0;
   out_854507982997558257[109] = 0;
   out_854507982997558257[110] = 0;
   out_854507982997558257[111] = 0;
   out_854507982997558257[112] = 0;
   out_854507982997558257[113] = 0;
   out_854507982997558257[114] = 0;
   out_854507982997558257[115] = 0;
   out_854507982997558257[116] = 0;
   out_854507982997558257[117] = 0;
   out_854507982997558257[118] = 0;
   out_854507982997558257[119] = 0;
   out_854507982997558257[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_7770403001027613956) {
   out_7770403001027613956[0] = dt*state[3] + state[0];
   out_7770403001027613956[1] = dt*state[4] + state[1];
   out_7770403001027613956[2] = dt*state[5] + state[2];
   out_7770403001027613956[3] = state[3];
   out_7770403001027613956[4] = state[4];
   out_7770403001027613956[5] = state[5];
   out_7770403001027613956[6] = dt*state[7] + state[6];
   out_7770403001027613956[7] = dt*state[8] + state[7];
   out_7770403001027613956[8] = state[8];
   out_7770403001027613956[9] = state[9];
   out_7770403001027613956[10] = state[10];
}
void F_fun(double *state, double dt, double *out_7798555812212859819) {
   out_7798555812212859819[0] = 1;
   out_7798555812212859819[1] = 0;
   out_7798555812212859819[2] = 0;
   out_7798555812212859819[3] = dt;
   out_7798555812212859819[4] = 0;
   out_7798555812212859819[5] = 0;
   out_7798555812212859819[6] = 0;
   out_7798555812212859819[7] = 0;
   out_7798555812212859819[8] = 0;
   out_7798555812212859819[9] = 0;
   out_7798555812212859819[10] = 0;
   out_7798555812212859819[11] = 0;
   out_7798555812212859819[12] = 1;
   out_7798555812212859819[13] = 0;
   out_7798555812212859819[14] = 0;
   out_7798555812212859819[15] = dt;
   out_7798555812212859819[16] = 0;
   out_7798555812212859819[17] = 0;
   out_7798555812212859819[18] = 0;
   out_7798555812212859819[19] = 0;
   out_7798555812212859819[20] = 0;
   out_7798555812212859819[21] = 0;
   out_7798555812212859819[22] = 0;
   out_7798555812212859819[23] = 0;
   out_7798555812212859819[24] = 1;
   out_7798555812212859819[25] = 0;
   out_7798555812212859819[26] = 0;
   out_7798555812212859819[27] = dt;
   out_7798555812212859819[28] = 0;
   out_7798555812212859819[29] = 0;
   out_7798555812212859819[30] = 0;
   out_7798555812212859819[31] = 0;
   out_7798555812212859819[32] = 0;
   out_7798555812212859819[33] = 0;
   out_7798555812212859819[34] = 0;
   out_7798555812212859819[35] = 0;
   out_7798555812212859819[36] = 1;
   out_7798555812212859819[37] = 0;
   out_7798555812212859819[38] = 0;
   out_7798555812212859819[39] = 0;
   out_7798555812212859819[40] = 0;
   out_7798555812212859819[41] = 0;
   out_7798555812212859819[42] = 0;
   out_7798555812212859819[43] = 0;
   out_7798555812212859819[44] = 0;
   out_7798555812212859819[45] = 0;
   out_7798555812212859819[46] = 0;
   out_7798555812212859819[47] = 0;
   out_7798555812212859819[48] = 1;
   out_7798555812212859819[49] = 0;
   out_7798555812212859819[50] = 0;
   out_7798555812212859819[51] = 0;
   out_7798555812212859819[52] = 0;
   out_7798555812212859819[53] = 0;
   out_7798555812212859819[54] = 0;
   out_7798555812212859819[55] = 0;
   out_7798555812212859819[56] = 0;
   out_7798555812212859819[57] = 0;
   out_7798555812212859819[58] = 0;
   out_7798555812212859819[59] = 0;
   out_7798555812212859819[60] = 1;
   out_7798555812212859819[61] = 0;
   out_7798555812212859819[62] = 0;
   out_7798555812212859819[63] = 0;
   out_7798555812212859819[64] = 0;
   out_7798555812212859819[65] = 0;
   out_7798555812212859819[66] = 0;
   out_7798555812212859819[67] = 0;
   out_7798555812212859819[68] = 0;
   out_7798555812212859819[69] = 0;
   out_7798555812212859819[70] = 0;
   out_7798555812212859819[71] = 0;
   out_7798555812212859819[72] = 1;
   out_7798555812212859819[73] = dt;
   out_7798555812212859819[74] = 0;
   out_7798555812212859819[75] = 0;
   out_7798555812212859819[76] = 0;
   out_7798555812212859819[77] = 0;
   out_7798555812212859819[78] = 0;
   out_7798555812212859819[79] = 0;
   out_7798555812212859819[80] = 0;
   out_7798555812212859819[81] = 0;
   out_7798555812212859819[82] = 0;
   out_7798555812212859819[83] = 0;
   out_7798555812212859819[84] = 1;
   out_7798555812212859819[85] = dt;
   out_7798555812212859819[86] = 0;
   out_7798555812212859819[87] = 0;
   out_7798555812212859819[88] = 0;
   out_7798555812212859819[89] = 0;
   out_7798555812212859819[90] = 0;
   out_7798555812212859819[91] = 0;
   out_7798555812212859819[92] = 0;
   out_7798555812212859819[93] = 0;
   out_7798555812212859819[94] = 0;
   out_7798555812212859819[95] = 0;
   out_7798555812212859819[96] = 1;
   out_7798555812212859819[97] = 0;
   out_7798555812212859819[98] = 0;
   out_7798555812212859819[99] = 0;
   out_7798555812212859819[100] = 0;
   out_7798555812212859819[101] = 0;
   out_7798555812212859819[102] = 0;
   out_7798555812212859819[103] = 0;
   out_7798555812212859819[104] = 0;
   out_7798555812212859819[105] = 0;
   out_7798555812212859819[106] = 0;
   out_7798555812212859819[107] = 0;
   out_7798555812212859819[108] = 1;
   out_7798555812212859819[109] = 0;
   out_7798555812212859819[110] = 0;
   out_7798555812212859819[111] = 0;
   out_7798555812212859819[112] = 0;
   out_7798555812212859819[113] = 0;
   out_7798555812212859819[114] = 0;
   out_7798555812212859819[115] = 0;
   out_7798555812212859819[116] = 0;
   out_7798555812212859819[117] = 0;
   out_7798555812212859819[118] = 0;
   out_7798555812212859819[119] = 0;
   out_7798555812212859819[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_9201175564716128809) {
   out_9201175564716128809[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1419005819748336970) {
   out_1419005819748336970[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1419005819748336970[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1419005819748336970[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1419005819748336970[3] = 0;
   out_1419005819748336970[4] = 0;
   out_1419005819748336970[5] = 0;
   out_1419005819748336970[6] = 1;
   out_1419005819748336970[7] = 0;
   out_1419005819748336970[8] = 0;
   out_1419005819748336970[9] = 0;
   out_1419005819748336970[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_8590157039648027835) {
   out_8590157039648027835[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5617129456073796795) {
   out_5617129456073796795[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5617129456073796795[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5617129456073796795[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5617129456073796795[3] = 0;
   out_5617129456073796795[4] = 0;
   out_5617129456073796795[5] = 0;
   out_5617129456073796795[6] = 1;
   out_5617129456073796795[7] = 0;
   out_5617129456073796795[8] = 0;
   out_5617129456073796795[9] = 1;
   out_5617129456073796795[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8069802722407087087) {
   out_8069802722407087087[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_7177713928341367106) {
   out_7177713928341367106[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[6] = 0;
   out_7177713928341367106[7] = 1;
   out_7177713928341367106[8] = 0;
   out_7177713928341367106[9] = 0;
   out_7177713928341367106[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8069802722407087087) {
   out_8069802722407087087[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_7177713928341367106) {
   out_7177713928341367106[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7177713928341367106[6] = 0;
   out_7177713928341367106[7] = 1;
   out_7177713928341367106[8] = 0;
   out_7177713928341367106[9] = 0;
   out_7177713928341367106[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9093778295671839447) {
  err_fun(nom_x, delta_x, out_9093778295671839447);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_9126699124174503695) {
  inv_err_fun(nom_x, true_x, out_9126699124174503695);
}
void gnss_H_mod_fun(double *state, double *out_854507982997558257) {
  H_mod_fun(state, out_854507982997558257);
}
void gnss_f_fun(double *state, double dt, double *out_7770403001027613956) {
  f_fun(state,  dt, out_7770403001027613956);
}
void gnss_F_fun(double *state, double dt, double *out_7798555812212859819) {
  F_fun(state,  dt, out_7798555812212859819);
}
void gnss_h_6(double *state, double *sat_pos, double *out_9201175564716128809) {
  h_6(state, sat_pos, out_9201175564716128809);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1419005819748336970) {
  H_6(state, sat_pos, out_1419005819748336970);
}
void gnss_h_20(double *state, double *sat_pos, double *out_8590157039648027835) {
  h_20(state, sat_pos, out_8590157039648027835);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5617129456073796795) {
  H_20(state, sat_pos, out_5617129456073796795);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8069802722407087087) {
  h_7(state, sat_pos_vel, out_8069802722407087087);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7177713928341367106) {
  H_7(state, sat_pos_vel, out_7177713928341367106);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8069802722407087087) {
  h_21(state, sat_pos_vel, out_8069802722407087087);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7177713928341367106) {
  H_21(state, sat_pos_vel, out_7177713928341367106);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
