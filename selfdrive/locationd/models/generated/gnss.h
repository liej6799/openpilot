#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9093778295671839447);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_9126699124174503695);
void gnss_H_mod_fun(double *state, double *out_854507982997558257);
void gnss_f_fun(double *state, double dt, double *out_7770403001027613956);
void gnss_F_fun(double *state, double dt, double *out_7798555812212859819);
void gnss_h_6(double *state, double *sat_pos, double *out_9201175564716128809);
void gnss_H_6(double *state, double *sat_pos, double *out_1419005819748336970);
void gnss_h_20(double *state, double *sat_pos, double *out_8590157039648027835);
void gnss_H_20(double *state, double *sat_pos, double *out_5617129456073796795);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8069802722407087087);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7177713928341367106);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8069802722407087087);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7177713928341367106);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}