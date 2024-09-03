#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5782114737457059360);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8977268484679926314);
void gnss_H_mod_fun(double *state, double *out_338300992960698012);
void gnss_f_fun(double *state, double dt, double *out_1371145784839709762);
void gnss_F_fun(double *state, double dt, double *out_8435684940982077033);
void gnss_h_6(double *state, double *sat_pos, double *out_5387927519311920081);
void gnss_H_6(double *state, double *sat_pos, double *out_1097586483885744352);
void gnss_h_20(double *state, double *sat_pos, double *out_281837704672554358);
void gnss_H_20(double *state, double *sat_pos, double *out_1069919034497094674);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4066013704851088785);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_204057149496812951);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4066013704851088785);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_204057149496812951);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}