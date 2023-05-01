#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7310749464307319056);
void live_err_fun(double *nom_x, double *delta_x, double *out_8236526131657428764);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4747477926419252764);
void live_H_mod_fun(double *state, double *out_341303559384211768);
void live_f_fun(double *state, double dt, double *out_4549525036970607503);
void live_F_fun(double *state, double dt, double *out_5133759507898990925);
void live_h_4(double *state, double *unused, double *out_6872450210111746980);
void live_H_4(double *state, double *unused, double *out_2937396917593955740);
void live_h_9(double *state, double *unused, double *out_8735722694958550472);
void live_H_9(double *state, double *unused, double *out_3178586564223546385);
void live_h_10(double *state, double *unused, double *out_4239705948005809237);
void live_H_10(double *state, double *unused, double *out_4956943889339789869);
void live_h_12(double *state, double *unused, double *out_3848117259651593996);
void live_H_12(double *state, double *unused, double *out_3558495942641549407);
void live_h_35(double *state, double *unused, double *out_854595621797169640);
void live_H_35(double *state, double *unused, double *out_6304058974966563116);
void live_h_32(double *state, double *unused, double *out_2979708909891080243);
void live_H_32(double *state, double *unused, double *out_9087717139384980709);
void live_h_13(double *state, double *unused, double *out_7896661243212334910);
void live_H_13(double *state, double *unused, double *out_83856576196215707);
void live_h_14(double *state, double *unused, double *out_8735722694958550472);
void live_H_14(double *state, double *unused, double *out_3178586564223546385);
void live_h_33(double *state, double *unused, double *out_9184153656412830582);
void live_H_33(double *state, double *unused, double *out_8992128094104130896);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}