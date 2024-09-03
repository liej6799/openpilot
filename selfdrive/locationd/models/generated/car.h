#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2472665262004959677);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8982529719675541846);
void car_H_mod_fun(double *state, double *out_1829537519410656848);
void car_f_fun(double *state, double dt, double *out_1757186677020913895);
void car_F_fun(double *state, double dt, double *out_5278060812158919790);
void car_h_25(double *state, double *unused, double *out_5407080294330651747);
void car_H_25(double *state, double *unused, double *out_3554883914703153887);
void car_h_24(double *state, double *unused, double *out_1039462419148683192);
void car_H_24(double *state, double *unused, double *out_3084426432659815163);
void car_h_30(double *state, double *unused, double *out_1363754932019699189);
void car_H_30(double *state, double *unused, double *out_3425544967559913817);
void car_h_26(double *state, double *unused, double *out_420245079118036754);
void car_H_26(double *state, double *unused, double *out_186619404170902337);
void car_h_27(double *state, double *unused, double *out_8037990511718054224);
void car_H_27(double *state, double *unused, double *out_1250781655759488906);
void car_h_29(double *state, double *unused, double *out_4400540983464031874);
void car_H_29(double *state, double *unused, double *out_462581071110062127);
void car_h_28(double *state, double *unused, double *out_758318952547942916);
void car_H_28(double *state, double *unused, double *out_1501049200455264124);
void car_h_31(double *state, double *unused, double *out_6857711686959534914);
void car_H_31(double *state, double *unused, double *out_3585529876580114315);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}