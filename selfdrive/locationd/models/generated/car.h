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
void car_err_fun(double *nom_x, double *delta_x, double *out_9008082079438066370);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7705390806256602072);
void car_H_mod_fun(double *state, double *out_4044238657628669408);
void car_f_fun(double *state, double dt, double *out_6587028471138044914);
void car_F_fun(double *state, double dt, double *out_7531965082275181187);
void car_h_25(double *state, double *unused, double *out_7956124124599872411);
void car_H_25(double *state, double *unused, double *out_1152149855673758191);
void car_h_24(double *state, double *unused, double *out_8831563141601233713);
void car_H_24(double *state, double *unused, double *out_1020499743331741375);
void car_h_30(double *state, double *unused, double *out_6628936472681177553);
void car_H_30(double *state, double *unused, double *out_3375546474453850007);
void car_h_26(double *state, double *unused, double *out_3816384369411324170);
void car_H_26(double *state, double *unused, double *out_2589353463200298033);
void car_h_27(double *state, double *unused, double *out_7012614271698209976);
void car_H_27(double *state, double *unused, double *out_5550309786254274918);
void car_h_29(double *state, double *unused, double *out_1627263205020331908);
void car_H_29(double *state, double *unused, double *out_2865315130139457823);
void car_h_28(double *state, double *unused, double *out_7994698365033508088);
void car_H_28(double *state, double *unused, double *out_901684858574131572);
void car_h_31(double *state, double *unused, double *out_988808959918839917);
void car_H_31(double *state, double *unused, double *out_3215561565433649509);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}