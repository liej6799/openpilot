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
void live_H(double *in_vec, double *out_768290455835651354);
void live_err_fun(double *nom_x, double *delta_x, double *out_5955955075753187712);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5388686253131612340);
void live_H_mod_fun(double *state, double *out_2552112100007039261);
void live_f_fun(double *state, double dt, double *out_1532088538845507207);
void live_F_fun(double *state, double dt, double *out_8958354368422034212);
void live_h_4(double *state, double *unused, double *out_1213802693008545631);
void live_H_4(double *state, double *unused, double *out_4709009694507262627);
void live_h_9(double *state, double *unused, double *out_246909689773124009);
void live_H_9(double *state, double *unused, double *out_551841958152485144);
void live_h_10(double *state, double *unused, double *out_7769468188847797302);
void live_H_10(double *state, double *unused, double *out_2350300367353127752);
void live_h_12(double *state, double *unused, double *out_3244101654778954460);
void live_H_12(double *state, double *unused, double *out_5330108719554856294);
void live_h_35(double *state, double *unused, double *out_3662689924684647337);
void live_H_35(double *state, double *unused, double *out_8075671751879870003);
void live_h_32(double *state, double *unused, double *out_2198963332362222475);
void live_H_32(double *state, double *unused, double *out_8155445796665977458);
void live_h_13(double *state, double *unused, double *out_3486139371308475582);
void live_H_13(double *state, double *unused, double *out_1290085859790998985);
void live_h_14(double *state, double *unused, double *out_246909689773124009);
void live_H_14(double *state, double *unused, double *out_551841958152485144);
void live_h_33(double *state, double *unused, double *out_2379161178703843220);
void live_H_33(double *state, double *unused, double *out_7220515317190824009);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}