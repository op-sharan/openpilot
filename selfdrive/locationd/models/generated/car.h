#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8059887522619628636);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3995293944301919807);
void car_H_mod_fun(double *state, double *out_1466751874933723746);
void car_f_fun(double *state, double dt, double *out_1681767971590425073);
void car_F_fun(double *state, double dt, double *out_7509233405601621058);
void car_h_25(double *state, double *unused, double *out_1246716073205151034);
void car_H_25(double *state, double *unused, double *out_7786803972863590633);
void car_h_24(double *state, double *unused, double *out_1348262878792507760);
void car_H_24(double *state, double *unused, double *out_3040294645897081599);
void car_h_30(double *state, double *unused, double *out_600163749077985182);
void car_H_30(double *state, double *unused, double *out_3743249759354344228);
void car_h_26(double *state, double *unused, double *out_5829263656854511712);
void car_H_26(double *state, double *unused, double *out_4045300653989534409);
void car_h_27(double *state, double *unused, double *out_3754957194455694611);
void car_H_27(double *state, double *unused, double *out_5918013071154769139);
void car_h_29(double *state, double *unused, double *out_3597770526104565466);
void car_H_29(double *state, double *unused, double *out_7631375798024320172);
void car_h_28(double *state, double *unused, double *out_718171143011361926);
void car_H_28(double *state, double *unused, double *out_5732969258615700870);
void car_h_31(double *state, double *unused, double *out_1521910135489656923);
void car_H_31(double *state, double *unused, double *out_7817449934740551061);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}