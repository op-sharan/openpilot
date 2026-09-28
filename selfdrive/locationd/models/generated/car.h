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
void car_err_fun(double *nom_x, double *delta_x, double *out_7628497221553728992);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4491489622993553759);
void car_H_mod_fun(double *state, double *out_4025689497106729796);
void car_f_fun(double *state, double dt, double *out_1307275291988832241);
void car_F_fun(double *state, double dt, double *out_6482213034681649735);
void car_h_25(double *state, double *unused, double *out_1838035438271359109);
void car_H_25(double *state, double *unused, double *out_5167498728805507441);
void car_h_24(double *state, double *unused, double *out_4340443045834773276);
void car_H_24(double *state, double *unused, double *out_7340148327811007007);
void car_h_30(double *state, double *unused, double *out_3476572610417626098);
void car_H_30(double *state, double *unused, double *out_1749191612686109314);
void car_h_26(double *state, double *unused, double *out_3922242655353979235);
void car_H_26(double *state, double *unused, double *out_1862972759044706840);
void car_h_27(double *state, double *unused, double *out_7596020352265097065);
void car_H_27(double *state, double *unused, double *out_425571699114315597);
void car_h_29(double *state, double *unused, double *out_4599627427431185905);
void car_H_29(double *state, double *unused, double *out_2259422957000501498);
void car_h_28(double *state, double *unused, double *out_8763764153341238802);
void car_H_28(double *state, double *unused, double *out_7221333443053397204);
void car_h_31(double *state, double *unused, double *out_7299681846412492073);
void car_H_31(double *state, double *unused, double *out_1909176521706309812);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}