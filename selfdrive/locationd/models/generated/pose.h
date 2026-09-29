#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5549195789352376606);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6348441241420563996);
void pose_H_mod_fun(double *state, double *out_932088622648113863);
void pose_f_fun(double *state, double dt, double *out_65737351960750703);
void pose_F_fun(double *state, double dt, double *out_3304224532205862059);
void pose_h_4(double *state, double *unused, double *out_5378292505758393229);
void pose_H_4(double *state, double *unused, double *out_2130036104531011967);
void pose_h_10(double *state, double *unused, double *out_2124950356332181460);
void pose_H_10(double *state, double *unused, double *out_8589748122456887360);
void pose_h_13(double *state, double *unused, double *out_7381262973170721868);
void pose_H_13(double *state, double *unused, double *out_5342309929863344768);
void pose_h_14(double *state, double *unused, double *out_2697166449345768413);
void pose_H_14(double *state, double *unused, double *out_952752327764360329);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}