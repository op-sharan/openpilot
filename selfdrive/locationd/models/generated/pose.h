#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1334608923131309184);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5362236298631416382);
void pose_H_mod_fun(double *state, double *out_4765044121338691268);
void pose_f_fun(double *state, double dt, double *out_2590892121983961236);
void pose_F_fun(double *state, double dt, double *out_6449160121632499508);
void pose_h_4(double *state, double *unused, double *out_7784886013591512861);
void pose_H_4(double *state, double *unused, double *out_3567096639455793164);
void pose_h_10(double *state, double *unused, double *out_7334008623224553615);
void pose_H_10(double *state, double *unused, double *out_157882816050330764);
void pose_h_13(double *state, double *unused, double *out_7104614274408665918);
void pose_H_13(double *state, double *unused, double *out_354822814123460363);
void pose_h_14(double *state, double *unused, double *out_7017778734957237132);
void pose_H_14(double *state, double *unused, double *out_6649885071751165460);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}