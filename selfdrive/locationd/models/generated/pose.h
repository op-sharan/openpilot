#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2114100039121924448);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2618631132718776507);
void pose_H_mod_fun(double *state, double *out_4058554015661107700);
void pose_f_fun(double *state, double dt, double *out_5119116025028968719);
void pose_F_fun(double *state, double dt, double *out_6155576186277025652);
void pose_h_4(double *state, double *unused, double *out_8035983586487788014);
void pose_H_4(double *state, double *unused, double *out_9211072400700188435);
void pose_h_10(double *state, double *unused, double *out_2517419169749679290);
void pose_H_10(double *state, double *unused, double *out_3554499959659757522);
void pose_h_13(double *state, double *unused, double *out_231752859429249092);
void pose_H_13(double *state, double *unused, double *out_6023397847677030380);
void pose_h_14(double *state, double *unused, double *out_4883626546262470581);
void pose_H_14(double *state, double *unused, double *out_6128283968404816139);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}