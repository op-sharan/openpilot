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
void car_err_fun(double *nom_x, double *delta_x, double *out_6312187491562868488);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8729778864089017210);
void car_H_mod_fun(double *state, double *out_2433884036008471400);
void car_f_fun(double *state, double dt, double *out_796409112878320158);
void car_F_fun(double *state, double dt, double *out_8757589862379398872);
void car_h_25(double *state, double *unused, double *out_7412030721099193028);
void car_H_25(double *state, double *unused, double *out_4581042973285851812);
void car_h_24(double *state, double *unused, double *out_36611885517981660);
void car_H_24(double *state, double *unused, double *out_4647022212783343413);
void car_h_30(double *state, double *unused, double *out_7173029876181856936);
void car_H_30(double *state, double *unused, double *out_9108739303413460010);
void car_h_26(double *state, double *unused, double *out_5482582041446175976);
void car_H_26(double *state, double *unused, double *out_8322546292159908036);
void car_h_27(double *state, double *unused, double *out_1569938450907647737);
void car_H_27(double *state, double *unused, double *out_6885145232229516793);
void car_h_29(double *state, double *unused, double *out_5017889475568647426);
void car_H_29(double *state, double *unused, double *out_8598507959099067826);
void car_h_28(double *state, double *unused, double *out_3431734816753197908);
void car_H_28(double *state, double *unused, double *out_4765837097540953216);
void car_h_31(double *state, double *unused, double *out_8237682826887035884);
void car_H_31(double *state, double *unused, double *out_4550397011408891384);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}