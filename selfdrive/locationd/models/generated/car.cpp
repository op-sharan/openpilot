#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6312187491562868488) {
   out_6312187491562868488[0] = delta_x[0] + nom_x[0];
   out_6312187491562868488[1] = delta_x[1] + nom_x[1];
   out_6312187491562868488[2] = delta_x[2] + nom_x[2];
   out_6312187491562868488[3] = delta_x[3] + nom_x[3];
   out_6312187491562868488[4] = delta_x[4] + nom_x[4];
   out_6312187491562868488[5] = delta_x[5] + nom_x[5];
   out_6312187491562868488[6] = delta_x[6] + nom_x[6];
   out_6312187491562868488[7] = delta_x[7] + nom_x[7];
   out_6312187491562868488[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8729778864089017210) {
   out_8729778864089017210[0] = -nom_x[0] + true_x[0];
   out_8729778864089017210[1] = -nom_x[1] + true_x[1];
   out_8729778864089017210[2] = -nom_x[2] + true_x[2];
   out_8729778864089017210[3] = -nom_x[3] + true_x[3];
   out_8729778864089017210[4] = -nom_x[4] + true_x[4];
   out_8729778864089017210[5] = -nom_x[5] + true_x[5];
   out_8729778864089017210[6] = -nom_x[6] + true_x[6];
   out_8729778864089017210[7] = -nom_x[7] + true_x[7];
   out_8729778864089017210[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2433884036008471400) {
   out_2433884036008471400[0] = 1.0;
   out_2433884036008471400[1] = 0.0;
   out_2433884036008471400[2] = 0.0;
   out_2433884036008471400[3] = 0.0;
   out_2433884036008471400[4] = 0.0;
   out_2433884036008471400[5] = 0.0;
   out_2433884036008471400[6] = 0.0;
   out_2433884036008471400[7] = 0.0;
   out_2433884036008471400[8] = 0.0;
   out_2433884036008471400[9] = 0.0;
   out_2433884036008471400[10] = 1.0;
   out_2433884036008471400[11] = 0.0;
   out_2433884036008471400[12] = 0.0;
   out_2433884036008471400[13] = 0.0;
   out_2433884036008471400[14] = 0.0;
   out_2433884036008471400[15] = 0.0;
   out_2433884036008471400[16] = 0.0;
   out_2433884036008471400[17] = 0.0;
   out_2433884036008471400[18] = 0.0;
   out_2433884036008471400[19] = 0.0;
   out_2433884036008471400[20] = 1.0;
   out_2433884036008471400[21] = 0.0;
   out_2433884036008471400[22] = 0.0;
   out_2433884036008471400[23] = 0.0;
   out_2433884036008471400[24] = 0.0;
   out_2433884036008471400[25] = 0.0;
   out_2433884036008471400[26] = 0.0;
   out_2433884036008471400[27] = 0.0;
   out_2433884036008471400[28] = 0.0;
   out_2433884036008471400[29] = 0.0;
   out_2433884036008471400[30] = 1.0;
   out_2433884036008471400[31] = 0.0;
   out_2433884036008471400[32] = 0.0;
   out_2433884036008471400[33] = 0.0;
   out_2433884036008471400[34] = 0.0;
   out_2433884036008471400[35] = 0.0;
   out_2433884036008471400[36] = 0.0;
   out_2433884036008471400[37] = 0.0;
   out_2433884036008471400[38] = 0.0;
   out_2433884036008471400[39] = 0.0;
   out_2433884036008471400[40] = 1.0;
   out_2433884036008471400[41] = 0.0;
   out_2433884036008471400[42] = 0.0;
   out_2433884036008471400[43] = 0.0;
   out_2433884036008471400[44] = 0.0;
   out_2433884036008471400[45] = 0.0;
   out_2433884036008471400[46] = 0.0;
   out_2433884036008471400[47] = 0.0;
   out_2433884036008471400[48] = 0.0;
   out_2433884036008471400[49] = 0.0;
   out_2433884036008471400[50] = 1.0;
   out_2433884036008471400[51] = 0.0;
   out_2433884036008471400[52] = 0.0;
   out_2433884036008471400[53] = 0.0;
   out_2433884036008471400[54] = 0.0;
   out_2433884036008471400[55] = 0.0;
   out_2433884036008471400[56] = 0.0;
   out_2433884036008471400[57] = 0.0;
   out_2433884036008471400[58] = 0.0;
   out_2433884036008471400[59] = 0.0;
   out_2433884036008471400[60] = 1.0;
   out_2433884036008471400[61] = 0.0;
   out_2433884036008471400[62] = 0.0;
   out_2433884036008471400[63] = 0.0;
   out_2433884036008471400[64] = 0.0;
   out_2433884036008471400[65] = 0.0;
   out_2433884036008471400[66] = 0.0;
   out_2433884036008471400[67] = 0.0;
   out_2433884036008471400[68] = 0.0;
   out_2433884036008471400[69] = 0.0;
   out_2433884036008471400[70] = 1.0;
   out_2433884036008471400[71] = 0.0;
   out_2433884036008471400[72] = 0.0;
   out_2433884036008471400[73] = 0.0;
   out_2433884036008471400[74] = 0.0;
   out_2433884036008471400[75] = 0.0;
   out_2433884036008471400[76] = 0.0;
   out_2433884036008471400[77] = 0.0;
   out_2433884036008471400[78] = 0.0;
   out_2433884036008471400[79] = 0.0;
   out_2433884036008471400[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_796409112878320158) {
   out_796409112878320158[0] = state[0];
   out_796409112878320158[1] = state[1];
   out_796409112878320158[2] = state[2];
   out_796409112878320158[3] = state[3];
   out_796409112878320158[4] = state[4];
   out_796409112878320158[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_796409112878320158[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_796409112878320158[7] = state[7];
   out_796409112878320158[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8757589862379398872) {
   out_8757589862379398872[0] = 1;
   out_8757589862379398872[1] = 0;
   out_8757589862379398872[2] = 0;
   out_8757589862379398872[3] = 0;
   out_8757589862379398872[4] = 0;
   out_8757589862379398872[5] = 0;
   out_8757589862379398872[6] = 0;
   out_8757589862379398872[7] = 0;
   out_8757589862379398872[8] = 0;
   out_8757589862379398872[9] = 0;
   out_8757589862379398872[10] = 1;
   out_8757589862379398872[11] = 0;
   out_8757589862379398872[12] = 0;
   out_8757589862379398872[13] = 0;
   out_8757589862379398872[14] = 0;
   out_8757589862379398872[15] = 0;
   out_8757589862379398872[16] = 0;
   out_8757589862379398872[17] = 0;
   out_8757589862379398872[18] = 0;
   out_8757589862379398872[19] = 0;
   out_8757589862379398872[20] = 1;
   out_8757589862379398872[21] = 0;
   out_8757589862379398872[22] = 0;
   out_8757589862379398872[23] = 0;
   out_8757589862379398872[24] = 0;
   out_8757589862379398872[25] = 0;
   out_8757589862379398872[26] = 0;
   out_8757589862379398872[27] = 0;
   out_8757589862379398872[28] = 0;
   out_8757589862379398872[29] = 0;
   out_8757589862379398872[30] = 1;
   out_8757589862379398872[31] = 0;
   out_8757589862379398872[32] = 0;
   out_8757589862379398872[33] = 0;
   out_8757589862379398872[34] = 0;
   out_8757589862379398872[35] = 0;
   out_8757589862379398872[36] = 0;
   out_8757589862379398872[37] = 0;
   out_8757589862379398872[38] = 0;
   out_8757589862379398872[39] = 0;
   out_8757589862379398872[40] = 1;
   out_8757589862379398872[41] = 0;
   out_8757589862379398872[42] = 0;
   out_8757589862379398872[43] = 0;
   out_8757589862379398872[44] = 0;
   out_8757589862379398872[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8757589862379398872[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8757589862379398872[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8757589862379398872[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8757589862379398872[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8757589862379398872[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8757589862379398872[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8757589862379398872[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8757589862379398872[53] = -9.8100000000000005*dt;
   out_8757589862379398872[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8757589862379398872[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8757589862379398872[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8757589862379398872[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8757589862379398872[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8757589862379398872[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8757589862379398872[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8757589862379398872[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8757589862379398872[62] = 0;
   out_8757589862379398872[63] = 0;
   out_8757589862379398872[64] = 0;
   out_8757589862379398872[65] = 0;
   out_8757589862379398872[66] = 0;
   out_8757589862379398872[67] = 0;
   out_8757589862379398872[68] = 0;
   out_8757589862379398872[69] = 0;
   out_8757589862379398872[70] = 1;
   out_8757589862379398872[71] = 0;
   out_8757589862379398872[72] = 0;
   out_8757589862379398872[73] = 0;
   out_8757589862379398872[74] = 0;
   out_8757589862379398872[75] = 0;
   out_8757589862379398872[76] = 0;
   out_8757589862379398872[77] = 0;
   out_8757589862379398872[78] = 0;
   out_8757589862379398872[79] = 0;
   out_8757589862379398872[80] = 1;
}
void h_25(double *state, double *unused, double *out_7412030721099193028) {
   out_7412030721099193028[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4581042973285851812) {
   out_4581042973285851812[0] = 0;
   out_4581042973285851812[1] = 0;
   out_4581042973285851812[2] = 0;
   out_4581042973285851812[3] = 0;
   out_4581042973285851812[4] = 0;
   out_4581042973285851812[5] = 0;
   out_4581042973285851812[6] = 1;
   out_4581042973285851812[7] = 0;
   out_4581042973285851812[8] = 0;
}
void h_24(double *state, double *unused, double *out_36611885517981660) {
   out_36611885517981660[0] = state[4];
   out_36611885517981660[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4647022212783343413) {
   out_4647022212783343413[0] = 0;
   out_4647022212783343413[1] = 0;
   out_4647022212783343413[2] = 0;
   out_4647022212783343413[3] = 0;
   out_4647022212783343413[4] = 1;
   out_4647022212783343413[5] = 0;
   out_4647022212783343413[6] = 0;
   out_4647022212783343413[7] = 0;
   out_4647022212783343413[8] = 0;
   out_4647022212783343413[9] = 0;
   out_4647022212783343413[10] = 0;
   out_4647022212783343413[11] = 0;
   out_4647022212783343413[12] = 0;
   out_4647022212783343413[13] = 0;
   out_4647022212783343413[14] = 1;
   out_4647022212783343413[15] = 0;
   out_4647022212783343413[16] = 0;
   out_4647022212783343413[17] = 0;
}
void h_30(double *state, double *unused, double *out_7173029876181856936) {
   out_7173029876181856936[0] = state[4];
}
void H_30(double *state, double *unused, double *out_9108739303413460010) {
   out_9108739303413460010[0] = 0;
   out_9108739303413460010[1] = 0;
   out_9108739303413460010[2] = 0;
   out_9108739303413460010[3] = 0;
   out_9108739303413460010[4] = 1;
   out_9108739303413460010[5] = 0;
   out_9108739303413460010[6] = 0;
   out_9108739303413460010[7] = 0;
   out_9108739303413460010[8] = 0;
}
void h_26(double *state, double *unused, double *out_5482582041446175976) {
   out_5482582041446175976[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8322546292159908036) {
   out_8322546292159908036[0] = 0;
   out_8322546292159908036[1] = 0;
   out_8322546292159908036[2] = 0;
   out_8322546292159908036[3] = 0;
   out_8322546292159908036[4] = 0;
   out_8322546292159908036[5] = 0;
   out_8322546292159908036[6] = 0;
   out_8322546292159908036[7] = 1;
   out_8322546292159908036[8] = 0;
}
void h_27(double *state, double *unused, double *out_1569938450907647737) {
   out_1569938450907647737[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6885145232229516793) {
   out_6885145232229516793[0] = 0;
   out_6885145232229516793[1] = 0;
   out_6885145232229516793[2] = 0;
   out_6885145232229516793[3] = 1;
   out_6885145232229516793[4] = 0;
   out_6885145232229516793[5] = 0;
   out_6885145232229516793[6] = 0;
   out_6885145232229516793[7] = 0;
   out_6885145232229516793[8] = 0;
}
void h_29(double *state, double *unused, double *out_5017889475568647426) {
   out_5017889475568647426[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8598507959099067826) {
   out_8598507959099067826[0] = 0;
   out_8598507959099067826[1] = 1;
   out_8598507959099067826[2] = 0;
   out_8598507959099067826[3] = 0;
   out_8598507959099067826[4] = 0;
   out_8598507959099067826[5] = 0;
   out_8598507959099067826[6] = 0;
   out_8598507959099067826[7] = 0;
   out_8598507959099067826[8] = 0;
}
void h_28(double *state, double *unused, double *out_3431734816753197908) {
   out_3431734816753197908[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4765837097540953216) {
   out_4765837097540953216[0] = 1;
   out_4765837097540953216[1] = 0;
   out_4765837097540953216[2] = 0;
   out_4765837097540953216[3] = 0;
   out_4765837097540953216[4] = 0;
   out_4765837097540953216[5] = 0;
   out_4765837097540953216[6] = 0;
   out_4765837097540953216[7] = 0;
   out_4765837097540953216[8] = 0;
}
void h_31(double *state, double *unused, double *out_8237682826887035884) {
   out_8237682826887035884[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4550397011408891384) {
   out_4550397011408891384[0] = 0;
   out_4550397011408891384[1] = 0;
   out_4550397011408891384[2] = 0;
   out_4550397011408891384[3] = 0;
   out_4550397011408891384[4] = 0;
   out_4550397011408891384[5] = 0;
   out_4550397011408891384[6] = 0;
   out_4550397011408891384[7] = 0;
   out_4550397011408891384[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6312187491562868488) {
  err_fun(nom_x, delta_x, out_6312187491562868488);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8729778864089017210) {
  inv_err_fun(nom_x, true_x, out_8729778864089017210);
}
void car_H_mod_fun(double *state, double *out_2433884036008471400) {
  H_mod_fun(state, out_2433884036008471400);
}
void car_f_fun(double *state, double dt, double *out_796409112878320158) {
  f_fun(state,  dt, out_796409112878320158);
}
void car_F_fun(double *state, double dt, double *out_8757589862379398872) {
  F_fun(state,  dt, out_8757589862379398872);
}
void car_h_25(double *state, double *unused, double *out_7412030721099193028) {
  h_25(state, unused, out_7412030721099193028);
}
void car_H_25(double *state, double *unused, double *out_4581042973285851812) {
  H_25(state, unused, out_4581042973285851812);
}
void car_h_24(double *state, double *unused, double *out_36611885517981660) {
  h_24(state, unused, out_36611885517981660);
}
void car_H_24(double *state, double *unused, double *out_4647022212783343413) {
  H_24(state, unused, out_4647022212783343413);
}
void car_h_30(double *state, double *unused, double *out_7173029876181856936) {
  h_30(state, unused, out_7173029876181856936);
}
void car_H_30(double *state, double *unused, double *out_9108739303413460010) {
  H_30(state, unused, out_9108739303413460010);
}
void car_h_26(double *state, double *unused, double *out_5482582041446175976) {
  h_26(state, unused, out_5482582041446175976);
}
void car_H_26(double *state, double *unused, double *out_8322546292159908036) {
  H_26(state, unused, out_8322546292159908036);
}
void car_h_27(double *state, double *unused, double *out_1569938450907647737) {
  h_27(state, unused, out_1569938450907647737);
}
void car_H_27(double *state, double *unused, double *out_6885145232229516793) {
  H_27(state, unused, out_6885145232229516793);
}
void car_h_29(double *state, double *unused, double *out_5017889475568647426) {
  h_29(state, unused, out_5017889475568647426);
}
void car_H_29(double *state, double *unused, double *out_8598507959099067826) {
  H_29(state, unused, out_8598507959099067826);
}
void car_h_28(double *state, double *unused, double *out_3431734816753197908) {
  h_28(state, unused, out_3431734816753197908);
}
void car_H_28(double *state, double *unused, double *out_4765837097540953216) {
  H_28(state, unused, out_4765837097540953216);
}
void car_h_31(double *state, double *unused, double *out_8237682826887035884) {
  h_31(state, unused, out_8237682826887035884);
}
void car_H_31(double *state, double *unused, double *out_4550397011408891384) {
  H_31(state, unused, out_4550397011408891384);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
