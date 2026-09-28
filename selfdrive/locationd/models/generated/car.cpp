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
void err_fun(double *nom_x, double *delta_x, double *out_7628497221553728992) {
   out_7628497221553728992[0] = delta_x[0] + nom_x[0];
   out_7628497221553728992[1] = delta_x[1] + nom_x[1];
   out_7628497221553728992[2] = delta_x[2] + nom_x[2];
   out_7628497221553728992[3] = delta_x[3] + nom_x[3];
   out_7628497221553728992[4] = delta_x[4] + nom_x[4];
   out_7628497221553728992[5] = delta_x[5] + nom_x[5];
   out_7628497221553728992[6] = delta_x[6] + nom_x[6];
   out_7628497221553728992[7] = delta_x[7] + nom_x[7];
   out_7628497221553728992[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4491489622993553759) {
   out_4491489622993553759[0] = -nom_x[0] + true_x[0];
   out_4491489622993553759[1] = -nom_x[1] + true_x[1];
   out_4491489622993553759[2] = -nom_x[2] + true_x[2];
   out_4491489622993553759[3] = -nom_x[3] + true_x[3];
   out_4491489622993553759[4] = -nom_x[4] + true_x[4];
   out_4491489622993553759[5] = -nom_x[5] + true_x[5];
   out_4491489622993553759[6] = -nom_x[6] + true_x[6];
   out_4491489622993553759[7] = -nom_x[7] + true_x[7];
   out_4491489622993553759[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4025689497106729796) {
   out_4025689497106729796[0] = 1.0;
   out_4025689497106729796[1] = 0.0;
   out_4025689497106729796[2] = 0.0;
   out_4025689497106729796[3] = 0.0;
   out_4025689497106729796[4] = 0.0;
   out_4025689497106729796[5] = 0.0;
   out_4025689497106729796[6] = 0.0;
   out_4025689497106729796[7] = 0.0;
   out_4025689497106729796[8] = 0.0;
   out_4025689497106729796[9] = 0.0;
   out_4025689497106729796[10] = 1.0;
   out_4025689497106729796[11] = 0.0;
   out_4025689497106729796[12] = 0.0;
   out_4025689497106729796[13] = 0.0;
   out_4025689497106729796[14] = 0.0;
   out_4025689497106729796[15] = 0.0;
   out_4025689497106729796[16] = 0.0;
   out_4025689497106729796[17] = 0.0;
   out_4025689497106729796[18] = 0.0;
   out_4025689497106729796[19] = 0.0;
   out_4025689497106729796[20] = 1.0;
   out_4025689497106729796[21] = 0.0;
   out_4025689497106729796[22] = 0.0;
   out_4025689497106729796[23] = 0.0;
   out_4025689497106729796[24] = 0.0;
   out_4025689497106729796[25] = 0.0;
   out_4025689497106729796[26] = 0.0;
   out_4025689497106729796[27] = 0.0;
   out_4025689497106729796[28] = 0.0;
   out_4025689497106729796[29] = 0.0;
   out_4025689497106729796[30] = 1.0;
   out_4025689497106729796[31] = 0.0;
   out_4025689497106729796[32] = 0.0;
   out_4025689497106729796[33] = 0.0;
   out_4025689497106729796[34] = 0.0;
   out_4025689497106729796[35] = 0.0;
   out_4025689497106729796[36] = 0.0;
   out_4025689497106729796[37] = 0.0;
   out_4025689497106729796[38] = 0.0;
   out_4025689497106729796[39] = 0.0;
   out_4025689497106729796[40] = 1.0;
   out_4025689497106729796[41] = 0.0;
   out_4025689497106729796[42] = 0.0;
   out_4025689497106729796[43] = 0.0;
   out_4025689497106729796[44] = 0.0;
   out_4025689497106729796[45] = 0.0;
   out_4025689497106729796[46] = 0.0;
   out_4025689497106729796[47] = 0.0;
   out_4025689497106729796[48] = 0.0;
   out_4025689497106729796[49] = 0.0;
   out_4025689497106729796[50] = 1.0;
   out_4025689497106729796[51] = 0.0;
   out_4025689497106729796[52] = 0.0;
   out_4025689497106729796[53] = 0.0;
   out_4025689497106729796[54] = 0.0;
   out_4025689497106729796[55] = 0.0;
   out_4025689497106729796[56] = 0.0;
   out_4025689497106729796[57] = 0.0;
   out_4025689497106729796[58] = 0.0;
   out_4025689497106729796[59] = 0.0;
   out_4025689497106729796[60] = 1.0;
   out_4025689497106729796[61] = 0.0;
   out_4025689497106729796[62] = 0.0;
   out_4025689497106729796[63] = 0.0;
   out_4025689497106729796[64] = 0.0;
   out_4025689497106729796[65] = 0.0;
   out_4025689497106729796[66] = 0.0;
   out_4025689497106729796[67] = 0.0;
   out_4025689497106729796[68] = 0.0;
   out_4025689497106729796[69] = 0.0;
   out_4025689497106729796[70] = 1.0;
   out_4025689497106729796[71] = 0.0;
   out_4025689497106729796[72] = 0.0;
   out_4025689497106729796[73] = 0.0;
   out_4025689497106729796[74] = 0.0;
   out_4025689497106729796[75] = 0.0;
   out_4025689497106729796[76] = 0.0;
   out_4025689497106729796[77] = 0.0;
   out_4025689497106729796[78] = 0.0;
   out_4025689497106729796[79] = 0.0;
   out_4025689497106729796[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1307275291988832241) {
   out_1307275291988832241[0] = state[0];
   out_1307275291988832241[1] = state[1];
   out_1307275291988832241[2] = state[2];
   out_1307275291988832241[3] = state[3];
   out_1307275291988832241[4] = state[4];
   out_1307275291988832241[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1307275291988832241[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1307275291988832241[7] = state[7];
   out_1307275291988832241[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6482213034681649735) {
   out_6482213034681649735[0] = 1;
   out_6482213034681649735[1] = 0;
   out_6482213034681649735[2] = 0;
   out_6482213034681649735[3] = 0;
   out_6482213034681649735[4] = 0;
   out_6482213034681649735[5] = 0;
   out_6482213034681649735[6] = 0;
   out_6482213034681649735[7] = 0;
   out_6482213034681649735[8] = 0;
   out_6482213034681649735[9] = 0;
   out_6482213034681649735[10] = 1;
   out_6482213034681649735[11] = 0;
   out_6482213034681649735[12] = 0;
   out_6482213034681649735[13] = 0;
   out_6482213034681649735[14] = 0;
   out_6482213034681649735[15] = 0;
   out_6482213034681649735[16] = 0;
   out_6482213034681649735[17] = 0;
   out_6482213034681649735[18] = 0;
   out_6482213034681649735[19] = 0;
   out_6482213034681649735[20] = 1;
   out_6482213034681649735[21] = 0;
   out_6482213034681649735[22] = 0;
   out_6482213034681649735[23] = 0;
   out_6482213034681649735[24] = 0;
   out_6482213034681649735[25] = 0;
   out_6482213034681649735[26] = 0;
   out_6482213034681649735[27] = 0;
   out_6482213034681649735[28] = 0;
   out_6482213034681649735[29] = 0;
   out_6482213034681649735[30] = 1;
   out_6482213034681649735[31] = 0;
   out_6482213034681649735[32] = 0;
   out_6482213034681649735[33] = 0;
   out_6482213034681649735[34] = 0;
   out_6482213034681649735[35] = 0;
   out_6482213034681649735[36] = 0;
   out_6482213034681649735[37] = 0;
   out_6482213034681649735[38] = 0;
   out_6482213034681649735[39] = 0;
   out_6482213034681649735[40] = 1;
   out_6482213034681649735[41] = 0;
   out_6482213034681649735[42] = 0;
   out_6482213034681649735[43] = 0;
   out_6482213034681649735[44] = 0;
   out_6482213034681649735[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6482213034681649735[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6482213034681649735[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6482213034681649735[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6482213034681649735[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6482213034681649735[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6482213034681649735[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6482213034681649735[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6482213034681649735[53] = -9.8100000000000005*dt;
   out_6482213034681649735[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6482213034681649735[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6482213034681649735[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6482213034681649735[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6482213034681649735[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6482213034681649735[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6482213034681649735[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6482213034681649735[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6482213034681649735[62] = 0;
   out_6482213034681649735[63] = 0;
   out_6482213034681649735[64] = 0;
   out_6482213034681649735[65] = 0;
   out_6482213034681649735[66] = 0;
   out_6482213034681649735[67] = 0;
   out_6482213034681649735[68] = 0;
   out_6482213034681649735[69] = 0;
   out_6482213034681649735[70] = 1;
   out_6482213034681649735[71] = 0;
   out_6482213034681649735[72] = 0;
   out_6482213034681649735[73] = 0;
   out_6482213034681649735[74] = 0;
   out_6482213034681649735[75] = 0;
   out_6482213034681649735[76] = 0;
   out_6482213034681649735[77] = 0;
   out_6482213034681649735[78] = 0;
   out_6482213034681649735[79] = 0;
   out_6482213034681649735[80] = 1;
}
void h_25(double *state, double *unused, double *out_1838035438271359109) {
   out_1838035438271359109[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5167498728805507441) {
   out_5167498728805507441[0] = 0;
   out_5167498728805507441[1] = 0;
   out_5167498728805507441[2] = 0;
   out_5167498728805507441[3] = 0;
   out_5167498728805507441[4] = 0;
   out_5167498728805507441[5] = 0;
   out_5167498728805507441[6] = 1;
   out_5167498728805507441[7] = 0;
   out_5167498728805507441[8] = 0;
}
void h_24(double *state, double *unused, double *out_4340443045834773276) {
   out_4340443045834773276[0] = state[4];
   out_4340443045834773276[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7340148327811007007) {
   out_7340148327811007007[0] = 0;
   out_7340148327811007007[1] = 0;
   out_7340148327811007007[2] = 0;
   out_7340148327811007007[3] = 0;
   out_7340148327811007007[4] = 1;
   out_7340148327811007007[5] = 0;
   out_7340148327811007007[6] = 0;
   out_7340148327811007007[7] = 0;
   out_7340148327811007007[8] = 0;
   out_7340148327811007007[9] = 0;
   out_7340148327811007007[10] = 0;
   out_7340148327811007007[11] = 0;
   out_7340148327811007007[12] = 0;
   out_7340148327811007007[13] = 0;
   out_7340148327811007007[14] = 1;
   out_7340148327811007007[15] = 0;
   out_7340148327811007007[16] = 0;
   out_7340148327811007007[17] = 0;
}
void h_30(double *state, double *unused, double *out_3476572610417626098) {
   out_3476572610417626098[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1749191612686109314) {
   out_1749191612686109314[0] = 0;
   out_1749191612686109314[1] = 0;
   out_1749191612686109314[2] = 0;
   out_1749191612686109314[3] = 0;
   out_1749191612686109314[4] = 1;
   out_1749191612686109314[5] = 0;
   out_1749191612686109314[6] = 0;
   out_1749191612686109314[7] = 0;
   out_1749191612686109314[8] = 0;
}
void h_26(double *state, double *unused, double *out_3922242655353979235) {
   out_3922242655353979235[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1862972759044706840) {
   out_1862972759044706840[0] = 0;
   out_1862972759044706840[1] = 0;
   out_1862972759044706840[2] = 0;
   out_1862972759044706840[3] = 0;
   out_1862972759044706840[4] = 0;
   out_1862972759044706840[5] = 0;
   out_1862972759044706840[6] = 0;
   out_1862972759044706840[7] = 1;
   out_1862972759044706840[8] = 0;
}
void h_27(double *state, double *unused, double *out_7596020352265097065) {
   out_7596020352265097065[0] = state[3];
}
void H_27(double *state, double *unused, double *out_425571699114315597) {
   out_425571699114315597[0] = 0;
   out_425571699114315597[1] = 0;
   out_425571699114315597[2] = 0;
   out_425571699114315597[3] = 1;
   out_425571699114315597[4] = 0;
   out_425571699114315597[5] = 0;
   out_425571699114315597[6] = 0;
   out_425571699114315597[7] = 0;
   out_425571699114315597[8] = 0;
}
void h_29(double *state, double *unused, double *out_4599627427431185905) {
   out_4599627427431185905[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2259422957000501498) {
   out_2259422957000501498[0] = 0;
   out_2259422957000501498[1] = 1;
   out_2259422957000501498[2] = 0;
   out_2259422957000501498[3] = 0;
   out_2259422957000501498[4] = 0;
   out_2259422957000501498[5] = 0;
   out_2259422957000501498[6] = 0;
   out_2259422957000501498[7] = 0;
   out_2259422957000501498[8] = 0;
}
void h_28(double *state, double *unused, double *out_8763764153341238802) {
   out_8763764153341238802[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7221333443053397204) {
   out_7221333443053397204[0] = 1;
   out_7221333443053397204[1] = 0;
   out_7221333443053397204[2] = 0;
   out_7221333443053397204[3] = 0;
   out_7221333443053397204[4] = 0;
   out_7221333443053397204[5] = 0;
   out_7221333443053397204[6] = 0;
   out_7221333443053397204[7] = 0;
   out_7221333443053397204[8] = 0;
}
void h_31(double *state, double *unused, double *out_7299681846412492073) {
   out_7299681846412492073[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1909176521706309812) {
   out_1909176521706309812[0] = 0;
   out_1909176521706309812[1] = 0;
   out_1909176521706309812[2] = 0;
   out_1909176521706309812[3] = 0;
   out_1909176521706309812[4] = 0;
   out_1909176521706309812[5] = 0;
   out_1909176521706309812[6] = 0;
   out_1909176521706309812[7] = 0;
   out_1909176521706309812[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7628497221553728992) {
  err_fun(nom_x, delta_x, out_7628497221553728992);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4491489622993553759) {
  inv_err_fun(nom_x, true_x, out_4491489622993553759);
}
void car_H_mod_fun(double *state, double *out_4025689497106729796) {
  H_mod_fun(state, out_4025689497106729796);
}
void car_f_fun(double *state, double dt, double *out_1307275291988832241) {
  f_fun(state,  dt, out_1307275291988832241);
}
void car_F_fun(double *state, double dt, double *out_6482213034681649735) {
  F_fun(state,  dt, out_6482213034681649735);
}
void car_h_25(double *state, double *unused, double *out_1838035438271359109) {
  h_25(state, unused, out_1838035438271359109);
}
void car_H_25(double *state, double *unused, double *out_5167498728805507441) {
  H_25(state, unused, out_5167498728805507441);
}
void car_h_24(double *state, double *unused, double *out_4340443045834773276) {
  h_24(state, unused, out_4340443045834773276);
}
void car_H_24(double *state, double *unused, double *out_7340148327811007007) {
  H_24(state, unused, out_7340148327811007007);
}
void car_h_30(double *state, double *unused, double *out_3476572610417626098) {
  h_30(state, unused, out_3476572610417626098);
}
void car_H_30(double *state, double *unused, double *out_1749191612686109314) {
  H_30(state, unused, out_1749191612686109314);
}
void car_h_26(double *state, double *unused, double *out_3922242655353979235) {
  h_26(state, unused, out_3922242655353979235);
}
void car_H_26(double *state, double *unused, double *out_1862972759044706840) {
  H_26(state, unused, out_1862972759044706840);
}
void car_h_27(double *state, double *unused, double *out_7596020352265097065) {
  h_27(state, unused, out_7596020352265097065);
}
void car_H_27(double *state, double *unused, double *out_425571699114315597) {
  H_27(state, unused, out_425571699114315597);
}
void car_h_29(double *state, double *unused, double *out_4599627427431185905) {
  h_29(state, unused, out_4599627427431185905);
}
void car_H_29(double *state, double *unused, double *out_2259422957000501498) {
  H_29(state, unused, out_2259422957000501498);
}
void car_h_28(double *state, double *unused, double *out_8763764153341238802) {
  h_28(state, unused, out_8763764153341238802);
}
void car_H_28(double *state, double *unused, double *out_7221333443053397204) {
  H_28(state, unused, out_7221333443053397204);
}
void car_h_31(double *state, double *unused, double *out_7299681846412492073) {
  h_31(state, unused, out_7299681846412492073);
}
void car_H_31(double *state, double *unused, double *out_1909176521706309812) {
  H_31(state, unused, out_1909176521706309812);
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
