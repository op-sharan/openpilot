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
void err_fun(double *nom_x, double *delta_x, double *out_1527400592183565183) {
   out_1527400592183565183[0] = delta_x[0] + nom_x[0];
   out_1527400592183565183[1] = delta_x[1] + nom_x[1];
   out_1527400592183565183[2] = delta_x[2] + nom_x[2];
   out_1527400592183565183[3] = delta_x[3] + nom_x[3];
   out_1527400592183565183[4] = delta_x[4] + nom_x[4];
   out_1527400592183565183[5] = delta_x[5] + nom_x[5];
   out_1527400592183565183[6] = delta_x[6] + nom_x[6];
   out_1527400592183565183[7] = delta_x[7] + nom_x[7];
   out_1527400592183565183[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4038333929682939877) {
   out_4038333929682939877[0] = -nom_x[0] + true_x[0];
   out_4038333929682939877[1] = -nom_x[1] + true_x[1];
   out_4038333929682939877[2] = -nom_x[2] + true_x[2];
   out_4038333929682939877[3] = -nom_x[3] + true_x[3];
   out_4038333929682939877[4] = -nom_x[4] + true_x[4];
   out_4038333929682939877[5] = -nom_x[5] + true_x[5];
   out_4038333929682939877[6] = -nom_x[6] + true_x[6];
   out_4038333929682939877[7] = -nom_x[7] + true_x[7];
   out_4038333929682939877[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8049619905800951591) {
   out_8049619905800951591[0] = 1.0;
   out_8049619905800951591[1] = 0.0;
   out_8049619905800951591[2] = 0.0;
   out_8049619905800951591[3] = 0.0;
   out_8049619905800951591[4] = 0.0;
   out_8049619905800951591[5] = 0.0;
   out_8049619905800951591[6] = 0.0;
   out_8049619905800951591[7] = 0.0;
   out_8049619905800951591[8] = 0.0;
   out_8049619905800951591[9] = 0.0;
   out_8049619905800951591[10] = 1.0;
   out_8049619905800951591[11] = 0.0;
   out_8049619905800951591[12] = 0.0;
   out_8049619905800951591[13] = 0.0;
   out_8049619905800951591[14] = 0.0;
   out_8049619905800951591[15] = 0.0;
   out_8049619905800951591[16] = 0.0;
   out_8049619905800951591[17] = 0.0;
   out_8049619905800951591[18] = 0.0;
   out_8049619905800951591[19] = 0.0;
   out_8049619905800951591[20] = 1.0;
   out_8049619905800951591[21] = 0.0;
   out_8049619905800951591[22] = 0.0;
   out_8049619905800951591[23] = 0.0;
   out_8049619905800951591[24] = 0.0;
   out_8049619905800951591[25] = 0.0;
   out_8049619905800951591[26] = 0.0;
   out_8049619905800951591[27] = 0.0;
   out_8049619905800951591[28] = 0.0;
   out_8049619905800951591[29] = 0.0;
   out_8049619905800951591[30] = 1.0;
   out_8049619905800951591[31] = 0.0;
   out_8049619905800951591[32] = 0.0;
   out_8049619905800951591[33] = 0.0;
   out_8049619905800951591[34] = 0.0;
   out_8049619905800951591[35] = 0.0;
   out_8049619905800951591[36] = 0.0;
   out_8049619905800951591[37] = 0.0;
   out_8049619905800951591[38] = 0.0;
   out_8049619905800951591[39] = 0.0;
   out_8049619905800951591[40] = 1.0;
   out_8049619905800951591[41] = 0.0;
   out_8049619905800951591[42] = 0.0;
   out_8049619905800951591[43] = 0.0;
   out_8049619905800951591[44] = 0.0;
   out_8049619905800951591[45] = 0.0;
   out_8049619905800951591[46] = 0.0;
   out_8049619905800951591[47] = 0.0;
   out_8049619905800951591[48] = 0.0;
   out_8049619905800951591[49] = 0.0;
   out_8049619905800951591[50] = 1.0;
   out_8049619905800951591[51] = 0.0;
   out_8049619905800951591[52] = 0.0;
   out_8049619905800951591[53] = 0.0;
   out_8049619905800951591[54] = 0.0;
   out_8049619905800951591[55] = 0.0;
   out_8049619905800951591[56] = 0.0;
   out_8049619905800951591[57] = 0.0;
   out_8049619905800951591[58] = 0.0;
   out_8049619905800951591[59] = 0.0;
   out_8049619905800951591[60] = 1.0;
   out_8049619905800951591[61] = 0.0;
   out_8049619905800951591[62] = 0.0;
   out_8049619905800951591[63] = 0.0;
   out_8049619905800951591[64] = 0.0;
   out_8049619905800951591[65] = 0.0;
   out_8049619905800951591[66] = 0.0;
   out_8049619905800951591[67] = 0.0;
   out_8049619905800951591[68] = 0.0;
   out_8049619905800951591[69] = 0.0;
   out_8049619905800951591[70] = 1.0;
   out_8049619905800951591[71] = 0.0;
   out_8049619905800951591[72] = 0.0;
   out_8049619905800951591[73] = 0.0;
   out_8049619905800951591[74] = 0.0;
   out_8049619905800951591[75] = 0.0;
   out_8049619905800951591[76] = 0.0;
   out_8049619905800951591[77] = 0.0;
   out_8049619905800951591[78] = 0.0;
   out_8049619905800951591[79] = 0.0;
   out_8049619905800951591[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7680940422259587756) {
   out_7680940422259587756[0] = state[0];
   out_7680940422259587756[1] = state[1];
   out_7680940422259587756[2] = state[2];
   out_7680940422259587756[3] = state[3];
   out_7680940422259587756[4] = state[4];
   out_7680940422259587756[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7680940422259587756[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7680940422259587756[7] = state[7];
   out_7680940422259587756[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8373137223162851881) {
   out_8373137223162851881[0] = 1;
   out_8373137223162851881[1] = 0;
   out_8373137223162851881[2] = 0;
   out_8373137223162851881[3] = 0;
   out_8373137223162851881[4] = 0;
   out_8373137223162851881[5] = 0;
   out_8373137223162851881[6] = 0;
   out_8373137223162851881[7] = 0;
   out_8373137223162851881[8] = 0;
   out_8373137223162851881[9] = 0;
   out_8373137223162851881[10] = 1;
   out_8373137223162851881[11] = 0;
   out_8373137223162851881[12] = 0;
   out_8373137223162851881[13] = 0;
   out_8373137223162851881[14] = 0;
   out_8373137223162851881[15] = 0;
   out_8373137223162851881[16] = 0;
   out_8373137223162851881[17] = 0;
   out_8373137223162851881[18] = 0;
   out_8373137223162851881[19] = 0;
   out_8373137223162851881[20] = 1;
   out_8373137223162851881[21] = 0;
   out_8373137223162851881[22] = 0;
   out_8373137223162851881[23] = 0;
   out_8373137223162851881[24] = 0;
   out_8373137223162851881[25] = 0;
   out_8373137223162851881[26] = 0;
   out_8373137223162851881[27] = 0;
   out_8373137223162851881[28] = 0;
   out_8373137223162851881[29] = 0;
   out_8373137223162851881[30] = 1;
   out_8373137223162851881[31] = 0;
   out_8373137223162851881[32] = 0;
   out_8373137223162851881[33] = 0;
   out_8373137223162851881[34] = 0;
   out_8373137223162851881[35] = 0;
   out_8373137223162851881[36] = 0;
   out_8373137223162851881[37] = 0;
   out_8373137223162851881[38] = 0;
   out_8373137223162851881[39] = 0;
   out_8373137223162851881[40] = 1;
   out_8373137223162851881[41] = 0;
   out_8373137223162851881[42] = 0;
   out_8373137223162851881[43] = 0;
   out_8373137223162851881[44] = 0;
   out_8373137223162851881[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8373137223162851881[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8373137223162851881[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8373137223162851881[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8373137223162851881[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8373137223162851881[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8373137223162851881[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8373137223162851881[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8373137223162851881[53] = -9.8100000000000005*dt;
   out_8373137223162851881[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8373137223162851881[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8373137223162851881[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8373137223162851881[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8373137223162851881[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8373137223162851881[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8373137223162851881[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8373137223162851881[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8373137223162851881[62] = 0;
   out_8373137223162851881[63] = 0;
   out_8373137223162851881[64] = 0;
   out_8373137223162851881[65] = 0;
   out_8373137223162851881[66] = 0;
   out_8373137223162851881[67] = 0;
   out_8373137223162851881[68] = 0;
   out_8373137223162851881[69] = 0;
   out_8373137223162851881[70] = 1;
   out_8373137223162851881[71] = 0;
   out_8373137223162851881[72] = 0;
   out_8373137223162851881[73] = 0;
   out_8373137223162851881[74] = 0;
   out_8373137223162851881[75] = 0;
   out_8373137223162851881[76] = 0;
   out_8373137223162851881[77] = 0;
   out_8373137223162851881[78] = 0;
   out_8373137223162851881[79] = 0;
   out_8373137223162851881[80] = 1;
}
void h_25(double *state, double *unused, double *out_5355327156585439901) {
   out_5355327156585439901[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2475786134934740220) {
   out_2475786134934740220[0] = 0;
   out_2475786134934740220[1] = 0;
   out_2475786134934740220[2] = 0;
   out_2475786134934740220[3] = 0;
   out_2475786134934740220[4] = 0;
   out_2475786134934740220[5] = 0;
   out_2475786134934740220[6] = 1;
   out_2475786134934740220[7] = 0;
   out_2475786134934740220[8] = 0;
}
void h_24(double *state, double *unused, double *out_4948890380757191940) {
   out_4948890380757191940[0] = state[4];
   out_4948890380757191940[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3316217919116785212) {
   out_3316217919116785212[0] = 0;
   out_3316217919116785212[1] = 0;
   out_3316217919116785212[2] = 0;
   out_3316217919116785212[3] = 0;
   out_3316217919116785212[4] = 1;
   out_3316217919116785212[5] = 0;
   out_3316217919116785212[6] = 0;
   out_3316217919116785212[7] = 0;
   out_3316217919116785212[8] = 0;
   out_3316217919116785212[9] = 0;
   out_3316217919116785212[10] = 0;
   out_3316217919116785212[11] = 0;
   out_3316217919116785212[12] = 0;
   out_3316217919116785212[13] = 0;
   out_3316217919116785212[14] = 1;
   out_3316217919116785212[15] = 0;
   out_3316217919116785212[16] = 0;
   out_3316217919116785212[17] = 0;
}
void h_30(double *state, double *unused, double *out_6805958549214323068) {
   out_6805958549214323068[0] = state[4];
}
void H_30(double *state, double *unused, double *out_42546823572508407) {
   out_42546823572508407[0] = 0;
   out_42546823572508407[1] = 0;
   out_42546823572508407[2] = 0;
   out_42546823572508407[3] = 0;
   out_42546823572508407[4] = 1;
   out_42546823572508407[5] = 0;
   out_42546823572508407[6] = 0;
   out_42546823572508407[7] = 0;
   out_42546823572508407[8] = 0;
}
void h_26(double *state, double *unused, double *out_2475727773492236361) {
   out_2475727773492236361[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6217289453808796444) {
   out_6217289453808796444[0] = 0;
   out_6217289453808796444[1] = 0;
   out_6217289453808796444[2] = 0;
   out_6217289453808796444[3] = 0;
   out_6217289453808796444[4] = 0;
   out_6217289453808796444[5] = 0;
   out_6217289453808796444[6] = 0;
   out_6217289453808796444[7] = 1;
   out_6217289453808796444[8] = 0;
}
void h_27(double *state, double *unused, double *out_353653888924594256) {
   out_353653888924594256[0] = state[3];
}
void H_27(double *state, double *unused, double *out_9178245776862773329) {
   out_9178245776862773329[0] = 0;
   out_9178245776862773329[1] = 0;
   out_9178245776862773329[2] = 0;
   out_9178245776862773329[3] = 1;
   out_9178245776862773329[4] = 0;
   out_9178245776862773329[5] = 0;
   out_9178245776862773329[6] = 0;
   out_9178245776862773329[7] = 0;
   out_9178245776862773329[8] = 0;
}
void h_29(double *state, double *unused, double *out_1804285281553477423) {
   out_1804285281553477423[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6493251120747956234) {
   out_6493251120747956234[0] = 0;
   out_6493251120747956234[1] = 1;
   out_6493251120747956234[2] = 0;
   out_6493251120747956234[3] = 0;
   out_6493251120747956234[4] = 0;
   out_6493251120747956234[5] = 0;
   out_6493251120747956234[6] = 0;
   out_6493251120747956234[7] = 0;
   out_6493251120747956234[8] = 0;
}
void h_28(double *state, double *unused, double *out_4712763629793748467) {
   out_4712763629793748467[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4529620849182629983) {
   out_4529620849182629983[0] = 1;
   out_4529620849182629983[1] = 0;
   out_4529620849182629983[2] = 0;
   out_4529620849182629983[3] = 0;
   out_4529620849182629983[4] = 0;
   out_4529620849182629983[5] = 0;
   out_4529620849182629983[6] = 0;
   out_4529620849182629983[7] = 0;
   out_4529620849182629983[8] = 0;
}
void h_31(double *state, double *unused, double *out_5630521218869945790) {
   out_5630521218869945790[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6843497556042147920) {
   out_6843497556042147920[0] = 0;
   out_6843497556042147920[1] = 0;
   out_6843497556042147920[2] = 0;
   out_6843497556042147920[3] = 0;
   out_6843497556042147920[4] = 0;
   out_6843497556042147920[5] = 0;
   out_6843497556042147920[6] = 0;
   out_6843497556042147920[7] = 0;
   out_6843497556042147920[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1527400592183565183) {
  err_fun(nom_x, delta_x, out_1527400592183565183);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4038333929682939877) {
  inv_err_fun(nom_x, true_x, out_4038333929682939877);
}
void car_H_mod_fun(double *state, double *out_8049619905800951591) {
  H_mod_fun(state, out_8049619905800951591);
}
void car_f_fun(double *state, double dt, double *out_7680940422259587756) {
  f_fun(state,  dt, out_7680940422259587756);
}
void car_F_fun(double *state, double dt, double *out_8373137223162851881) {
  F_fun(state,  dt, out_8373137223162851881);
}
void car_h_25(double *state, double *unused, double *out_5355327156585439901) {
  h_25(state, unused, out_5355327156585439901);
}
void car_H_25(double *state, double *unused, double *out_2475786134934740220) {
  H_25(state, unused, out_2475786134934740220);
}
void car_h_24(double *state, double *unused, double *out_4948890380757191940) {
  h_24(state, unused, out_4948890380757191940);
}
void car_H_24(double *state, double *unused, double *out_3316217919116785212) {
  H_24(state, unused, out_3316217919116785212);
}
void car_h_30(double *state, double *unused, double *out_6805958549214323068) {
  h_30(state, unused, out_6805958549214323068);
}
void car_H_30(double *state, double *unused, double *out_42546823572508407) {
  H_30(state, unused, out_42546823572508407);
}
void car_h_26(double *state, double *unused, double *out_2475727773492236361) {
  h_26(state, unused, out_2475727773492236361);
}
void car_H_26(double *state, double *unused, double *out_6217289453808796444) {
  H_26(state, unused, out_6217289453808796444);
}
void car_h_27(double *state, double *unused, double *out_353653888924594256) {
  h_27(state, unused, out_353653888924594256);
}
void car_H_27(double *state, double *unused, double *out_9178245776862773329) {
  H_27(state, unused, out_9178245776862773329);
}
void car_h_29(double *state, double *unused, double *out_1804285281553477423) {
  h_29(state, unused, out_1804285281553477423);
}
void car_H_29(double *state, double *unused, double *out_6493251120747956234) {
  H_29(state, unused, out_6493251120747956234);
}
void car_h_28(double *state, double *unused, double *out_4712763629793748467) {
  h_28(state, unused, out_4712763629793748467);
}
void car_H_28(double *state, double *unused, double *out_4529620849182629983) {
  H_28(state, unused, out_4529620849182629983);
}
void car_h_31(double *state, double *unused, double *out_5630521218869945790) {
  h_31(state, unused, out_5630521218869945790);
}
void car_H_31(double *state, double *unused, double *out_6843497556042147920) {
  H_31(state, unused, out_6843497556042147920);
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
