#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2128814047154352200) {
   out_2128814047154352200[0] = delta_x[0] + nom_x[0];
   out_2128814047154352200[1] = delta_x[1] + nom_x[1];
   out_2128814047154352200[2] = delta_x[2] + nom_x[2];
   out_2128814047154352200[3] = delta_x[3] + nom_x[3];
   out_2128814047154352200[4] = delta_x[4] + nom_x[4];
   out_2128814047154352200[5] = delta_x[5] + nom_x[5];
   out_2128814047154352200[6] = delta_x[6] + nom_x[6];
   out_2128814047154352200[7] = delta_x[7] + nom_x[7];
   out_2128814047154352200[8] = delta_x[8] + nom_x[8];
   out_2128814047154352200[9] = delta_x[9] + nom_x[9];
   out_2128814047154352200[10] = delta_x[10] + nom_x[10];
   out_2128814047154352200[11] = delta_x[11] + nom_x[11];
   out_2128814047154352200[12] = delta_x[12] + nom_x[12];
   out_2128814047154352200[13] = delta_x[13] + nom_x[13];
   out_2128814047154352200[14] = delta_x[14] + nom_x[14];
   out_2128814047154352200[15] = delta_x[15] + nom_x[15];
   out_2128814047154352200[16] = delta_x[16] + nom_x[16];
   out_2128814047154352200[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7001406389145399648) {
   out_7001406389145399648[0] = -nom_x[0] + true_x[0];
   out_7001406389145399648[1] = -nom_x[1] + true_x[1];
   out_7001406389145399648[2] = -nom_x[2] + true_x[2];
   out_7001406389145399648[3] = -nom_x[3] + true_x[3];
   out_7001406389145399648[4] = -nom_x[4] + true_x[4];
   out_7001406389145399648[5] = -nom_x[5] + true_x[5];
   out_7001406389145399648[6] = -nom_x[6] + true_x[6];
   out_7001406389145399648[7] = -nom_x[7] + true_x[7];
   out_7001406389145399648[8] = -nom_x[8] + true_x[8];
   out_7001406389145399648[9] = -nom_x[9] + true_x[9];
   out_7001406389145399648[10] = -nom_x[10] + true_x[10];
   out_7001406389145399648[11] = -nom_x[11] + true_x[11];
   out_7001406389145399648[12] = -nom_x[12] + true_x[12];
   out_7001406389145399648[13] = -nom_x[13] + true_x[13];
   out_7001406389145399648[14] = -nom_x[14] + true_x[14];
   out_7001406389145399648[15] = -nom_x[15] + true_x[15];
   out_7001406389145399648[16] = -nom_x[16] + true_x[16];
   out_7001406389145399648[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1783591311774877036) {
   out_1783591311774877036[0] = 1.0;
   out_1783591311774877036[1] = 0.0;
   out_1783591311774877036[2] = 0.0;
   out_1783591311774877036[3] = 0.0;
   out_1783591311774877036[4] = 0.0;
   out_1783591311774877036[5] = 0.0;
   out_1783591311774877036[6] = 0.0;
   out_1783591311774877036[7] = 0.0;
   out_1783591311774877036[8] = 0.0;
   out_1783591311774877036[9] = 0.0;
   out_1783591311774877036[10] = 0.0;
   out_1783591311774877036[11] = 0.0;
   out_1783591311774877036[12] = 0.0;
   out_1783591311774877036[13] = 0.0;
   out_1783591311774877036[14] = 0.0;
   out_1783591311774877036[15] = 0.0;
   out_1783591311774877036[16] = 0.0;
   out_1783591311774877036[17] = 0.0;
   out_1783591311774877036[18] = 0.0;
   out_1783591311774877036[19] = 1.0;
   out_1783591311774877036[20] = 0.0;
   out_1783591311774877036[21] = 0.0;
   out_1783591311774877036[22] = 0.0;
   out_1783591311774877036[23] = 0.0;
   out_1783591311774877036[24] = 0.0;
   out_1783591311774877036[25] = 0.0;
   out_1783591311774877036[26] = 0.0;
   out_1783591311774877036[27] = 0.0;
   out_1783591311774877036[28] = 0.0;
   out_1783591311774877036[29] = 0.0;
   out_1783591311774877036[30] = 0.0;
   out_1783591311774877036[31] = 0.0;
   out_1783591311774877036[32] = 0.0;
   out_1783591311774877036[33] = 0.0;
   out_1783591311774877036[34] = 0.0;
   out_1783591311774877036[35] = 0.0;
   out_1783591311774877036[36] = 0.0;
   out_1783591311774877036[37] = 0.0;
   out_1783591311774877036[38] = 1.0;
   out_1783591311774877036[39] = 0.0;
   out_1783591311774877036[40] = 0.0;
   out_1783591311774877036[41] = 0.0;
   out_1783591311774877036[42] = 0.0;
   out_1783591311774877036[43] = 0.0;
   out_1783591311774877036[44] = 0.0;
   out_1783591311774877036[45] = 0.0;
   out_1783591311774877036[46] = 0.0;
   out_1783591311774877036[47] = 0.0;
   out_1783591311774877036[48] = 0.0;
   out_1783591311774877036[49] = 0.0;
   out_1783591311774877036[50] = 0.0;
   out_1783591311774877036[51] = 0.0;
   out_1783591311774877036[52] = 0.0;
   out_1783591311774877036[53] = 0.0;
   out_1783591311774877036[54] = 0.0;
   out_1783591311774877036[55] = 0.0;
   out_1783591311774877036[56] = 0.0;
   out_1783591311774877036[57] = 1.0;
   out_1783591311774877036[58] = 0.0;
   out_1783591311774877036[59] = 0.0;
   out_1783591311774877036[60] = 0.0;
   out_1783591311774877036[61] = 0.0;
   out_1783591311774877036[62] = 0.0;
   out_1783591311774877036[63] = 0.0;
   out_1783591311774877036[64] = 0.0;
   out_1783591311774877036[65] = 0.0;
   out_1783591311774877036[66] = 0.0;
   out_1783591311774877036[67] = 0.0;
   out_1783591311774877036[68] = 0.0;
   out_1783591311774877036[69] = 0.0;
   out_1783591311774877036[70] = 0.0;
   out_1783591311774877036[71] = 0.0;
   out_1783591311774877036[72] = 0.0;
   out_1783591311774877036[73] = 0.0;
   out_1783591311774877036[74] = 0.0;
   out_1783591311774877036[75] = 0.0;
   out_1783591311774877036[76] = 1.0;
   out_1783591311774877036[77] = 0.0;
   out_1783591311774877036[78] = 0.0;
   out_1783591311774877036[79] = 0.0;
   out_1783591311774877036[80] = 0.0;
   out_1783591311774877036[81] = 0.0;
   out_1783591311774877036[82] = 0.0;
   out_1783591311774877036[83] = 0.0;
   out_1783591311774877036[84] = 0.0;
   out_1783591311774877036[85] = 0.0;
   out_1783591311774877036[86] = 0.0;
   out_1783591311774877036[87] = 0.0;
   out_1783591311774877036[88] = 0.0;
   out_1783591311774877036[89] = 0.0;
   out_1783591311774877036[90] = 0.0;
   out_1783591311774877036[91] = 0.0;
   out_1783591311774877036[92] = 0.0;
   out_1783591311774877036[93] = 0.0;
   out_1783591311774877036[94] = 0.0;
   out_1783591311774877036[95] = 1.0;
   out_1783591311774877036[96] = 0.0;
   out_1783591311774877036[97] = 0.0;
   out_1783591311774877036[98] = 0.0;
   out_1783591311774877036[99] = 0.0;
   out_1783591311774877036[100] = 0.0;
   out_1783591311774877036[101] = 0.0;
   out_1783591311774877036[102] = 0.0;
   out_1783591311774877036[103] = 0.0;
   out_1783591311774877036[104] = 0.0;
   out_1783591311774877036[105] = 0.0;
   out_1783591311774877036[106] = 0.0;
   out_1783591311774877036[107] = 0.0;
   out_1783591311774877036[108] = 0.0;
   out_1783591311774877036[109] = 0.0;
   out_1783591311774877036[110] = 0.0;
   out_1783591311774877036[111] = 0.0;
   out_1783591311774877036[112] = 0.0;
   out_1783591311774877036[113] = 0.0;
   out_1783591311774877036[114] = 1.0;
   out_1783591311774877036[115] = 0.0;
   out_1783591311774877036[116] = 0.0;
   out_1783591311774877036[117] = 0.0;
   out_1783591311774877036[118] = 0.0;
   out_1783591311774877036[119] = 0.0;
   out_1783591311774877036[120] = 0.0;
   out_1783591311774877036[121] = 0.0;
   out_1783591311774877036[122] = 0.0;
   out_1783591311774877036[123] = 0.0;
   out_1783591311774877036[124] = 0.0;
   out_1783591311774877036[125] = 0.0;
   out_1783591311774877036[126] = 0.0;
   out_1783591311774877036[127] = 0.0;
   out_1783591311774877036[128] = 0.0;
   out_1783591311774877036[129] = 0.0;
   out_1783591311774877036[130] = 0.0;
   out_1783591311774877036[131] = 0.0;
   out_1783591311774877036[132] = 0.0;
   out_1783591311774877036[133] = 1.0;
   out_1783591311774877036[134] = 0.0;
   out_1783591311774877036[135] = 0.0;
   out_1783591311774877036[136] = 0.0;
   out_1783591311774877036[137] = 0.0;
   out_1783591311774877036[138] = 0.0;
   out_1783591311774877036[139] = 0.0;
   out_1783591311774877036[140] = 0.0;
   out_1783591311774877036[141] = 0.0;
   out_1783591311774877036[142] = 0.0;
   out_1783591311774877036[143] = 0.0;
   out_1783591311774877036[144] = 0.0;
   out_1783591311774877036[145] = 0.0;
   out_1783591311774877036[146] = 0.0;
   out_1783591311774877036[147] = 0.0;
   out_1783591311774877036[148] = 0.0;
   out_1783591311774877036[149] = 0.0;
   out_1783591311774877036[150] = 0.0;
   out_1783591311774877036[151] = 0.0;
   out_1783591311774877036[152] = 1.0;
   out_1783591311774877036[153] = 0.0;
   out_1783591311774877036[154] = 0.0;
   out_1783591311774877036[155] = 0.0;
   out_1783591311774877036[156] = 0.0;
   out_1783591311774877036[157] = 0.0;
   out_1783591311774877036[158] = 0.0;
   out_1783591311774877036[159] = 0.0;
   out_1783591311774877036[160] = 0.0;
   out_1783591311774877036[161] = 0.0;
   out_1783591311774877036[162] = 0.0;
   out_1783591311774877036[163] = 0.0;
   out_1783591311774877036[164] = 0.0;
   out_1783591311774877036[165] = 0.0;
   out_1783591311774877036[166] = 0.0;
   out_1783591311774877036[167] = 0.0;
   out_1783591311774877036[168] = 0.0;
   out_1783591311774877036[169] = 0.0;
   out_1783591311774877036[170] = 0.0;
   out_1783591311774877036[171] = 1.0;
   out_1783591311774877036[172] = 0.0;
   out_1783591311774877036[173] = 0.0;
   out_1783591311774877036[174] = 0.0;
   out_1783591311774877036[175] = 0.0;
   out_1783591311774877036[176] = 0.0;
   out_1783591311774877036[177] = 0.0;
   out_1783591311774877036[178] = 0.0;
   out_1783591311774877036[179] = 0.0;
   out_1783591311774877036[180] = 0.0;
   out_1783591311774877036[181] = 0.0;
   out_1783591311774877036[182] = 0.0;
   out_1783591311774877036[183] = 0.0;
   out_1783591311774877036[184] = 0.0;
   out_1783591311774877036[185] = 0.0;
   out_1783591311774877036[186] = 0.0;
   out_1783591311774877036[187] = 0.0;
   out_1783591311774877036[188] = 0.0;
   out_1783591311774877036[189] = 0.0;
   out_1783591311774877036[190] = 1.0;
   out_1783591311774877036[191] = 0.0;
   out_1783591311774877036[192] = 0.0;
   out_1783591311774877036[193] = 0.0;
   out_1783591311774877036[194] = 0.0;
   out_1783591311774877036[195] = 0.0;
   out_1783591311774877036[196] = 0.0;
   out_1783591311774877036[197] = 0.0;
   out_1783591311774877036[198] = 0.0;
   out_1783591311774877036[199] = 0.0;
   out_1783591311774877036[200] = 0.0;
   out_1783591311774877036[201] = 0.0;
   out_1783591311774877036[202] = 0.0;
   out_1783591311774877036[203] = 0.0;
   out_1783591311774877036[204] = 0.0;
   out_1783591311774877036[205] = 0.0;
   out_1783591311774877036[206] = 0.0;
   out_1783591311774877036[207] = 0.0;
   out_1783591311774877036[208] = 0.0;
   out_1783591311774877036[209] = 1.0;
   out_1783591311774877036[210] = 0.0;
   out_1783591311774877036[211] = 0.0;
   out_1783591311774877036[212] = 0.0;
   out_1783591311774877036[213] = 0.0;
   out_1783591311774877036[214] = 0.0;
   out_1783591311774877036[215] = 0.0;
   out_1783591311774877036[216] = 0.0;
   out_1783591311774877036[217] = 0.0;
   out_1783591311774877036[218] = 0.0;
   out_1783591311774877036[219] = 0.0;
   out_1783591311774877036[220] = 0.0;
   out_1783591311774877036[221] = 0.0;
   out_1783591311774877036[222] = 0.0;
   out_1783591311774877036[223] = 0.0;
   out_1783591311774877036[224] = 0.0;
   out_1783591311774877036[225] = 0.0;
   out_1783591311774877036[226] = 0.0;
   out_1783591311774877036[227] = 0.0;
   out_1783591311774877036[228] = 1.0;
   out_1783591311774877036[229] = 0.0;
   out_1783591311774877036[230] = 0.0;
   out_1783591311774877036[231] = 0.0;
   out_1783591311774877036[232] = 0.0;
   out_1783591311774877036[233] = 0.0;
   out_1783591311774877036[234] = 0.0;
   out_1783591311774877036[235] = 0.0;
   out_1783591311774877036[236] = 0.0;
   out_1783591311774877036[237] = 0.0;
   out_1783591311774877036[238] = 0.0;
   out_1783591311774877036[239] = 0.0;
   out_1783591311774877036[240] = 0.0;
   out_1783591311774877036[241] = 0.0;
   out_1783591311774877036[242] = 0.0;
   out_1783591311774877036[243] = 0.0;
   out_1783591311774877036[244] = 0.0;
   out_1783591311774877036[245] = 0.0;
   out_1783591311774877036[246] = 0.0;
   out_1783591311774877036[247] = 1.0;
   out_1783591311774877036[248] = 0.0;
   out_1783591311774877036[249] = 0.0;
   out_1783591311774877036[250] = 0.0;
   out_1783591311774877036[251] = 0.0;
   out_1783591311774877036[252] = 0.0;
   out_1783591311774877036[253] = 0.0;
   out_1783591311774877036[254] = 0.0;
   out_1783591311774877036[255] = 0.0;
   out_1783591311774877036[256] = 0.0;
   out_1783591311774877036[257] = 0.0;
   out_1783591311774877036[258] = 0.0;
   out_1783591311774877036[259] = 0.0;
   out_1783591311774877036[260] = 0.0;
   out_1783591311774877036[261] = 0.0;
   out_1783591311774877036[262] = 0.0;
   out_1783591311774877036[263] = 0.0;
   out_1783591311774877036[264] = 0.0;
   out_1783591311774877036[265] = 0.0;
   out_1783591311774877036[266] = 1.0;
   out_1783591311774877036[267] = 0.0;
   out_1783591311774877036[268] = 0.0;
   out_1783591311774877036[269] = 0.0;
   out_1783591311774877036[270] = 0.0;
   out_1783591311774877036[271] = 0.0;
   out_1783591311774877036[272] = 0.0;
   out_1783591311774877036[273] = 0.0;
   out_1783591311774877036[274] = 0.0;
   out_1783591311774877036[275] = 0.0;
   out_1783591311774877036[276] = 0.0;
   out_1783591311774877036[277] = 0.0;
   out_1783591311774877036[278] = 0.0;
   out_1783591311774877036[279] = 0.0;
   out_1783591311774877036[280] = 0.0;
   out_1783591311774877036[281] = 0.0;
   out_1783591311774877036[282] = 0.0;
   out_1783591311774877036[283] = 0.0;
   out_1783591311774877036[284] = 0.0;
   out_1783591311774877036[285] = 1.0;
   out_1783591311774877036[286] = 0.0;
   out_1783591311774877036[287] = 0.0;
   out_1783591311774877036[288] = 0.0;
   out_1783591311774877036[289] = 0.0;
   out_1783591311774877036[290] = 0.0;
   out_1783591311774877036[291] = 0.0;
   out_1783591311774877036[292] = 0.0;
   out_1783591311774877036[293] = 0.0;
   out_1783591311774877036[294] = 0.0;
   out_1783591311774877036[295] = 0.0;
   out_1783591311774877036[296] = 0.0;
   out_1783591311774877036[297] = 0.0;
   out_1783591311774877036[298] = 0.0;
   out_1783591311774877036[299] = 0.0;
   out_1783591311774877036[300] = 0.0;
   out_1783591311774877036[301] = 0.0;
   out_1783591311774877036[302] = 0.0;
   out_1783591311774877036[303] = 0.0;
   out_1783591311774877036[304] = 1.0;
   out_1783591311774877036[305] = 0.0;
   out_1783591311774877036[306] = 0.0;
   out_1783591311774877036[307] = 0.0;
   out_1783591311774877036[308] = 0.0;
   out_1783591311774877036[309] = 0.0;
   out_1783591311774877036[310] = 0.0;
   out_1783591311774877036[311] = 0.0;
   out_1783591311774877036[312] = 0.0;
   out_1783591311774877036[313] = 0.0;
   out_1783591311774877036[314] = 0.0;
   out_1783591311774877036[315] = 0.0;
   out_1783591311774877036[316] = 0.0;
   out_1783591311774877036[317] = 0.0;
   out_1783591311774877036[318] = 0.0;
   out_1783591311774877036[319] = 0.0;
   out_1783591311774877036[320] = 0.0;
   out_1783591311774877036[321] = 0.0;
   out_1783591311774877036[322] = 0.0;
   out_1783591311774877036[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3415300158525823285) {
   out_3415300158525823285[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3415300158525823285[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3415300158525823285[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3415300158525823285[3] = dt*state[12] + state[3];
   out_3415300158525823285[4] = dt*state[13] + state[4];
   out_3415300158525823285[5] = dt*state[14] + state[5];
   out_3415300158525823285[6] = state[6];
   out_3415300158525823285[7] = state[7];
   out_3415300158525823285[8] = state[8];
   out_3415300158525823285[9] = state[9];
   out_3415300158525823285[10] = state[10];
   out_3415300158525823285[11] = state[11];
   out_3415300158525823285[12] = state[12];
   out_3415300158525823285[13] = state[13];
   out_3415300158525823285[14] = state[14];
   out_3415300158525823285[15] = state[15];
   out_3415300158525823285[16] = state[16];
   out_3415300158525823285[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5538305373514744468) {
   out_5538305373514744468[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5538305373514744468[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5538305373514744468[2] = 0;
   out_5538305373514744468[3] = 0;
   out_5538305373514744468[4] = 0;
   out_5538305373514744468[5] = 0;
   out_5538305373514744468[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5538305373514744468[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5538305373514744468[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5538305373514744468[9] = 0;
   out_5538305373514744468[10] = 0;
   out_5538305373514744468[11] = 0;
   out_5538305373514744468[12] = 0;
   out_5538305373514744468[13] = 0;
   out_5538305373514744468[14] = 0;
   out_5538305373514744468[15] = 0;
   out_5538305373514744468[16] = 0;
   out_5538305373514744468[17] = 0;
   out_5538305373514744468[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5538305373514744468[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5538305373514744468[20] = 0;
   out_5538305373514744468[21] = 0;
   out_5538305373514744468[22] = 0;
   out_5538305373514744468[23] = 0;
   out_5538305373514744468[24] = 0;
   out_5538305373514744468[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5538305373514744468[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5538305373514744468[27] = 0;
   out_5538305373514744468[28] = 0;
   out_5538305373514744468[29] = 0;
   out_5538305373514744468[30] = 0;
   out_5538305373514744468[31] = 0;
   out_5538305373514744468[32] = 0;
   out_5538305373514744468[33] = 0;
   out_5538305373514744468[34] = 0;
   out_5538305373514744468[35] = 0;
   out_5538305373514744468[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5538305373514744468[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5538305373514744468[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5538305373514744468[39] = 0;
   out_5538305373514744468[40] = 0;
   out_5538305373514744468[41] = 0;
   out_5538305373514744468[42] = 0;
   out_5538305373514744468[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5538305373514744468[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5538305373514744468[45] = 0;
   out_5538305373514744468[46] = 0;
   out_5538305373514744468[47] = 0;
   out_5538305373514744468[48] = 0;
   out_5538305373514744468[49] = 0;
   out_5538305373514744468[50] = 0;
   out_5538305373514744468[51] = 0;
   out_5538305373514744468[52] = 0;
   out_5538305373514744468[53] = 0;
   out_5538305373514744468[54] = 0;
   out_5538305373514744468[55] = 0;
   out_5538305373514744468[56] = 0;
   out_5538305373514744468[57] = 1;
   out_5538305373514744468[58] = 0;
   out_5538305373514744468[59] = 0;
   out_5538305373514744468[60] = 0;
   out_5538305373514744468[61] = 0;
   out_5538305373514744468[62] = 0;
   out_5538305373514744468[63] = 0;
   out_5538305373514744468[64] = 0;
   out_5538305373514744468[65] = 0;
   out_5538305373514744468[66] = dt;
   out_5538305373514744468[67] = 0;
   out_5538305373514744468[68] = 0;
   out_5538305373514744468[69] = 0;
   out_5538305373514744468[70] = 0;
   out_5538305373514744468[71] = 0;
   out_5538305373514744468[72] = 0;
   out_5538305373514744468[73] = 0;
   out_5538305373514744468[74] = 0;
   out_5538305373514744468[75] = 0;
   out_5538305373514744468[76] = 1;
   out_5538305373514744468[77] = 0;
   out_5538305373514744468[78] = 0;
   out_5538305373514744468[79] = 0;
   out_5538305373514744468[80] = 0;
   out_5538305373514744468[81] = 0;
   out_5538305373514744468[82] = 0;
   out_5538305373514744468[83] = 0;
   out_5538305373514744468[84] = 0;
   out_5538305373514744468[85] = dt;
   out_5538305373514744468[86] = 0;
   out_5538305373514744468[87] = 0;
   out_5538305373514744468[88] = 0;
   out_5538305373514744468[89] = 0;
   out_5538305373514744468[90] = 0;
   out_5538305373514744468[91] = 0;
   out_5538305373514744468[92] = 0;
   out_5538305373514744468[93] = 0;
   out_5538305373514744468[94] = 0;
   out_5538305373514744468[95] = 1;
   out_5538305373514744468[96] = 0;
   out_5538305373514744468[97] = 0;
   out_5538305373514744468[98] = 0;
   out_5538305373514744468[99] = 0;
   out_5538305373514744468[100] = 0;
   out_5538305373514744468[101] = 0;
   out_5538305373514744468[102] = 0;
   out_5538305373514744468[103] = 0;
   out_5538305373514744468[104] = dt;
   out_5538305373514744468[105] = 0;
   out_5538305373514744468[106] = 0;
   out_5538305373514744468[107] = 0;
   out_5538305373514744468[108] = 0;
   out_5538305373514744468[109] = 0;
   out_5538305373514744468[110] = 0;
   out_5538305373514744468[111] = 0;
   out_5538305373514744468[112] = 0;
   out_5538305373514744468[113] = 0;
   out_5538305373514744468[114] = 1;
   out_5538305373514744468[115] = 0;
   out_5538305373514744468[116] = 0;
   out_5538305373514744468[117] = 0;
   out_5538305373514744468[118] = 0;
   out_5538305373514744468[119] = 0;
   out_5538305373514744468[120] = 0;
   out_5538305373514744468[121] = 0;
   out_5538305373514744468[122] = 0;
   out_5538305373514744468[123] = 0;
   out_5538305373514744468[124] = 0;
   out_5538305373514744468[125] = 0;
   out_5538305373514744468[126] = 0;
   out_5538305373514744468[127] = 0;
   out_5538305373514744468[128] = 0;
   out_5538305373514744468[129] = 0;
   out_5538305373514744468[130] = 0;
   out_5538305373514744468[131] = 0;
   out_5538305373514744468[132] = 0;
   out_5538305373514744468[133] = 1;
   out_5538305373514744468[134] = 0;
   out_5538305373514744468[135] = 0;
   out_5538305373514744468[136] = 0;
   out_5538305373514744468[137] = 0;
   out_5538305373514744468[138] = 0;
   out_5538305373514744468[139] = 0;
   out_5538305373514744468[140] = 0;
   out_5538305373514744468[141] = 0;
   out_5538305373514744468[142] = 0;
   out_5538305373514744468[143] = 0;
   out_5538305373514744468[144] = 0;
   out_5538305373514744468[145] = 0;
   out_5538305373514744468[146] = 0;
   out_5538305373514744468[147] = 0;
   out_5538305373514744468[148] = 0;
   out_5538305373514744468[149] = 0;
   out_5538305373514744468[150] = 0;
   out_5538305373514744468[151] = 0;
   out_5538305373514744468[152] = 1;
   out_5538305373514744468[153] = 0;
   out_5538305373514744468[154] = 0;
   out_5538305373514744468[155] = 0;
   out_5538305373514744468[156] = 0;
   out_5538305373514744468[157] = 0;
   out_5538305373514744468[158] = 0;
   out_5538305373514744468[159] = 0;
   out_5538305373514744468[160] = 0;
   out_5538305373514744468[161] = 0;
   out_5538305373514744468[162] = 0;
   out_5538305373514744468[163] = 0;
   out_5538305373514744468[164] = 0;
   out_5538305373514744468[165] = 0;
   out_5538305373514744468[166] = 0;
   out_5538305373514744468[167] = 0;
   out_5538305373514744468[168] = 0;
   out_5538305373514744468[169] = 0;
   out_5538305373514744468[170] = 0;
   out_5538305373514744468[171] = 1;
   out_5538305373514744468[172] = 0;
   out_5538305373514744468[173] = 0;
   out_5538305373514744468[174] = 0;
   out_5538305373514744468[175] = 0;
   out_5538305373514744468[176] = 0;
   out_5538305373514744468[177] = 0;
   out_5538305373514744468[178] = 0;
   out_5538305373514744468[179] = 0;
   out_5538305373514744468[180] = 0;
   out_5538305373514744468[181] = 0;
   out_5538305373514744468[182] = 0;
   out_5538305373514744468[183] = 0;
   out_5538305373514744468[184] = 0;
   out_5538305373514744468[185] = 0;
   out_5538305373514744468[186] = 0;
   out_5538305373514744468[187] = 0;
   out_5538305373514744468[188] = 0;
   out_5538305373514744468[189] = 0;
   out_5538305373514744468[190] = 1;
   out_5538305373514744468[191] = 0;
   out_5538305373514744468[192] = 0;
   out_5538305373514744468[193] = 0;
   out_5538305373514744468[194] = 0;
   out_5538305373514744468[195] = 0;
   out_5538305373514744468[196] = 0;
   out_5538305373514744468[197] = 0;
   out_5538305373514744468[198] = 0;
   out_5538305373514744468[199] = 0;
   out_5538305373514744468[200] = 0;
   out_5538305373514744468[201] = 0;
   out_5538305373514744468[202] = 0;
   out_5538305373514744468[203] = 0;
   out_5538305373514744468[204] = 0;
   out_5538305373514744468[205] = 0;
   out_5538305373514744468[206] = 0;
   out_5538305373514744468[207] = 0;
   out_5538305373514744468[208] = 0;
   out_5538305373514744468[209] = 1;
   out_5538305373514744468[210] = 0;
   out_5538305373514744468[211] = 0;
   out_5538305373514744468[212] = 0;
   out_5538305373514744468[213] = 0;
   out_5538305373514744468[214] = 0;
   out_5538305373514744468[215] = 0;
   out_5538305373514744468[216] = 0;
   out_5538305373514744468[217] = 0;
   out_5538305373514744468[218] = 0;
   out_5538305373514744468[219] = 0;
   out_5538305373514744468[220] = 0;
   out_5538305373514744468[221] = 0;
   out_5538305373514744468[222] = 0;
   out_5538305373514744468[223] = 0;
   out_5538305373514744468[224] = 0;
   out_5538305373514744468[225] = 0;
   out_5538305373514744468[226] = 0;
   out_5538305373514744468[227] = 0;
   out_5538305373514744468[228] = 1;
   out_5538305373514744468[229] = 0;
   out_5538305373514744468[230] = 0;
   out_5538305373514744468[231] = 0;
   out_5538305373514744468[232] = 0;
   out_5538305373514744468[233] = 0;
   out_5538305373514744468[234] = 0;
   out_5538305373514744468[235] = 0;
   out_5538305373514744468[236] = 0;
   out_5538305373514744468[237] = 0;
   out_5538305373514744468[238] = 0;
   out_5538305373514744468[239] = 0;
   out_5538305373514744468[240] = 0;
   out_5538305373514744468[241] = 0;
   out_5538305373514744468[242] = 0;
   out_5538305373514744468[243] = 0;
   out_5538305373514744468[244] = 0;
   out_5538305373514744468[245] = 0;
   out_5538305373514744468[246] = 0;
   out_5538305373514744468[247] = 1;
   out_5538305373514744468[248] = 0;
   out_5538305373514744468[249] = 0;
   out_5538305373514744468[250] = 0;
   out_5538305373514744468[251] = 0;
   out_5538305373514744468[252] = 0;
   out_5538305373514744468[253] = 0;
   out_5538305373514744468[254] = 0;
   out_5538305373514744468[255] = 0;
   out_5538305373514744468[256] = 0;
   out_5538305373514744468[257] = 0;
   out_5538305373514744468[258] = 0;
   out_5538305373514744468[259] = 0;
   out_5538305373514744468[260] = 0;
   out_5538305373514744468[261] = 0;
   out_5538305373514744468[262] = 0;
   out_5538305373514744468[263] = 0;
   out_5538305373514744468[264] = 0;
   out_5538305373514744468[265] = 0;
   out_5538305373514744468[266] = 1;
   out_5538305373514744468[267] = 0;
   out_5538305373514744468[268] = 0;
   out_5538305373514744468[269] = 0;
   out_5538305373514744468[270] = 0;
   out_5538305373514744468[271] = 0;
   out_5538305373514744468[272] = 0;
   out_5538305373514744468[273] = 0;
   out_5538305373514744468[274] = 0;
   out_5538305373514744468[275] = 0;
   out_5538305373514744468[276] = 0;
   out_5538305373514744468[277] = 0;
   out_5538305373514744468[278] = 0;
   out_5538305373514744468[279] = 0;
   out_5538305373514744468[280] = 0;
   out_5538305373514744468[281] = 0;
   out_5538305373514744468[282] = 0;
   out_5538305373514744468[283] = 0;
   out_5538305373514744468[284] = 0;
   out_5538305373514744468[285] = 1;
   out_5538305373514744468[286] = 0;
   out_5538305373514744468[287] = 0;
   out_5538305373514744468[288] = 0;
   out_5538305373514744468[289] = 0;
   out_5538305373514744468[290] = 0;
   out_5538305373514744468[291] = 0;
   out_5538305373514744468[292] = 0;
   out_5538305373514744468[293] = 0;
   out_5538305373514744468[294] = 0;
   out_5538305373514744468[295] = 0;
   out_5538305373514744468[296] = 0;
   out_5538305373514744468[297] = 0;
   out_5538305373514744468[298] = 0;
   out_5538305373514744468[299] = 0;
   out_5538305373514744468[300] = 0;
   out_5538305373514744468[301] = 0;
   out_5538305373514744468[302] = 0;
   out_5538305373514744468[303] = 0;
   out_5538305373514744468[304] = 1;
   out_5538305373514744468[305] = 0;
   out_5538305373514744468[306] = 0;
   out_5538305373514744468[307] = 0;
   out_5538305373514744468[308] = 0;
   out_5538305373514744468[309] = 0;
   out_5538305373514744468[310] = 0;
   out_5538305373514744468[311] = 0;
   out_5538305373514744468[312] = 0;
   out_5538305373514744468[313] = 0;
   out_5538305373514744468[314] = 0;
   out_5538305373514744468[315] = 0;
   out_5538305373514744468[316] = 0;
   out_5538305373514744468[317] = 0;
   out_5538305373514744468[318] = 0;
   out_5538305373514744468[319] = 0;
   out_5538305373514744468[320] = 0;
   out_5538305373514744468[321] = 0;
   out_5538305373514744468[322] = 0;
   out_5538305373514744468[323] = 1;
}
void h_4(double *state, double *unused, double *out_1030168054688066876) {
   out_1030168054688066876[0] = state[6] + state[9];
   out_1030168054688066876[1] = state[7] + state[10];
   out_1030168054688066876[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8462847877961449813) {
   out_8462847877961449813[0] = 0;
   out_8462847877961449813[1] = 0;
   out_8462847877961449813[2] = 0;
   out_8462847877961449813[3] = 0;
   out_8462847877961449813[4] = 0;
   out_8462847877961449813[5] = 0;
   out_8462847877961449813[6] = 1;
   out_8462847877961449813[7] = 0;
   out_8462847877961449813[8] = 0;
   out_8462847877961449813[9] = 1;
   out_8462847877961449813[10] = 0;
   out_8462847877961449813[11] = 0;
   out_8462847877961449813[12] = 0;
   out_8462847877961449813[13] = 0;
   out_8462847877961449813[14] = 0;
   out_8462847877961449813[15] = 0;
   out_8462847877961449813[16] = 0;
   out_8462847877961449813[17] = 0;
   out_8462847877961449813[18] = 0;
   out_8462847877961449813[19] = 0;
   out_8462847877961449813[20] = 0;
   out_8462847877961449813[21] = 0;
   out_8462847877961449813[22] = 0;
   out_8462847877961449813[23] = 0;
   out_8462847877961449813[24] = 0;
   out_8462847877961449813[25] = 1;
   out_8462847877961449813[26] = 0;
   out_8462847877961449813[27] = 0;
   out_8462847877961449813[28] = 1;
   out_8462847877961449813[29] = 0;
   out_8462847877961449813[30] = 0;
   out_8462847877961449813[31] = 0;
   out_8462847877961449813[32] = 0;
   out_8462847877961449813[33] = 0;
   out_8462847877961449813[34] = 0;
   out_8462847877961449813[35] = 0;
   out_8462847877961449813[36] = 0;
   out_8462847877961449813[37] = 0;
   out_8462847877961449813[38] = 0;
   out_8462847877961449813[39] = 0;
   out_8462847877961449813[40] = 0;
   out_8462847877961449813[41] = 0;
   out_8462847877961449813[42] = 0;
   out_8462847877961449813[43] = 0;
   out_8462847877961449813[44] = 1;
   out_8462847877961449813[45] = 0;
   out_8462847877961449813[46] = 0;
   out_8462847877961449813[47] = 1;
   out_8462847877961449813[48] = 0;
   out_8462847877961449813[49] = 0;
   out_8462847877961449813[50] = 0;
   out_8462847877961449813[51] = 0;
   out_8462847877961449813[52] = 0;
   out_8462847877961449813[53] = 0;
}
void h_10(double *state, double *unused, double *out_1708780435725268150) {
   out_1708780435725268150[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1708780435725268150[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1708780435725268150[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_764250186211597429) {
   out_764250186211597429[0] = 0;
   out_764250186211597429[1] = 9.8100000000000005*cos(state[1]);
   out_764250186211597429[2] = 0;
   out_764250186211597429[3] = 0;
   out_764250186211597429[4] = -state[8];
   out_764250186211597429[5] = state[7];
   out_764250186211597429[6] = 0;
   out_764250186211597429[7] = state[5];
   out_764250186211597429[8] = -state[4];
   out_764250186211597429[9] = 0;
   out_764250186211597429[10] = 0;
   out_764250186211597429[11] = 0;
   out_764250186211597429[12] = 1;
   out_764250186211597429[13] = 0;
   out_764250186211597429[14] = 0;
   out_764250186211597429[15] = 1;
   out_764250186211597429[16] = 0;
   out_764250186211597429[17] = 0;
   out_764250186211597429[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_764250186211597429[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_764250186211597429[20] = 0;
   out_764250186211597429[21] = state[8];
   out_764250186211597429[22] = 0;
   out_764250186211597429[23] = -state[6];
   out_764250186211597429[24] = -state[5];
   out_764250186211597429[25] = 0;
   out_764250186211597429[26] = state[3];
   out_764250186211597429[27] = 0;
   out_764250186211597429[28] = 0;
   out_764250186211597429[29] = 0;
   out_764250186211597429[30] = 0;
   out_764250186211597429[31] = 1;
   out_764250186211597429[32] = 0;
   out_764250186211597429[33] = 0;
   out_764250186211597429[34] = 1;
   out_764250186211597429[35] = 0;
   out_764250186211597429[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_764250186211597429[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_764250186211597429[38] = 0;
   out_764250186211597429[39] = -state[7];
   out_764250186211597429[40] = state[6];
   out_764250186211597429[41] = 0;
   out_764250186211597429[42] = state[4];
   out_764250186211597429[43] = -state[3];
   out_764250186211597429[44] = 0;
   out_764250186211597429[45] = 0;
   out_764250186211597429[46] = 0;
   out_764250186211597429[47] = 0;
   out_764250186211597429[48] = 0;
   out_764250186211597429[49] = 0;
   out_764250186211597429[50] = 1;
   out_764250186211597429[51] = 0;
   out_764250186211597429[52] = 0;
   out_764250186211597429[53] = 1;
}
void h_13(double *state, double *unused, double *out_7179790802110218228) {
   out_7179790802110218228[0] = state[3];
   out_7179790802110218228[1] = state[4];
   out_7179790802110218228[2] = state[5];
}
void H_13(double *state, double *unused, double *out_852216669644748884) {
   out_852216669644748884[0] = 0;
   out_852216669644748884[1] = 0;
   out_852216669644748884[2] = 0;
   out_852216669644748884[3] = 1;
   out_852216669644748884[4] = 0;
   out_852216669644748884[5] = 0;
   out_852216669644748884[6] = 0;
   out_852216669644748884[7] = 0;
   out_852216669644748884[8] = 0;
   out_852216669644748884[9] = 0;
   out_852216669644748884[10] = 0;
   out_852216669644748884[11] = 0;
   out_852216669644748884[12] = 0;
   out_852216669644748884[13] = 0;
   out_852216669644748884[14] = 0;
   out_852216669644748884[15] = 0;
   out_852216669644748884[16] = 0;
   out_852216669644748884[17] = 0;
   out_852216669644748884[18] = 0;
   out_852216669644748884[19] = 0;
   out_852216669644748884[20] = 0;
   out_852216669644748884[21] = 0;
   out_852216669644748884[22] = 1;
   out_852216669644748884[23] = 0;
   out_852216669644748884[24] = 0;
   out_852216669644748884[25] = 0;
   out_852216669644748884[26] = 0;
   out_852216669644748884[27] = 0;
   out_852216669644748884[28] = 0;
   out_852216669644748884[29] = 0;
   out_852216669644748884[30] = 0;
   out_852216669644748884[31] = 0;
   out_852216669644748884[32] = 0;
   out_852216669644748884[33] = 0;
   out_852216669644748884[34] = 0;
   out_852216669644748884[35] = 0;
   out_852216669644748884[36] = 0;
   out_852216669644748884[37] = 0;
   out_852216669644748884[38] = 0;
   out_852216669644748884[39] = 0;
   out_852216669644748884[40] = 0;
   out_852216669644748884[41] = 1;
   out_852216669644748884[42] = 0;
   out_852216669644748884[43] = 0;
   out_852216669644748884[44] = 0;
   out_852216669644748884[45] = 0;
   out_852216669644748884[46] = 0;
   out_852216669644748884[47] = 0;
   out_852216669644748884[48] = 0;
   out_852216669644748884[49] = 0;
   out_852216669644748884[50] = 0;
   out_852216669644748884[51] = 0;
   out_852216669644748884[52] = 0;
   out_852216669644748884[53] = 0;
}
void h_14(double *state, double *unused, double *out_8927767218490339382) {
   out_8927767218490339382[0] = state[6];
   out_8927767218490339382[1] = state[7];
   out_8927767218490339382[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4499607021621965284) {
   out_4499607021621965284[0] = 0;
   out_4499607021621965284[1] = 0;
   out_4499607021621965284[2] = 0;
   out_4499607021621965284[3] = 0;
   out_4499607021621965284[4] = 0;
   out_4499607021621965284[5] = 0;
   out_4499607021621965284[6] = 1;
   out_4499607021621965284[7] = 0;
   out_4499607021621965284[8] = 0;
   out_4499607021621965284[9] = 0;
   out_4499607021621965284[10] = 0;
   out_4499607021621965284[11] = 0;
   out_4499607021621965284[12] = 0;
   out_4499607021621965284[13] = 0;
   out_4499607021621965284[14] = 0;
   out_4499607021621965284[15] = 0;
   out_4499607021621965284[16] = 0;
   out_4499607021621965284[17] = 0;
   out_4499607021621965284[18] = 0;
   out_4499607021621965284[19] = 0;
   out_4499607021621965284[20] = 0;
   out_4499607021621965284[21] = 0;
   out_4499607021621965284[22] = 0;
   out_4499607021621965284[23] = 0;
   out_4499607021621965284[24] = 0;
   out_4499607021621965284[25] = 1;
   out_4499607021621965284[26] = 0;
   out_4499607021621965284[27] = 0;
   out_4499607021621965284[28] = 0;
   out_4499607021621965284[29] = 0;
   out_4499607021621965284[30] = 0;
   out_4499607021621965284[31] = 0;
   out_4499607021621965284[32] = 0;
   out_4499607021621965284[33] = 0;
   out_4499607021621965284[34] = 0;
   out_4499607021621965284[35] = 0;
   out_4499607021621965284[36] = 0;
   out_4499607021621965284[37] = 0;
   out_4499607021621965284[38] = 0;
   out_4499607021621965284[39] = 0;
   out_4499607021621965284[40] = 0;
   out_4499607021621965284[41] = 0;
   out_4499607021621965284[42] = 0;
   out_4499607021621965284[43] = 0;
   out_4499607021621965284[44] = 1;
   out_4499607021621965284[45] = 0;
   out_4499607021621965284[46] = 0;
   out_4499607021621965284[47] = 0;
   out_4499607021621965284[48] = 0;
   out_4499607021621965284[49] = 0;
   out_4499607021621965284[50] = 0;
   out_4499607021621965284[51] = 0;
   out_4499607021621965284[52] = 0;
   out_4499607021621965284[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2128814047154352200) {
  err_fun(nom_x, delta_x, out_2128814047154352200);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7001406389145399648) {
  inv_err_fun(nom_x, true_x, out_7001406389145399648);
}
void pose_H_mod_fun(double *state, double *out_1783591311774877036) {
  H_mod_fun(state, out_1783591311774877036);
}
void pose_f_fun(double *state, double dt, double *out_3415300158525823285) {
  f_fun(state,  dt, out_3415300158525823285);
}
void pose_F_fun(double *state, double dt, double *out_5538305373514744468) {
  F_fun(state,  dt, out_5538305373514744468);
}
void pose_h_4(double *state, double *unused, double *out_1030168054688066876) {
  h_4(state, unused, out_1030168054688066876);
}
void pose_H_4(double *state, double *unused, double *out_8462847877961449813) {
  H_4(state, unused, out_8462847877961449813);
}
void pose_h_10(double *state, double *unused, double *out_1708780435725268150) {
  h_10(state, unused, out_1708780435725268150);
}
void pose_H_10(double *state, double *unused, double *out_764250186211597429) {
  H_10(state, unused, out_764250186211597429);
}
void pose_h_13(double *state, double *unused, double *out_7179790802110218228) {
  h_13(state, unused, out_7179790802110218228);
}
void pose_H_13(double *state, double *unused, double *out_852216669644748884) {
  H_13(state, unused, out_852216669644748884);
}
void pose_h_14(double *state, double *unused, double *out_8927767218490339382) {
  h_14(state, unused, out_8927767218490339382);
}
void pose_H_14(double *state, double *unused, double *out_4499607021621965284) {
  H_14(state, unused, out_4499607021621965284);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
