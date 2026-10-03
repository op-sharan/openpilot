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
void err_fun(double *nom_x, double *delta_x, double *out_2114100039121924448) {
   out_2114100039121924448[0] = delta_x[0] + nom_x[0];
   out_2114100039121924448[1] = delta_x[1] + nom_x[1];
   out_2114100039121924448[2] = delta_x[2] + nom_x[2];
   out_2114100039121924448[3] = delta_x[3] + nom_x[3];
   out_2114100039121924448[4] = delta_x[4] + nom_x[4];
   out_2114100039121924448[5] = delta_x[5] + nom_x[5];
   out_2114100039121924448[6] = delta_x[6] + nom_x[6];
   out_2114100039121924448[7] = delta_x[7] + nom_x[7];
   out_2114100039121924448[8] = delta_x[8] + nom_x[8];
   out_2114100039121924448[9] = delta_x[9] + nom_x[9];
   out_2114100039121924448[10] = delta_x[10] + nom_x[10];
   out_2114100039121924448[11] = delta_x[11] + nom_x[11];
   out_2114100039121924448[12] = delta_x[12] + nom_x[12];
   out_2114100039121924448[13] = delta_x[13] + nom_x[13];
   out_2114100039121924448[14] = delta_x[14] + nom_x[14];
   out_2114100039121924448[15] = delta_x[15] + nom_x[15];
   out_2114100039121924448[16] = delta_x[16] + nom_x[16];
   out_2114100039121924448[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2618631132718776507) {
   out_2618631132718776507[0] = -nom_x[0] + true_x[0];
   out_2618631132718776507[1] = -nom_x[1] + true_x[1];
   out_2618631132718776507[2] = -nom_x[2] + true_x[2];
   out_2618631132718776507[3] = -nom_x[3] + true_x[3];
   out_2618631132718776507[4] = -nom_x[4] + true_x[4];
   out_2618631132718776507[5] = -nom_x[5] + true_x[5];
   out_2618631132718776507[6] = -nom_x[6] + true_x[6];
   out_2618631132718776507[7] = -nom_x[7] + true_x[7];
   out_2618631132718776507[8] = -nom_x[8] + true_x[8];
   out_2618631132718776507[9] = -nom_x[9] + true_x[9];
   out_2618631132718776507[10] = -nom_x[10] + true_x[10];
   out_2618631132718776507[11] = -nom_x[11] + true_x[11];
   out_2618631132718776507[12] = -nom_x[12] + true_x[12];
   out_2618631132718776507[13] = -nom_x[13] + true_x[13];
   out_2618631132718776507[14] = -nom_x[14] + true_x[14];
   out_2618631132718776507[15] = -nom_x[15] + true_x[15];
   out_2618631132718776507[16] = -nom_x[16] + true_x[16];
   out_2618631132718776507[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4058554015661107700) {
   out_4058554015661107700[0] = 1.0;
   out_4058554015661107700[1] = 0.0;
   out_4058554015661107700[2] = 0.0;
   out_4058554015661107700[3] = 0.0;
   out_4058554015661107700[4] = 0.0;
   out_4058554015661107700[5] = 0.0;
   out_4058554015661107700[6] = 0.0;
   out_4058554015661107700[7] = 0.0;
   out_4058554015661107700[8] = 0.0;
   out_4058554015661107700[9] = 0.0;
   out_4058554015661107700[10] = 0.0;
   out_4058554015661107700[11] = 0.0;
   out_4058554015661107700[12] = 0.0;
   out_4058554015661107700[13] = 0.0;
   out_4058554015661107700[14] = 0.0;
   out_4058554015661107700[15] = 0.0;
   out_4058554015661107700[16] = 0.0;
   out_4058554015661107700[17] = 0.0;
   out_4058554015661107700[18] = 0.0;
   out_4058554015661107700[19] = 1.0;
   out_4058554015661107700[20] = 0.0;
   out_4058554015661107700[21] = 0.0;
   out_4058554015661107700[22] = 0.0;
   out_4058554015661107700[23] = 0.0;
   out_4058554015661107700[24] = 0.0;
   out_4058554015661107700[25] = 0.0;
   out_4058554015661107700[26] = 0.0;
   out_4058554015661107700[27] = 0.0;
   out_4058554015661107700[28] = 0.0;
   out_4058554015661107700[29] = 0.0;
   out_4058554015661107700[30] = 0.0;
   out_4058554015661107700[31] = 0.0;
   out_4058554015661107700[32] = 0.0;
   out_4058554015661107700[33] = 0.0;
   out_4058554015661107700[34] = 0.0;
   out_4058554015661107700[35] = 0.0;
   out_4058554015661107700[36] = 0.0;
   out_4058554015661107700[37] = 0.0;
   out_4058554015661107700[38] = 1.0;
   out_4058554015661107700[39] = 0.0;
   out_4058554015661107700[40] = 0.0;
   out_4058554015661107700[41] = 0.0;
   out_4058554015661107700[42] = 0.0;
   out_4058554015661107700[43] = 0.0;
   out_4058554015661107700[44] = 0.0;
   out_4058554015661107700[45] = 0.0;
   out_4058554015661107700[46] = 0.0;
   out_4058554015661107700[47] = 0.0;
   out_4058554015661107700[48] = 0.0;
   out_4058554015661107700[49] = 0.0;
   out_4058554015661107700[50] = 0.0;
   out_4058554015661107700[51] = 0.0;
   out_4058554015661107700[52] = 0.0;
   out_4058554015661107700[53] = 0.0;
   out_4058554015661107700[54] = 0.0;
   out_4058554015661107700[55] = 0.0;
   out_4058554015661107700[56] = 0.0;
   out_4058554015661107700[57] = 1.0;
   out_4058554015661107700[58] = 0.0;
   out_4058554015661107700[59] = 0.0;
   out_4058554015661107700[60] = 0.0;
   out_4058554015661107700[61] = 0.0;
   out_4058554015661107700[62] = 0.0;
   out_4058554015661107700[63] = 0.0;
   out_4058554015661107700[64] = 0.0;
   out_4058554015661107700[65] = 0.0;
   out_4058554015661107700[66] = 0.0;
   out_4058554015661107700[67] = 0.0;
   out_4058554015661107700[68] = 0.0;
   out_4058554015661107700[69] = 0.0;
   out_4058554015661107700[70] = 0.0;
   out_4058554015661107700[71] = 0.0;
   out_4058554015661107700[72] = 0.0;
   out_4058554015661107700[73] = 0.0;
   out_4058554015661107700[74] = 0.0;
   out_4058554015661107700[75] = 0.0;
   out_4058554015661107700[76] = 1.0;
   out_4058554015661107700[77] = 0.0;
   out_4058554015661107700[78] = 0.0;
   out_4058554015661107700[79] = 0.0;
   out_4058554015661107700[80] = 0.0;
   out_4058554015661107700[81] = 0.0;
   out_4058554015661107700[82] = 0.0;
   out_4058554015661107700[83] = 0.0;
   out_4058554015661107700[84] = 0.0;
   out_4058554015661107700[85] = 0.0;
   out_4058554015661107700[86] = 0.0;
   out_4058554015661107700[87] = 0.0;
   out_4058554015661107700[88] = 0.0;
   out_4058554015661107700[89] = 0.0;
   out_4058554015661107700[90] = 0.0;
   out_4058554015661107700[91] = 0.0;
   out_4058554015661107700[92] = 0.0;
   out_4058554015661107700[93] = 0.0;
   out_4058554015661107700[94] = 0.0;
   out_4058554015661107700[95] = 1.0;
   out_4058554015661107700[96] = 0.0;
   out_4058554015661107700[97] = 0.0;
   out_4058554015661107700[98] = 0.0;
   out_4058554015661107700[99] = 0.0;
   out_4058554015661107700[100] = 0.0;
   out_4058554015661107700[101] = 0.0;
   out_4058554015661107700[102] = 0.0;
   out_4058554015661107700[103] = 0.0;
   out_4058554015661107700[104] = 0.0;
   out_4058554015661107700[105] = 0.0;
   out_4058554015661107700[106] = 0.0;
   out_4058554015661107700[107] = 0.0;
   out_4058554015661107700[108] = 0.0;
   out_4058554015661107700[109] = 0.0;
   out_4058554015661107700[110] = 0.0;
   out_4058554015661107700[111] = 0.0;
   out_4058554015661107700[112] = 0.0;
   out_4058554015661107700[113] = 0.0;
   out_4058554015661107700[114] = 1.0;
   out_4058554015661107700[115] = 0.0;
   out_4058554015661107700[116] = 0.0;
   out_4058554015661107700[117] = 0.0;
   out_4058554015661107700[118] = 0.0;
   out_4058554015661107700[119] = 0.0;
   out_4058554015661107700[120] = 0.0;
   out_4058554015661107700[121] = 0.0;
   out_4058554015661107700[122] = 0.0;
   out_4058554015661107700[123] = 0.0;
   out_4058554015661107700[124] = 0.0;
   out_4058554015661107700[125] = 0.0;
   out_4058554015661107700[126] = 0.0;
   out_4058554015661107700[127] = 0.0;
   out_4058554015661107700[128] = 0.0;
   out_4058554015661107700[129] = 0.0;
   out_4058554015661107700[130] = 0.0;
   out_4058554015661107700[131] = 0.0;
   out_4058554015661107700[132] = 0.0;
   out_4058554015661107700[133] = 1.0;
   out_4058554015661107700[134] = 0.0;
   out_4058554015661107700[135] = 0.0;
   out_4058554015661107700[136] = 0.0;
   out_4058554015661107700[137] = 0.0;
   out_4058554015661107700[138] = 0.0;
   out_4058554015661107700[139] = 0.0;
   out_4058554015661107700[140] = 0.0;
   out_4058554015661107700[141] = 0.0;
   out_4058554015661107700[142] = 0.0;
   out_4058554015661107700[143] = 0.0;
   out_4058554015661107700[144] = 0.0;
   out_4058554015661107700[145] = 0.0;
   out_4058554015661107700[146] = 0.0;
   out_4058554015661107700[147] = 0.0;
   out_4058554015661107700[148] = 0.0;
   out_4058554015661107700[149] = 0.0;
   out_4058554015661107700[150] = 0.0;
   out_4058554015661107700[151] = 0.0;
   out_4058554015661107700[152] = 1.0;
   out_4058554015661107700[153] = 0.0;
   out_4058554015661107700[154] = 0.0;
   out_4058554015661107700[155] = 0.0;
   out_4058554015661107700[156] = 0.0;
   out_4058554015661107700[157] = 0.0;
   out_4058554015661107700[158] = 0.0;
   out_4058554015661107700[159] = 0.0;
   out_4058554015661107700[160] = 0.0;
   out_4058554015661107700[161] = 0.0;
   out_4058554015661107700[162] = 0.0;
   out_4058554015661107700[163] = 0.0;
   out_4058554015661107700[164] = 0.0;
   out_4058554015661107700[165] = 0.0;
   out_4058554015661107700[166] = 0.0;
   out_4058554015661107700[167] = 0.0;
   out_4058554015661107700[168] = 0.0;
   out_4058554015661107700[169] = 0.0;
   out_4058554015661107700[170] = 0.0;
   out_4058554015661107700[171] = 1.0;
   out_4058554015661107700[172] = 0.0;
   out_4058554015661107700[173] = 0.0;
   out_4058554015661107700[174] = 0.0;
   out_4058554015661107700[175] = 0.0;
   out_4058554015661107700[176] = 0.0;
   out_4058554015661107700[177] = 0.0;
   out_4058554015661107700[178] = 0.0;
   out_4058554015661107700[179] = 0.0;
   out_4058554015661107700[180] = 0.0;
   out_4058554015661107700[181] = 0.0;
   out_4058554015661107700[182] = 0.0;
   out_4058554015661107700[183] = 0.0;
   out_4058554015661107700[184] = 0.0;
   out_4058554015661107700[185] = 0.0;
   out_4058554015661107700[186] = 0.0;
   out_4058554015661107700[187] = 0.0;
   out_4058554015661107700[188] = 0.0;
   out_4058554015661107700[189] = 0.0;
   out_4058554015661107700[190] = 1.0;
   out_4058554015661107700[191] = 0.0;
   out_4058554015661107700[192] = 0.0;
   out_4058554015661107700[193] = 0.0;
   out_4058554015661107700[194] = 0.0;
   out_4058554015661107700[195] = 0.0;
   out_4058554015661107700[196] = 0.0;
   out_4058554015661107700[197] = 0.0;
   out_4058554015661107700[198] = 0.0;
   out_4058554015661107700[199] = 0.0;
   out_4058554015661107700[200] = 0.0;
   out_4058554015661107700[201] = 0.0;
   out_4058554015661107700[202] = 0.0;
   out_4058554015661107700[203] = 0.0;
   out_4058554015661107700[204] = 0.0;
   out_4058554015661107700[205] = 0.0;
   out_4058554015661107700[206] = 0.0;
   out_4058554015661107700[207] = 0.0;
   out_4058554015661107700[208] = 0.0;
   out_4058554015661107700[209] = 1.0;
   out_4058554015661107700[210] = 0.0;
   out_4058554015661107700[211] = 0.0;
   out_4058554015661107700[212] = 0.0;
   out_4058554015661107700[213] = 0.0;
   out_4058554015661107700[214] = 0.0;
   out_4058554015661107700[215] = 0.0;
   out_4058554015661107700[216] = 0.0;
   out_4058554015661107700[217] = 0.0;
   out_4058554015661107700[218] = 0.0;
   out_4058554015661107700[219] = 0.0;
   out_4058554015661107700[220] = 0.0;
   out_4058554015661107700[221] = 0.0;
   out_4058554015661107700[222] = 0.0;
   out_4058554015661107700[223] = 0.0;
   out_4058554015661107700[224] = 0.0;
   out_4058554015661107700[225] = 0.0;
   out_4058554015661107700[226] = 0.0;
   out_4058554015661107700[227] = 0.0;
   out_4058554015661107700[228] = 1.0;
   out_4058554015661107700[229] = 0.0;
   out_4058554015661107700[230] = 0.0;
   out_4058554015661107700[231] = 0.0;
   out_4058554015661107700[232] = 0.0;
   out_4058554015661107700[233] = 0.0;
   out_4058554015661107700[234] = 0.0;
   out_4058554015661107700[235] = 0.0;
   out_4058554015661107700[236] = 0.0;
   out_4058554015661107700[237] = 0.0;
   out_4058554015661107700[238] = 0.0;
   out_4058554015661107700[239] = 0.0;
   out_4058554015661107700[240] = 0.0;
   out_4058554015661107700[241] = 0.0;
   out_4058554015661107700[242] = 0.0;
   out_4058554015661107700[243] = 0.0;
   out_4058554015661107700[244] = 0.0;
   out_4058554015661107700[245] = 0.0;
   out_4058554015661107700[246] = 0.0;
   out_4058554015661107700[247] = 1.0;
   out_4058554015661107700[248] = 0.0;
   out_4058554015661107700[249] = 0.0;
   out_4058554015661107700[250] = 0.0;
   out_4058554015661107700[251] = 0.0;
   out_4058554015661107700[252] = 0.0;
   out_4058554015661107700[253] = 0.0;
   out_4058554015661107700[254] = 0.0;
   out_4058554015661107700[255] = 0.0;
   out_4058554015661107700[256] = 0.0;
   out_4058554015661107700[257] = 0.0;
   out_4058554015661107700[258] = 0.0;
   out_4058554015661107700[259] = 0.0;
   out_4058554015661107700[260] = 0.0;
   out_4058554015661107700[261] = 0.0;
   out_4058554015661107700[262] = 0.0;
   out_4058554015661107700[263] = 0.0;
   out_4058554015661107700[264] = 0.0;
   out_4058554015661107700[265] = 0.0;
   out_4058554015661107700[266] = 1.0;
   out_4058554015661107700[267] = 0.0;
   out_4058554015661107700[268] = 0.0;
   out_4058554015661107700[269] = 0.0;
   out_4058554015661107700[270] = 0.0;
   out_4058554015661107700[271] = 0.0;
   out_4058554015661107700[272] = 0.0;
   out_4058554015661107700[273] = 0.0;
   out_4058554015661107700[274] = 0.0;
   out_4058554015661107700[275] = 0.0;
   out_4058554015661107700[276] = 0.0;
   out_4058554015661107700[277] = 0.0;
   out_4058554015661107700[278] = 0.0;
   out_4058554015661107700[279] = 0.0;
   out_4058554015661107700[280] = 0.0;
   out_4058554015661107700[281] = 0.0;
   out_4058554015661107700[282] = 0.0;
   out_4058554015661107700[283] = 0.0;
   out_4058554015661107700[284] = 0.0;
   out_4058554015661107700[285] = 1.0;
   out_4058554015661107700[286] = 0.0;
   out_4058554015661107700[287] = 0.0;
   out_4058554015661107700[288] = 0.0;
   out_4058554015661107700[289] = 0.0;
   out_4058554015661107700[290] = 0.0;
   out_4058554015661107700[291] = 0.0;
   out_4058554015661107700[292] = 0.0;
   out_4058554015661107700[293] = 0.0;
   out_4058554015661107700[294] = 0.0;
   out_4058554015661107700[295] = 0.0;
   out_4058554015661107700[296] = 0.0;
   out_4058554015661107700[297] = 0.0;
   out_4058554015661107700[298] = 0.0;
   out_4058554015661107700[299] = 0.0;
   out_4058554015661107700[300] = 0.0;
   out_4058554015661107700[301] = 0.0;
   out_4058554015661107700[302] = 0.0;
   out_4058554015661107700[303] = 0.0;
   out_4058554015661107700[304] = 1.0;
   out_4058554015661107700[305] = 0.0;
   out_4058554015661107700[306] = 0.0;
   out_4058554015661107700[307] = 0.0;
   out_4058554015661107700[308] = 0.0;
   out_4058554015661107700[309] = 0.0;
   out_4058554015661107700[310] = 0.0;
   out_4058554015661107700[311] = 0.0;
   out_4058554015661107700[312] = 0.0;
   out_4058554015661107700[313] = 0.0;
   out_4058554015661107700[314] = 0.0;
   out_4058554015661107700[315] = 0.0;
   out_4058554015661107700[316] = 0.0;
   out_4058554015661107700[317] = 0.0;
   out_4058554015661107700[318] = 0.0;
   out_4058554015661107700[319] = 0.0;
   out_4058554015661107700[320] = 0.0;
   out_4058554015661107700[321] = 0.0;
   out_4058554015661107700[322] = 0.0;
   out_4058554015661107700[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5119116025028968719) {
   out_5119116025028968719[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5119116025028968719[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5119116025028968719[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5119116025028968719[3] = dt*state[12] + state[3];
   out_5119116025028968719[4] = dt*state[13] + state[4];
   out_5119116025028968719[5] = dt*state[14] + state[5];
   out_5119116025028968719[6] = state[6];
   out_5119116025028968719[7] = state[7];
   out_5119116025028968719[8] = state[8];
   out_5119116025028968719[9] = state[9];
   out_5119116025028968719[10] = state[10];
   out_5119116025028968719[11] = state[11];
   out_5119116025028968719[12] = state[12];
   out_5119116025028968719[13] = state[13];
   out_5119116025028968719[14] = state[14];
   out_5119116025028968719[15] = state[15];
   out_5119116025028968719[16] = state[16];
   out_5119116025028968719[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6155576186277025652) {
   out_6155576186277025652[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6155576186277025652[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6155576186277025652[2] = 0;
   out_6155576186277025652[3] = 0;
   out_6155576186277025652[4] = 0;
   out_6155576186277025652[5] = 0;
   out_6155576186277025652[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6155576186277025652[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6155576186277025652[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6155576186277025652[9] = 0;
   out_6155576186277025652[10] = 0;
   out_6155576186277025652[11] = 0;
   out_6155576186277025652[12] = 0;
   out_6155576186277025652[13] = 0;
   out_6155576186277025652[14] = 0;
   out_6155576186277025652[15] = 0;
   out_6155576186277025652[16] = 0;
   out_6155576186277025652[17] = 0;
   out_6155576186277025652[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6155576186277025652[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6155576186277025652[20] = 0;
   out_6155576186277025652[21] = 0;
   out_6155576186277025652[22] = 0;
   out_6155576186277025652[23] = 0;
   out_6155576186277025652[24] = 0;
   out_6155576186277025652[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6155576186277025652[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6155576186277025652[27] = 0;
   out_6155576186277025652[28] = 0;
   out_6155576186277025652[29] = 0;
   out_6155576186277025652[30] = 0;
   out_6155576186277025652[31] = 0;
   out_6155576186277025652[32] = 0;
   out_6155576186277025652[33] = 0;
   out_6155576186277025652[34] = 0;
   out_6155576186277025652[35] = 0;
   out_6155576186277025652[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6155576186277025652[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6155576186277025652[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6155576186277025652[39] = 0;
   out_6155576186277025652[40] = 0;
   out_6155576186277025652[41] = 0;
   out_6155576186277025652[42] = 0;
   out_6155576186277025652[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6155576186277025652[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6155576186277025652[45] = 0;
   out_6155576186277025652[46] = 0;
   out_6155576186277025652[47] = 0;
   out_6155576186277025652[48] = 0;
   out_6155576186277025652[49] = 0;
   out_6155576186277025652[50] = 0;
   out_6155576186277025652[51] = 0;
   out_6155576186277025652[52] = 0;
   out_6155576186277025652[53] = 0;
   out_6155576186277025652[54] = 0;
   out_6155576186277025652[55] = 0;
   out_6155576186277025652[56] = 0;
   out_6155576186277025652[57] = 1;
   out_6155576186277025652[58] = 0;
   out_6155576186277025652[59] = 0;
   out_6155576186277025652[60] = 0;
   out_6155576186277025652[61] = 0;
   out_6155576186277025652[62] = 0;
   out_6155576186277025652[63] = 0;
   out_6155576186277025652[64] = 0;
   out_6155576186277025652[65] = 0;
   out_6155576186277025652[66] = dt;
   out_6155576186277025652[67] = 0;
   out_6155576186277025652[68] = 0;
   out_6155576186277025652[69] = 0;
   out_6155576186277025652[70] = 0;
   out_6155576186277025652[71] = 0;
   out_6155576186277025652[72] = 0;
   out_6155576186277025652[73] = 0;
   out_6155576186277025652[74] = 0;
   out_6155576186277025652[75] = 0;
   out_6155576186277025652[76] = 1;
   out_6155576186277025652[77] = 0;
   out_6155576186277025652[78] = 0;
   out_6155576186277025652[79] = 0;
   out_6155576186277025652[80] = 0;
   out_6155576186277025652[81] = 0;
   out_6155576186277025652[82] = 0;
   out_6155576186277025652[83] = 0;
   out_6155576186277025652[84] = 0;
   out_6155576186277025652[85] = dt;
   out_6155576186277025652[86] = 0;
   out_6155576186277025652[87] = 0;
   out_6155576186277025652[88] = 0;
   out_6155576186277025652[89] = 0;
   out_6155576186277025652[90] = 0;
   out_6155576186277025652[91] = 0;
   out_6155576186277025652[92] = 0;
   out_6155576186277025652[93] = 0;
   out_6155576186277025652[94] = 0;
   out_6155576186277025652[95] = 1;
   out_6155576186277025652[96] = 0;
   out_6155576186277025652[97] = 0;
   out_6155576186277025652[98] = 0;
   out_6155576186277025652[99] = 0;
   out_6155576186277025652[100] = 0;
   out_6155576186277025652[101] = 0;
   out_6155576186277025652[102] = 0;
   out_6155576186277025652[103] = 0;
   out_6155576186277025652[104] = dt;
   out_6155576186277025652[105] = 0;
   out_6155576186277025652[106] = 0;
   out_6155576186277025652[107] = 0;
   out_6155576186277025652[108] = 0;
   out_6155576186277025652[109] = 0;
   out_6155576186277025652[110] = 0;
   out_6155576186277025652[111] = 0;
   out_6155576186277025652[112] = 0;
   out_6155576186277025652[113] = 0;
   out_6155576186277025652[114] = 1;
   out_6155576186277025652[115] = 0;
   out_6155576186277025652[116] = 0;
   out_6155576186277025652[117] = 0;
   out_6155576186277025652[118] = 0;
   out_6155576186277025652[119] = 0;
   out_6155576186277025652[120] = 0;
   out_6155576186277025652[121] = 0;
   out_6155576186277025652[122] = 0;
   out_6155576186277025652[123] = 0;
   out_6155576186277025652[124] = 0;
   out_6155576186277025652[125] = 0;
   out_6155576186277025652[126] = 0;
   out_6155576186277025652[127] = 0;
   out_6155576186277025652[128] = 0;
   out_6155576186277025652[129] = 0;
   out_6155576186277025652[130] = 0;
   out_6155576186277025652[131] = 0;
   out_6155576186277025652[132] = 0;
   out_6155576186277025652[133] = 1;
   out_6155576186277025652[134] = 0;
   out_6155576186277025652[135] = 0;
   out_6155576186277025652[136] = 0;
   out_6155576186277025652[137] = 0;
   out_6155576186277025652[138] = 0;
   out_6155576186277025652[139] = 0;
   out_6155576186277025652[140] = 0;
   out_6155576186277025652[141] = 0;
   out_6155576186277025652[142] = 0;
   out_6155576186277025652[143] = 0;
   out_6155576186277025652[144] = 0;
   out_6155576186277025652[145] = 0;
   out_6155576186277025652[146] = 0;
   out_6155576186277025652[147] = 0;
   out_6155576186277025652[148] = 0;
   out_6155576186277025652[149] = 0;
   out_6155576186277025652[150] = 0;
   out_6155576186277025652[151] = 0;
   out_6155576186277025652[152] = 1;
   out_6155576186277025652[153] = 0;
   out_6155576186277025652[154] = 0;
   out_6155576186277025652[155] = 0;
   out_6155576186277025652[156] = 0;
   out_6155576186277025652[157] = 0;
   out_6155576186277025652[158] = 0;
   out_6155576186277025652[159] = 0;
   out_6155576186277025652[160] = 0;
   out_6155576186277025652[161] = 0;
   out_6155576186277025652[162] = 0;
   out_6155576186277025652[163] = 0;
   out_6155576186277025652[164] = 0;
   out_6155576186277025652[165] = 0;
   out_6155576186277025652[166] = 0;
   out_6155576186277025652[167] = 0;
   out_6155576186277025652[168] = 0;
   out_6155576186277025652[169] = 0;
   out_6155576186277025652[170] = 0;
   out_6155576186277025652[171] = 1;
   out_6155576186277025652[172] = 0;
   out_6155576186277025652[173] = 0;
   out_6155576186277025652[174] = 0;
   out_6155576186277025652[175] = 0;
   out_6155576186277025652[176] = 0;
   out_6155576186277025652[177] = 0;
   out_6155576186277025652[178] = 0;
   out_6155576186277025652[179] = 0;
   out_6155576186277025652[180] = 0;
   out_6155576186277025652[181] = 0;
   out_6155576186277025652[182] = 0;
   out_6155576186277025652[183] = 0;
   out_6155576186277025652[184] = 0;
   out_6155576186277025652[185] = 0;
   out_6155576186277025652[186] = 0;
   out_6155576186277025652[187] = 0;
   out_6155576186277025652[188] = 0;
   out_6155576186277025652[189] = 0;
   out_6155576186277025652[190] = 1;
   out_6155576186277025652[191] = 0;
   out_6155576186277025652[192] = 0;
   out_6155576186277025652[193] = 0;
   out_6155576186277025652[194] = 0;
   out_6155576186277025652[195] = 0;
   out_6155576186277025652[196] = 0;
   out_6155576186277025652[197] = 0;
   out_6155576186277025652[198] = 0;
   out_6155576186277025652[199] = 0;
   out_6155576186277025652[200] = 0;
   out_6155576186277025652[201] = 0;
   out_6155576186277025652[202] = 0;
   out_6155576186277025652[203] = 0;
   out_6155576186277025652[204] = 0;
   out_6155576186277025652[205] = 0;
   out_6155576186277025652[206] = 0;
   out_6155576186277025652[207] = 0;
   out_6155576186277025652[208] = 0;
   out_6155576186277025652[209] = 1;
   out_6155576186277025652[210] = 0;
   out_6155576186277025652[211] = 0;
   out_6155576186277025652[212] = 0;
   out_6155576186277025652[213] = 0;
   out_6155576186277025652[214] = 0;
   out_6155576186277025652[215] = 0;
   out_6155576186277025652[216] = 0;
   out_6155576186277025652[217] = 0;
   out_6155576186277025652[218] = 0;
   out_6155576186277025652[219] = 0;
   out_6155576186277025652[220] = 0;
   out_6155576186277025652[221] = 0;
   out_6155576186277025652[222] = 0;
   out_6155576186277025652[223] = 0;
   out_6155576186277025652[224] = 0;
   out_6155576186277025652[225] = 0;
   out_6155576186277025652[226] = 0;
   out_6155576186277025652[227] = 0;
   out_6155576186277025652[228] = 1;
   out_6155576186277025652[229] = 0;
   out_6155576186277025652[230] = 0;
   out_6155576186277025652[231] = 0;
   out_6155576186277025652[232] = 0;
   out_6155576186277025652[233] = 0;
   out_6155576186277025652[234] = 0;
   out_6155576186277025652[235] = 0;
   out_6155576186277025652[236] = 0;
   out_6155576186277025652[237] = 0;
   out_6155576186277025652[238] = 0;
   out_6155576186277025652[239] = 0;
   out_6155576186277025652[240] = 0;
   out_6155576186277025652[241] = 0;
   out_6155576186277025652[242] = 0;
   out_6155576186277025652[243] = 0;
   out_6155576186277025652[244] = 0;
   out_6155576186277025652[245] = 0;
   out_6155576186277025652[246] = 0;
   out_6155576186277025652[247] = 1;
   out_6155576186277025652[248] = 0;
   out_6155576186277025652[249] = 0;
   out_6155576186277025652[250] = 0;
   out_6155576186277025652[251] = 0;
   out_6155576186277025652[252] = 0;
   out_6155576186277025652[253] = 0;
   out_6155576186277025652[254] = 0;
   out_6155576186277025652[255] = 0;
   out_6155576186277025652[256] = 0;
   out_6155576186277025652[257] = 0;
   out_6155576186277025652[258] = 0;
   out_6155576186277025652[259] = 0;
   out_6155576186277025652[260] = 0;
   out_6155576186277025652[261] = 0;
   out_6155576186277025652[262] = 0;
   out_6155576186277025652[263] = 0;
   out_6155576186277025652[264] = 0;
   out_6155576186277025652[265] = 0;
   out_6155576186277025652[266] = 1;
   out_6155576186277025652[267] = 0;
   out_6155576186277025652[268] = 0;
   out_6155576186277025652[269] = 0;
   out_6155576186277025652[270] = 0;
   out_6155576186277025652[271] = 0;
   out_6155576186277025652[272] = 0;
   out_6155576186277025652[273] = 0;
   out_6155576186277025652[274] = 0;
   out_6155576186277025652[275] = 0;
   out_6155576186277025652[276] = 0;
   out_6155576186277025652[277] = 0;
   out_6155576186277025652[278] = 0;
   out_6155576186277025652[279] = 0;
   out_6155576186277025652[280] = 0;
   out_6155576186277025652[281] = 0;
   out_6155576186277025652[282] = 0;
   out_6155576186277025652[283] = 0;
   out_6155576186277025652[284] = 0;
   out_6155576186277025652[285] = 1;
   out_6155576186277025652[286] = 0;
   out_6155576186277025652[287] = 0;
   out_6155576186277025652[288] = 0;
   out_6155576186277025652[289] = 0;
   out_6155576186277025652[290] = 0;
   out_6155576186277025652[291] = 0;
   out_6155576186277025652[292] = 0;
   out_6155576186277025652[293] = 0;
   out_6155576186277025652[294] = 0;
   out_6155576186277025652[295] = 0;
   out_6155576186277025652[296] = 0;
   out_6155576186277025652[297] = 0;
   out_6155576186277025652[298] = 0;
   out_6155576186277025652[299] = 0;
   out_6155576186277025652[300] = 0;
   out_6155576186277025652[301] = 0;
   out_6155576186277025652[302] = 0;
   out_6155576186277025652[303] = 0;
   out_6155576186277025652[304] = 1;
   out_6155576186277025652[305] = 0;
   out_6155576186277025652[306] = 0;
   out_6155576186277025652[307] = 0;
   out_6155576186277025652[308] = 0;
   out_6155576186277025652[309] = 0;
   out_6155576186277025652[310] = 0;
   out_6155576186277025652[311] = 0;
   out_6155576186277025652[312] = 0;
   out_6155576186277025652[313] = 0;
   out_6155576186277025652[314] = 0;
   out_6155576186277025652[315] = 0;
   out_6155576186277025652[316] = 0;
   out_6155576186277025652[317] = 0;
   out_6155576186277025652[318] = 0;
   out_6155576186277025652[319] = 0;
   out_6155576186277025652[320] = 0;
   out_6155576186277025652[321] = 0;
   out_6155576186277025652[322] = 0;
   out_6155576186277025652[323] = 1;
}
void h_4(double *state, double *unused, double *out_8035983586487788014) {
   out_8035983586487788014[0] = state[6] + state[9];
   out_8035983586487788014[1] = state[7] + state[10];
   out_8035983586487788014[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_9211072400700188435) {
   out_9211072400700188435[0] = 0;
   out_9211072400700188435[1] = 0;
   out_9211072400700188435[2] = 0;
   out_9211072400700188435[3] = 0;
   out_9211072400700188435[4] = 0;
   out_9211072400700188435[5] = 0;
   out_9211072400700188435[6] = 1;
   out_9211072400700188435[7] = 0;
   out_9211072400700188435[8] = 0;
   out_9211072400700188435[9] = 1;
   out_9211072400700188435[10] = 0;
   out_9211072400700188435[11] = 0;
   out_9211072400700188435[12] = 0;
   out_9211072400700188435[13] = 0;
   out_9211072400700188435[14] = 0;
   out_9211072400700188435[15] = 0;
   out_9211072400700188435[16] = 0;
   out_9211072400700188435[17] = 0;
   out_9211072400700188435[18] = 0;
   out_9211072400700188435[19] = 0;
   out_9211072400700188435[20] = 0;
   out_9211072400700188435[21] = 0;
   out_9211072400700188435[22] = 0;
   out_9211072400700188435[23] = 0;
   out_9211072400700188435[24] = 0;
   out_9211072400700188435[25] = 1;
   out_9211072400700188435[26] = 0;
   out_9211072400700188435[27] = 0;
   out_9211072400700188435[28] = 1;
   out_9211072400700188435[29] = 0;
   out_9211072400700188435[30] = 0;
   out_9211072400700188435[31] = 0;
   out_9211072400700188435[32] = 0;
   out_9211072400700188435[33] = 0;
   out_9211072400700188435[34] = 0;
   out_9211072400700188435[35] = 0;
   out_9211072400700188435[36] = 0;
   out_9211072400700188435[37] = 0;
   out_9211072400700188435[38] = 0;
   out_9211072400700188435[39] = 0;
   out_9211072400700188435[40] = 0;
   out_9211072400700188435[41] = 0;
   out_9211072400700188435[42] = 0;
   out_9211072400700188435[43] = 0;
   out_9211072400700188435[44] = 1;
   out_9211072400700188435[45] = 0;
   out_9211072400700188435[46] = 0;
   out_9211072400700188435[47] = 1;
   out_9211072400700188435[48] = 0;
   out_9211072400700188435[49] = 0;
   out_9211072400700188435[50] = 0;
   out_9211072400700188435[51] = 0;
   out_9211072400700188435[52] = 0;
   out_9211072400700188435[53] = 0;
}
void h_10(double *state, double *unused, double *out_2517419169749679290) {
   out_2517419169749679290[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2517419169749679290[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2517419169749679290[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3554499959659757522) {
   out_3554499959659757522[0] = 0;
   out_3554499959659757522[1] = 9.8100000000000005*cos(state[1]);
   out_3554499959659757522[2] = 0;
   out_3554499959659757522[3] = 0;
   out_3554499959659757522[4] = -state[8];
   out_3554499959659757522[5] = state[7];
   out_3554499959659757522[6] = 0;
   out_3554499959659757522[7] = state[5];
   out_3554499959659757522[8] = -state[4];
   out_3554499959659757522[9] = 0;
   out_3554499959659757522[10] = 0;
   out_3554499959659757522[11] = 0;
   out_3554499959659757522[12] = 1;
   out_3554499959659757522[13] = 0;
   out_3554499959659757522[14] = 0;
   out_3554499959659757522[15] = 1;
   out_3554499959659757522[16] = 0;
   out_3554499959659757522[17] = 0;
   out_3554499959659757522[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3554499959659757522[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3554499959659757522[20] = 0;
   out_3554499959659757522[21] = state[8];
   out_3554499959659757522[22] = 0;
   out_3554499959659757522[23] = -state[6];
   out_3554499959659757522[24] = -state[5];
   out_3554499959659757522[25] = 0;
   out_3554499959659757522[26] = state[3];
   out_3554499959659757522[27] = 0;
   out_3554499959659757522[28] = 0;
   out_3554499959659757522[29] = 0;
   out_3554499959659757522[30] = 0;
   out_3554499959659757522[31] = 1;
   out_3554499959659757522[32] = 0;
   out_3554499959659757522[33] = 0;
   out_3554499959659757522[34] = 1;
   out_3554499959659757522[35] = 0;
   out_3554499959659757522[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3554499959659757522[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3554499959659757522[38] = 0;
   out_3554499959659757522[39] = -state[7];
   out_3554499959659757522[40] = state[6];
   out_3554499959659757522[41] = 0;
   out_3554499959659757522[42] = state[4];
   out_3554499959659757522[43] = -state[3];
   out_3554499959659757522[44] = 0;
   out_3554499959659757522[45] = 0;
   out_3554499959659757522[46] = 0;
   out_3554499959659757522[47] = 0;
   out_3554499959659757522[48] = 0;
   out_3554499959659757522[49] = 0;
   out_3554499959659757522[50] = 1;
   out_3554499959659757522[51] = 0;
   out_3554499959659757522[52] = 0;
   out_3554499959659757522[53] = 1;
}
void h_13(double *state, double *unused, double *out_231752859429249092) {
   out_231752859429249092[0] = state[3];
   out_231752859429249092[1] = state[4];
   out_231752859429249092[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6023397847677030380) {
   out_6023397847677030380[0] = 0;
   out_6023397847677030380[1] = 0;
   out_6023397847677030380[2] = 0;
   out_6023397847677030380[3] = 1;
   out_6023397847677030380[4] = 0;
   out_6023397847677030380[5] = 0;
   out_6023397847677030380[6] = 0;
   out_6023397847677030380[7] = 0;
   out_6023397847677030380[8] = 0;
   out_6023397847677030380[9] = 0;
   out_6023397847677030380[10] = 0;
   out_6023397847677030380[11] = 0;
   out_6023397847677030380[12] = 0;
   out_6023397847677030380[13] = 0;
   out_6023397847677030380[14] = 0;
   out_6023397847677030380[15] = 0;
   out_6023397847677030380[16] = 0;
   out_6023397847677030380[17] = 0;
   out_6023397847677030380[18] = 0;
   out_6023397847677030380[19] = 0;
   out_6023397847677030380[20] = 0;
   out_6023397847677030380[21] = 0;
   out_6023397847677030380[22] = 1;
   out_6023397847677030380[23] = 0;
   out_6023397847677030380[24] = 0;
   out_6023397847677030380[25] = 0;
   out_6023397847677030380[26] = 0;
   out_6023397847677030380[27] = 0;
   out_6023397847677030380[28] = 0;
   out_6023397847677030380[29] = 0;
   out_6023397847677030380[30] = 0;
   out_6023397847677030380[31] = 0;
   out_6023397847677030380[32] = 0;
   out_6023397847677030380[33] = 0;
   out_6023397847677030380[34] = 0;
   out_6023397847677030380[35] = 0;
   out_6023397847677030380[36] = 0;
   out_6023397847677030380[37] = 0;
   out_6023397847677030380[38] = 0;
   out_6023397847677030380[39] = 0;
   out_6023397847677030380[40] = 0;
   out_6023397847677030380[41] = 1;
   out_6023397847677030380[42] = 0;
   out_6023397847677030380[43] = 0;
   out_6023397847677030380[44] = 0;
   out_6023397847677030380[45] = 0;
   out_6023397847677030380[46] = 0;
   out_6023397847677030380[47] = 0;
   out_6023397847677030380[48] = 0;
   out_6023397847677030380[49] = 0;
   out_6023397847677030380[50] = 0;
   out_6023397847677030380[51] = 0;
   out_6023397847677030380[52] = 0;
   out_6023397847677030380[53] = 0;
}
void h_14(double *state, double *unused, double *out_4883626546262470581) {
   out_4883626546262470581[0] = state[6];
   out_4883626546262470581[1] = state[7];
   out_4883626546262470581[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6128283968404816139) {
   out_6128283968404816139[0] = 0;
   out_6128283968404816139[1] = 0;
   out_6128283968404816139[2] = 0;
   out_6128283968404816139[3] = 0;
   out_6128283968404816139[4] = 0;
   out_6128283968404816139[5] = 0;
   out_6128283968404816139[6] = 1;
   out_6128283968404816139[7] = 0;
   out_6128283968404816139[8] = 0;
   out_6128283968404816139[9] = 0;
   out_6128283968404816139[10] = 0;
   out_6128283968404816139[11] = 0;
   out_6128283968404816139[12] = 0;
   out_6128283968404816139[13] = 0;
   out_6128283968404816139[14] = 0;
   out_6128283968404816139[15] = 0;
   out_6128283968404816139[16] = 0;
   out_6128283968404816139[17] = 0;
   out_6128283968404816139[18] = 0;
   out_6128283968404816139[19] = 0;
   out_6128283968404816139[20] = 0;
   out_6128283968404816139[21] = 0;
   out_6128283968404816139[22] = 0;
   out_6128283968404816139[23] = 0;
   out_6128283968404816139[24] = 0;
   out_6128283968404816139[25] = 1;
   out_6128283968404816139[26] = 0;
   out_6128283968404816139[27] = 0;
   out_6128283968404816139[28] = 0;
   out_6128283968404816139[29] = 0;
   out_6128283968404816139[30] = 0;
   out_6128283968404816139[31] = 0;
   out_6128283968404816139[32] = 0;
   out_6128283968404816139[33] = 0;
   out_6128283968404816139[34] = 0;
   out_6128283968404816139[35] = 0;
   out_6128283968404816139[36] = 0;
   out_6128283968404816139[37] = 0;
   out_6128283968404816139[38] = 0;
   out_6128283968404816139[39] = 0;
   out_6128283968404816139[40] = 0;
   out_6128283968404816139[41] = 0;
   out_6128283968404816139[42] = 0;
   out_6128283968404816139[43] = 0;
   out_6128283968404816139[44] = 1;
   out_6128283968404816139[45] = 0;
   out_6128283968404816139[46] = 0;
   out_6128283968404816139[47] = 0;
   out_6128283968404816139[48] = 0;
   out_6128283968404816139[49] = 0;
   out_6128283968404816139[50] = 0;
   out_6128283968404816139[51] = 0;
   out_6128283968404816139[52] = 0;
   out_6128283968404816139[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2114100039121924448) {
  err_fun(nom_x, delta_x, out_2114100039121924448);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2618631132718776507) {
  inv_err_fun(nom_x, true_x, out_2618631132718776507);
}
void pose_H_mod_fun(double *state, double *out_4058554015661107700) {
  H_mod_fun(state, out_4058554015661107700);
}
void pose_f_fun(double *state, double dt, double *out_5119116025028968719) {
  f_fun(state,  dt, out_5119116025028968719);
}
void pose_F_fun(double *state, double dt, double *out_6155576186277025652) {
  F_fun(state,  dt, out_6155576186277025652);
}
void pose_h_4(double *state, double *unused, double *out_8035983586487788014) {
  h_4(state, unused, out_8035983586487788014);
}
void pose_H_4(double *state, double *unused, double *out_9211072400700188435) {
  H_4(state, unused, out_9211072400700188435);
}
void pose_h_10(double *state, double *unused, double *out_2517419169749679290) {
  h_10(state, unused, out_2517419169749679290);
}
void pose_H_10(double *state, double *unused, double *out_3554499959659757522) {
  H_10(state, unused, out_3554499959659757522);
}
void pose_h_13(double *state, double *unused, double *out_231752859429249092) {
  h_13(state, unused, out_231752859429249092);
}
void pose_H_13(double *state, double *unused, double *out_6023397847677030380) {
  H_13(state, unused, out_6023397847677030380);
}
void pose_h_14(double *state, double *unused, double *out_4883626546262470581) {
  h_14(state, unused, out_4883626546262470581);
}
void pose_H_14(double *state, double *unused, double *out_6128283968404816139) {
  H_14(state, unused, out_6128283968404816139);
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
