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
void err_fun(double *nom_x, double *delta_x, double *out_5549195789352376606) {
   out_5549195789352376606[0] = delta_x[0] + nom_x[0];
   out_5549195789352376606[1] = delta_x[1] + nom_x[1];
   out_5549195789352376606[2] = delta_x[2] + nom_x[2];
   out_5549195789352376606[3] = delta_x[3] + nom_x[3];
   out_5549195789352376606[4] = delta_x[4] + nom_x[4];
   out_5549195789352376606[5] = delta_x[5] + nom_x[5];
   out_5549195789352376606[6] = delta_x[6] + nom_x[6];
   out_5549195789352376606[7] = delta_x[7] + nom_x[7];
   out_5549195789352376606[8] = delta_x[8] + nom_x[8];
   out_5549195789352376606[9] = delta_x[9] + nom_x[9];
   out_5549195789352376606[10] = delta_x[10] + nom_x[10];
   out_5549195789352376606[11] = delta_x[11] + nom_x[11];
   out_5549195789352376606[12] = delta_x[12] + nom_x[12];
   out_5549195789352376606[13] = delta_x[13] + nom_x[13];
   out_5549195789352376606[14] = delta_x[14] + nom_x[14];
   out_5549195789352376606[15] = delta_x[15] + nom_x[15];
   out_5549195789352376606[16] = delta_x[16] + nom_x[16];
   out_5549195789352376606[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6348441241420563996) {
   out_6348441241420563996[0] = -nom_x[0] + true_x[0];
   out_6348441241420563996[1] = -nom_x[1] + true_x[1];
   out_6348441241420563996[2] = -nom_x[2] + true_x[2];
   out_6348441241420563996[3] = -nom_x[3] + true_x[3];
   out_6348441241420563996[4] = -nom_x[4] + true_x[4];
   out_6348441241420563996[5] = -nom_x[5] + true_x[5];
   out_6348441241420563996[6] = -nom_x[6] + true_x[6];
   out_6348441241420563996[7] = -nom_x[7] + true_x[7];
   out_6348441241420563996[8] = -nom_x[8] + true_x[8];
   out_6348441241420563996[9] = -nom_x[9] + true_x[9];
   out_6348441241420563996[10] = -nom_x[10] + true_x[10];
   out_6348441241420563996[11] = -nom_x[11] + true_x[11];
   out_6348441241420563996[12] = -nom_x[12] + true_x[12];
   out_6348441241420563996[13] = -nom_x[13] + true_x[13];
   out_6348441241420563996[14] = -nom_x[14] + true_x[14];
   out_6348441241420563996[15] = -nom_x[15] + true_x[15];
   out_6348441241420563996[16] = -nom_x[16] + true_x[16];
   out_6348441241420563996[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_932088622648113863) {
   out_932088622648113863[0] = 1.0;
   out_932088622648113863[1] = 0.0;
   out_932088622648113863[2] = 0.0;
   out_932088622648113863[3] = 0.0;
   out_932088622648113863[4] = 0.0;
   out_932088622648113863[5] = 0.0;
   out_932088622648113863[6] = 0.0;
   out_932088622648113863[7] = 0.0;
   out_932088622648113863[8] = 0.0;
   out_932088622648113863[9] = 0.0;
   out_932088622648113863[10] = 0.0;
   out_932088622648113863[11] = 0.0;
   out_932088622648113863[12] = 0.0;
   out_932088622648113863[13] = 0.0;
   out_932088622648113863[14] = 0.0;
   out_932088622648113863[15] = 0.0;
   out_932088622648113863[16] = 0.0;
   out_932088622648113863[17] = 0.0;
   out_932088622648113863[18] = 0.0;
   out_932088622648113863[19] = 1.0;
   out_932088622648113863[20] = 0.0;
   out_932088622648113863[21] = 0.0;
   out_932088622648113863[22] = 0.0;
   out_932088622648113863[23] = 0.0;
   out_932088622648113863[24] = 0.0;
   out_932088622648113863[25] = 0.0;
   out_932088622648113863[26] = 0.0;
   out_932088622648113863[27] = 0.0;
   out_932088622648113863[28] = 0.0;
   out_932088622648113863[29] = 0.0;
   out_932088622648113863[30] = 0.0;
   out_932088622648113863[31] = 0.0;
   out_932088622648113863[32] = 0.0;
   out_932088622648113863[33] = 0.0;
   out_932088622648113863[34] = 0.0;
   out_932088622648113863[35] = 0.0;
   out_932088622648113863[36] = 0.0;
   out_932088622648113863[37] = 0.0;
   out_932088622648113863[38] = 1.0;
   out_932088622648113863[39] = 0.0;
   out_932088622648113863[40] = 0.0;
   out_932088622648113863[41] = 0.0;
   out_932088622648113863[42] = 0.0;
   out_932088622648113863[43] = 0.0;
   out_932088622648113863[44] = 0.0;
   out_932088622648113863[45] = 0.0;
   out_932088622648113863[46] = 0.0;
   out_932088622648113863[47] = 0.0;
   out_932088622648113863[48] = 0.0;
   out_932088622648113863[49] = 0.0;
   out_932088622648113863[50] = 0.0;
   out_932088622648113863[51] = 0.0;
   out_932088622648113863[52] = 0.0;
   out_932088622648113863[53] = 0.0;
   out_932088622648113863[54] = 0.0;
   out_932088622648113863[55] = 0.0;
   out_932088622648113863[56] = 0.0;
   out_932088622648113863[57] = 1.0;
   out_932088622648113863[58] = 0.0;
   out_932088622648113863[59] = 0.0;
   out_932088622648113863[60] = 0.0;
   out_932088622648113863[61] = 0.0;
   out_932088622648113863[62] = 0.0;
   out_932088622648113863[63] = 0.0;
   out_932088622648113863[64] = 0.0;
   out_932088622648113863[65] = 0.0;
   out_932088622648113863[66] = 0.0;
   out_932088622648113863[67] = 0.0;
   out_932088622648113863[68] = 0.0;
   out_932088622648113863[69] = 0.0;
   out_932088622648113863[70] = 0.0;
   out_932088622648113863[71] = 0.0;
   out_932088622648113863[72] = 0.0;
   out_932088622648113863[73] = 0.0;
   out_932088622648113863[74] = 0.0;
   out_932088622648113863[75] = 0.0;
   out_932088622648113863[76] = 1.0;
   out_932088622648113863[77] = 0.0;
   out_932088622648113863[78] = 0.0;
   out_932088622648113863[79] = 0.0;
   out_932088622648113863[80] = 0.0;
   out_932088622648113863[81] = 0.0;
   out_932088622648113863[82] = 0.0;
   out_932088622648113863[83] = 0.0;
   out_932088622648113863[84] = 0.0;
   out_932088622648113863[85] = 0.0;
   out_932088622648113863[86] = 0.0;
   out_932088622648113863[87] = 0.0;
   out_932088622648113863[88] = 0.0;
   out_932088622648113863[89] = 0.0;
   out_932088622648113863[90] = 0.0;
   out_932088622648113863[91] = 0.0;
   out_932088622648113863[92] = 0.0;
   out_932088622648113863[93] = 0.0;
   out_932088622648113863[94] = 0.0;
   out_932088622648113863[95] = 1.0;
   out_932088622648113863[96] = 0.0;
   out_932088622648113863[97] = 0.0;
   out_932088622648113863[98] = 0.0;
   out_932088622648113863[99] = 0.0;
   out_932088622648113863[100] = 0.0;
   out_932088622648113863[101] = 0.0;
   out_932088622648113863[102] = 0.0;
   out_932088622648113863[103] = 0.0;
   out_932088622648113863[104] = 0.0;
   out_932088622648113863[105] = 0.0;
   out_932088622648113863[106] = 0.0;
   out_932088622648113863[107] = 0.0;
   out_932088622648113863[108] = 0.0;
   out_932088622648113863[109] = 0.0;
   out_932088622648113863[110] = 0.0;
   out_932088622648113863[111] = 0.0;
   out_932088622648113863[112] = 0.0;
   out_932088622648113863[113] = 0.0;
   out_932088622648113863[114] = 1.0;
   out_932088622648113863[115] = 0.0;
   out_932088622648113863[116] = 0.0;
   out_932088622648113863[117] = 0.0;
   out_932088622648113863[118] = 0.0;
   out_932088622648113863[119] = 0.0;
   out_932088622648113863[120] = 0.0;
   out_932088622648113863[121] = 0.0;
   out_932088622648113863[122] = 0.0;
   out_932088622648113863[123] = 0.0;
   out_932088622648113863[124] = 0.0;
   out_932088622648113863[125] = 0.0;
   out_932088622648113863[126] = 0.0;
   out_932088622648113863[127] = 0.0;
   out_932088622648113863[128] = 0.0;
   out_932088622648113863[129] = 0.0;
   out_932088622648113863[130] = 0.0;
   out_932088622648113863[131] = 0.0;
   out_932088622648113863[132] = 0.0;
   out_932088622648113863[133] = 1.0;
   out_932088622648113863[134] = 0.0;
   out_932088622648113863[135] = 0.0;
   out_932088622648113863[136] = 0.0;
   out_932088622648113863[137] = 0.0;
   out_932088622648113863[138] = 0.0;
   out_932088622648113863[139] = 0.0;
   out_932088622648113863[140] = 0.0;
   out_932088622648113863[141] = 0.0;
   out_932088622648113863[142] = 0.0;
   out_932088622648113863[143] = 0.0;
   out_932088622648113863[144] = 0.0;
   out_932088622648113863[145] = 0.0;
   out_932088622648113863[146] = 0.0;
   out_932088622648113863[147] = 0.0;
   out_932088622648113863[148] = 0.0;
   out_932088622648113863[149] = 0.0;
   out_932088622648113863[150] = 0.0;
   out_932088622648113863[151] = 0.0;
   out_932088622648113863[152] = 1.0;
   out_932088622648113863[153] = 0.0;
   out_932088622648113863[154] = 0.0;
   out_932088622648113863[155] = 0.0;
   out_932088622648113863[156] = 0.0;
   out_932088622648113863[157] = 0.0;
   out_932088622648113863[158] = 0.0;
   out_932088622648113863[159] = 0.0;
   out_932088622648113863[160] = 0.0;
   out_932088622648113863[161] = 0.0;
   out_932088622648113863[162] = 0.0;
   out_932088622648113863[163] = 0.0;
   out_932088622648113863[164] = 0.0;
   out_932088622648113863[165] = 0.0;
   out_932088622648113863[166] = 0.0;
   out_932088622648113863[167] = 0.0;
   out_932088622648113863[168] = 0.0;
   out_932088622648113863[169] = 0.0;
   out_932088622648113863[170] = 0.0;
   out_932088622648113863[171] = 1.0;
   out_932088622648113863[172] = 0.0;
   out_932088622648113863[173] = 0.0;
   out_932088622648113863[174] = 0.0;
   out_932088622648113863[175] = 0.0;
   out_932088622648113863[176] = 0.0;
   out_932088622648113863[177] = 0.0;
   out_932088622648113863[178] = 0.0;
   out_932088622648113863[179] = 0.0;
   out_932088622648113863[180] = 0.0;
   out_932088622648113863[181] = 0.0;
   out_932088622648113863[182] = 0.0;
   out_932088622648113863[183] = 0.0;
   out_932088622648113863[184] = 0.0;
   out_932088622648113863[185] = 0.0;
   out_932088622648113863[186] = 0.0;
   out_932088622648113863[187] = 0.0;
   out_932088622648113863[188] = 0.0;
   out_932088622648113863[189] = 0.0;
   out_932088622648113863[190] = 1.0;
   out_932088622648113863[191] = 0.0;
   out_932088622648113863[192] = 0.0;
   out_932088622648113863[193] = 0.0;
   out_932088622648113863[194] = 0.0;
   out_932088622648113863[195] = 0.0;
   out_932088622648113863[196] = 0.0;
   out_932088622648113863[197] = 0.0;
   out_932088622648113863[198] = 0.0;
   out_932088622648113863[199] = 0.0;
   out_932088622648113863[200] = 0.0;
   out_932088622648113863[201] = 0.0;
   out_932088622648113863[202] = 0.0;
   out_932088622648113863[203] = 0.0;
   out_932088622648113863[204] = 0.0;
   out_932088622648113863[205] = 0.0;
   out_932088622648113863[206] = 0.0;
   out_932088622648113863[207] = 0.0;
   out_932088622648113863[208] = 0.0;
   out_932088622648113863[209] = 1.0;
   out_932088622648113863[210] = 0.0;
   out_932088622648113863[211] = 0.0;
   out_932088622648113863[212] = 0.0;
   out_932088622648113863[213] = 0.0;
   out_932088622648113863[214] = 0.0;
   out_932088622648113863[215] = 0.0;
   out_932088622648113863[216] = 0.0;
   out_932088622648113863[217] = 0.0;
   out_932088622648113863[218] = 0.0;
   out_932088622648113863[219] = 0.0;
   out_932088622648113863[220] = 0.0;
   out_932088622648113863[221] = 0.0;
   out_932088622648113863[222] = 0.0;
   out_932088622648113863[223] = 0.0;
   out_932088622648113863[224] = 0.0;
   out_932088622648113863[225] = 0.0;
   out_932088622648113863[226] = 0.0;
   out_932088622648113863[227] = 0.0;
   out_932088622648113863[228] = 1.0;
   out_932088622648113863[229] = 0.0;
   out_932088622648113863[230] = 0.0;
   out_932088622648113863[231] = 0.0;
   out_932088622648113863[232] = 0.0;
   out_932088622648113863[233] = 0.0;
   out_932088622648113863[234] = 0.0;
   out_932088622648113863[235] = 0.0;
   out_932088622648113863[236] = 0.0;
   out_932088622648113863[237] = 0.0;
   out_932088622648113863[238] = 0.0;
   out_932088622648113863[239] = 0.0;
   out_932088622648113863[240] = 0.0;
   out_932088622648113863[241] = 0.0;
   out_932088622648113863[242] = 0.0;
   out_932088622648113863[243] = 0.0;
   out_932088622648113863[244] = 0.0;
   out_932088622648113863[245] = 0.0;
   out_932088622648113863[246] = 0.0;
   out_932088622648113863[247] = 1.0;
   out_932088622648113863[248] = 0.0;
   out_932088622648113863[249] = 0.0;
   out_932088622648113863[250] = 0.0;
   out_932088622648113863[251] = 0.0;
   out_932088622648113863[252] = 0.0;
   out_932088622648113863[253] = 0.0;
   out_932088622648113863[254] = 0.0;
   out_932088622648113863[255] = 0.0;
   out_932088622648113863[256] = 0.0;
   out_932088622648113863[257] = 0.0;
   out_932088622648113863[258] = 0.0;
   out_932088622648113863[259] = 0.0;
   out_932088622648113863[260] = 0.0;
   out_932088622648113863[261] = 0.0;
   out_932088622648113863[262] = 0.0;
   out_932088622648113863[263] = 0.0;
   out_932088622648113863[264] = 0.0;
   out_932088622648113863[265] = 0.0;
   out_932088622648113863[266] = 1.0;
   out_932088622648113863[267] = 0.0;
   out_932088622648113863[268] = 0.0;
   out_932088622648113863[269] = 0.0;
   out_932088622648113863[270] = 0.0;
   out_932088622648113863[271] = 0.0;
   out_932088622648113863[272] = 0.0;
   out_932088622648113863[273] = 0.0;
   out_932088622648113863[274] = 0.0;
   out_932088622648113863[275] = 0.0;
   out_932088622648113863[276] = 0.0;
   out_932088622648113863[277] = 0.0;
   out_932088622648113863[278] = 0.0;
   out_932088622648113863[279] = 0.0;
   out_932088622648113863[280] = 0.0;
   out_932088622648113863[281] = 0.0;
   out_932088622648113863[282] = 0.0;
   out_932088622648113863[283] = 0.0;
   out_932088622648113863[284] = 0.0;
   out_932088622648113863[285] = 1.0;
   out_932088622648113863[286] = 0.0;
   out_932088622648113863[287] = 0.0;
   out_932088622648113863[288] = 0.0;
   out_932088622648113863[289] = 0.0;
   out_932088622648113863[290] = 0.0;
   out_932088622648113863[291] = 0.0;
   out_932088622648113863[292] = 0.0;
   out_932088622648113863[293] = 0.0;
   out_932088622648113863[294] = 0.0;
   out_932088622648113863[295] = 0.0;
   out_932088622648113863[296] = 0.0;
   out_932088622648113863[297] = 0.0;
   out_932088622648113863[298] = 0.0;
   out_932088622648113863[299] = 0.0;
   out_932088622648113863[300] = 0.0;
   out_932088622648113863[301] = 0.0;
   out_932088622648113863[302] = 0.0;
   out_932088622648113863[303] = 0.0;
   out_932088622648113863[304] = 1.0;
   out_932088622648113863[305] = 0.0;
   out_932088622648113863[306] = 0.0;
   out_932088622648113863[307] = 0.0;
   out_932088622648113863[308] = 0.0;
   out_932088622648113863[309] = 0.0;
   out_932088622648113863[310] = 0.0;
   out_932088622648113863[311] = 0.0;
   out_932088622648113863[312] = 0.0;
   out_932088622648113863[313] = 0.0;
   out_932088622648113863[314] = 0.0;
   out_932088622648113863[315] = 0.0;
   out_932088622648113863[316] = 0.0;
   out_932088622648113863[317] = 0.0;
   out_932088622648113863[318] = 0.0;
   out_932088622648113863[319] = 0.0;
   out_932088622648113863[320] = 0.0;
   out_932088622648113863[321] = 0.0;
   out_932088622648113863[322] = 0.0;
   out_932088622648113863[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_65737351960750703) {
   out_65737351960750703[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_65737351960750703[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_65737351960750703[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_65737351960750703[3] = dt*state[12] + state[3];
   out_65737351960750703[4] = dt*state[13] + state[4];
   out_65737351960750703[5] = dt*state[14] + state[5];
   out_65737351960750703[6] = state[6];
   out_65737351960750703[7] = state[7];
   out_65737351960750703[8] = state[8];
   out_65737351960750703[9] = state[9];
   out_65737351960750703[10] = state[10];
   out_65737351960750703[11] = state[11];
   out_65737351960750703[12] = state[12];
   out_65737351960750703[13] = state[13];
   out_65737351960750703[14] = state[14];
   out_65737351960750703[15] = state[15];
   out_65737351960750703[16] = state[16];
   out_65737351960750703[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3304224532205862059) {
   out_3304224532205862059[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3304224532205862059[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3304224532205862059[2] = 0;
   out_3304224532205862059[3] = 0;
   out_3304224532205862059[4] = 0;
   out_3304224532205862059[5] = 0;
   out_3304224532205862059[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3304224532205862059[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3304224532205862059[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3304224532205862059[9] = 0;
   out_3304224532205862059[10] = 0;
   out_3304224532205862059[11] = 0;
   out_3304224532205862059[12] = 0;
   out_3304224532205862059[13] = 0;
   out_3304224532205862059[14] = 0;
   out_3304224532205862059[15] = 0;
   out_3304224532205862059[16] = 0;
   out_3304224532205862059[17] = 0;
   out_3304224532205862059[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3304224532205862059[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3304224532205862059[20] = 0;
   out_3304224532205862059[21] = 0;
   out_3304224532205862059[22] = 0;
   out_3304224532205862059[23] = 0;
   out_3304224532205862059[24] = 0;
   out_3304224532205862059[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3304224532205862059[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3304224532205862059[27] = 0;
   out_3304224532205862059[28] = 0;
   out_3304224532205862059[29] = 0;
   out_3304224532205862059[30] = 0;
   out_3304224532205862059[31] = 0;
   out_3304224532205862059[32] = 0;
   out_3304224532205862059[33] = 0;
   out_3304224532205862059[34] = 0;
   out_3304224532205862059[35] = 0;
   out_3304224532205862059[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3304224532205862059[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3304224532205862059[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3304224532205862059[39] = 0;
   out_3304224532205862059[40] = 0;
   out_3304224532205862059[41] = 0;
   out_3304224532205862059[42] = 0;
   out_3304224532205862059[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3304224532205862059[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3304224532205862059[45] = 0;
   out_3304224532205862059[46] = 0;
   out_3304224532205862059[47] = 0;
   out_3304224532205862059[48] = 0;
   out_3304224532205862059[49] = 0;
   out_3304224532205862059[50] = 0;
   out_3304224532205862059[51] = 0;
   out_3304224532205862059[52] = 0;
   out_3304224532205862059[53] = 0;
   out_3304224532205862059[54] = 0;
   out_3304224532205862059[55] = 0;
   out_3304224532205862059[56] = 0;
   out_3304224532205862059[57] = 1;
   out_3304224532205862059[58] = 0;
   out_3304224532205862059[59] = 0;
   out_3304224532205862059[60] = 0;
   out_3304224532205862059[61] = 0;
   out_3304224532205862059[62] = 0;
   out_3304224532205862059[63] = 0;
   out_3304224532205862059[64] = 0;
   out_3304224532205862059[65] = 0;
   out_3304224532205862059[66] = dt;
   out_3304224532205862059[67] = 0;
   out_3304224532205862059[68] = 0;
   out_3304224532205862059[69] = 0;
   out_3304224532205862059[70] = 0;
   out_3304224532205862059[71] = 0;
   out_3304224532205862059[72] = 0;
   out_3304224532205862059[73] = 0;
   out_3304224532205862059[74] = 0;
   out_3304224532205862059[75] = 0;
   out_3304224532205862059[76] = 1;
   out_3304224532205862059[77] = 0;
   out_3304224532205862059[78] = 0;
   out_3304224532205862059[79] = 0;
   out_3304224532205862059[80] = 0;
   out_3304224532205862059[81] = 0;
   out_3304224532205862059[82] = 0;
   out_3304224532205862059[83] = 0;
   out_3304224532205862059[84] = 0;
   out_3304224532205862059[85] = dt;
   out_3304224532205862059[86] = 0;
   out_3304224532205862059[87] = 0;
   out_3304224532205862059[88] = 0;
   out_3304224532205862059[89] = 0;
   out_3304224532205862059[90] = 0;
   out_3304224532205862059[91] = 0;
   out_3304224532205862059[92] = 0;
   out_3304224532205862059[93] = 0;
   out_3304224532205862059[94] = 0;
   out_3304224532205862059[95] = 1;
   out_3304224532205862059[96] = 0;
   out_3304224532205862059[97] = 0;
   out_3304224532205862059[98] = 0;
   out_3304224532205862059[99] = 0;
   out_3304224532205862059[100] = 0;
   out_3304224532205862059[101] = 0;
   out_3304224532205862059[102] = 0;
   out_3304224532205862059[103] = 0;
   out_3304224532205862059[104] = dt;
   out_3304224532205862059[105] = 0;
   out_3304224532205862059[106] = 0;
   out_3304224532205862059[107] = 0;
   out_3304224532205862059[108] = 0;
   out_3304224532205862059[109] = 0;
   out_3304224532205862059[110] = 0;
   out_3304224532205862059[111] = 0;
   out_3304224532205862059[112] = 0;
   out_3304224532205862059[113] = 0;
   out_3304224532205862059[114] = 1;
   out_3304224532205862059[115] = 0;
   out_3304224532205862059[116] = 0;
   out_3304224532205862059[117] = 0;
   out_3304224532205862059[118] = 0;
   out_3304224532205862059[119] = 0;
   out_3304224532205862059[120] = 0;
   out_3304224532205862059[121] = 0;
   out_3304224532205862059[122] = 0;
   out_3304224532205862059[123] = 0;
   out_3304224532205862059[124] = 0;
   out_3304224532205862059[125] = 0;
   out_3304224532205862059[126] = 0;
   out_3304224532205862059[127] = 0;
   out_3304224532205862059[128] = 0;
   out_3304224532205862059[129] = 0;
   out_3304224532205862059[130] = 0;
   out_3304224532205862059[131] = 0;
   out_3304224532205862059[132] = 0;
   out_3304224532205862059[133] = 1;
   out_3304224532205862059[134] = 0;
   out_3304224532205862059[135] = 0;
   out_3304224532205862059[136] = 0;
   out_3304224532205862059[137] = 0;
   out_3304224532205862059[138] = 0;
   out_3304224532205862059[139] = 0;
   out_3304224532205862059[140] = 0;
   out_3304224532205862059[141] = 0;
   out_3304224532205862059[142] = 0;
   out_3304224532205862059[143] = 0;
   out_3304224532205862059[144] = 0;
   out_3304224532205862059[145] = 0;
   out_3304224532205862059[146] = 0;
   out_3304224532205862059[147] = 0;
   out_3304224532205862059[148] = 0;
   out_3304224532205862059[149] = 0;
   out_3304224532205862059[150] = 0;
   out_3304224532205862059[151] = 0;
   out_3304224532205862059[152] = 1;
   out_3304224532205862059[153] = 0;
   out_3304224532205862059[154] = 0;
   out_3304224532205862059[155] = 0;
   out_3304224532205862059[156] = 0;
   out_3304224532205862059[157] = 0;
   out_3304224532205862059[158] = 0;
   out_3304224532205862059[159] = 0;
   out_3304224532205862059[160] = 0;
   out_3304224532205862059[161] = 0;
   out_3304224532205862059[162] = 0;
   out_3304224532205862059[163] = 0;
   out_3304224532205862059[164] = 0;
   out_3304224532205862059[165] = 0;
   out_3304224532205862059[166] = 0;
   out_3304224532205862059[167] = 0;
   out_3304224532205862059[168] = 0;
   out_3304224532205862059[169] = 0;
   out_3304224532205862059[170] = 0;
   out_3304224532205862059[171] = 1;
   out_3304224532205862059[172] = 0;
   out_3304224532205862059[173] = 0;
   out_3304224532205862059[174] = 0;
   out_3304224532205862059[175] = 0;
   out_3304224532205862059[176] = 0;
   out_3304224532205862059[177] = 0;
   out_3304224532205862059[178] = 0;
   out_3304224532205862059[179] = 0;
   out_3304224532205862059[180] = 0;
   out_3304224532205862059[181] = 0;
   out_3304224532205862059[182] = 0;
   out_3304224532205862059[183] = 0;
   out_3304224532205862059[184] = 0;
   out_3304224532205862059[185] = 0;
   out_3304224532205862059[186] = 0;
   out_3304224532205862059[187] = 0;
   out_3304224532205862059[188] = 0;
   out_3304224532205862059[189] = 0;
   out_3304224532205862059[190] = 1;
   out_3304224532205862059[191] = 0;
   out_3304224532205862059[192] = 0;
   out_3304224532205862059[193] = 0;
   out_3304224532205862059[194] = 0;
   out_3304224532205862059[195] = 0;
   out_3304224532205862059[196] = 0;
   out_3304224532205862059[197] = 0;
   out_3304224532205862059[198] = 0;
   out_3304224532205862059[199] = 0;
   out_3304224532205862059[200] = 0;
   out_3304224532205862059[201] = 0;
   out_3304224532205862059[202] = 0;
   out_3304224532205862059[203] = 0;
   out_3304224532205862059[204] = 0;
   out_3304224532205862059[205] = 0;
   out_3304224532205862059[206] = 0;
   out_3304224532205862059[207] = 0;
   out_3304224532205862059[208] = 0;
   out_3304224532205862059[209] = 1;
   out_3304224532205862059[210] = 0;
   out_3304224532205862059[211] = 0;
   out_3304224532205862059[212] = 0;
   out_3304224532205862059[213] = 0;
   out_3304224532205862059[214] = 0;
   out_3304224532205862059[215] = 0;
   out_3304224532205862059[216] = 0;
   out_3304224532205862059[217] = 0;
   out_3304224532205862059[218] = 0;
   out_3304224532205862059[219] = 0;
   out_3304224532205862059[220] = 0;
   out_3304224532205862059[221] = 0;
   out_3304224532205862059[222] = 0;
   out_3304224532205862059[223] = 0;
   out_3304224532205862059[224] = 0;
   out_3304224532205862059[225] = 0;
   out_3304224532205862059[226] = 0;
   out_3304224532205862059[227] = 0;
   out_3304224532205862059[228] = 1;
   out_3304224532205862059[229] = 0;
   out_3304224532205862059[230] = 0;
   out_3304224532205862059[231] = 0;
   out_3304224532205862059[232] = 0;
   out_3304224532205862059[233] = 0;
   out_3304224532205862059[234] = 0;
   out_3304224532205862059[235] = 0;
   out_3304224532205862059[236] = 0;
   out_3304224532205862059[237] = 0;
   out_3304224532205862059[238] = 0;
   out_3304224532205862059[239] = 0;
   out_3304224532205862059[240] = 0;
   out_3304224532205862059[241] = 0;
   out_3304224532205862059[242] = 0;
   out_3304224532205862059[243] = 0;
   out_3304224532205862059[244] = 0;
   out_3304224532205862059[245] = 0;
   out_3304224532205862059[246] = 0;
   out_3304224532205862059[247] = 1;
   out_3304224532205862059[248] = 0;
   out_3304224532205862059[249] = 0;
   out_3304224532205862059[250] = 0;
   out_3304224532205862059[251] = 0;
   out_3304224532205862059[252] = 0;
   out_3304224532205862059[253] = 0;
   out_3304224532205862059[254] = 0;
   out_3304224532205862059[255] = 0;
   out_3304224532205862059[256] = 0;
   out_3304224532205862059[257] = 0;
   out_3304224532205862059[258] = 0;
   out_3304224532205862059[259] = 0;
   out_3304224532205862059[260] = 0;
   out_3304224532205862059[261] = 0;
   out_3304224532205862059[262] = 0;
   out_3304224532205862059[263] = 0;
   out_3304224532205862059[264] = 0;
   out_3304224532205862059[265] = 0;
   out_3304224532205862059[266] = 1;
   out_3304224532205862059[267] = 0;
   out_3304224532205862059[268] = 0;
   out_3304224532205862059[269] = 0;
   out_3304224532205862059[270] = 0;
   out_3304224532205862059[271] = 0;
   out_3304224532205862059[272] = 0;
   out_3304224532205862059[273] = 0;
   out_3304224532205862059[274] = 0;
   out_3304224532205862059[275] = 0;
   out_3304224532205862059[276] = 0;
   out_3304224532205862059[277] = 0;
   out_3304224532205862059[278] = 0;
   out_3304224532205862059[279] = 0;
   out_3304224532205862059[280] = 0;
   out_3304224532205862059[281] = 0;
   out_3304224532205862059[282] = 0;
   out_3304224532205862059[283] = 0;
   out_3304224532205862059[284] = 0;
   out_3304224532205862059[285] = 1;
   out_3304224532205862059[286] = 0;
   out_3304224532205862059[287] = 0;
   out_3304224532205862059[288] = 0;
   out_3304224532205862059[289] = 0;
   out_3304224532205862059[290] = 0;
   out_3304224532205862059[291] = 0;
   out_3304224532205862059[292] = 0;
   out_3304224532205862059[293] = 0;
   out_3304224532205862059[294] = 0;
   out_3304224532205862059[295] = 0;
   out_3304224532205862059[296] = 0;
   out_3304224532205862059[297] = 0;
   out_3304224532205862059[298] = 0;
   out_3304224532205862059[299] = 0;
   out_3304224532205862059[300] = 0;
   out_3304224532205862059[301] = 0;
   out_3304224532205862059[302] = 0;
   out_3304224532205862059[303] = 0;
   out_3304224532205862059[304] = 1;
   out_3304224532205862059[305] = 0;
   out_3304224532205862059[306] = 0;
   out_3304224532205862059[307] = 0;
   out_3304224532205862059[308] = 0;
   out_3304224532205862059[309] = 0;
   out_3304224532205862059[310] = 0;
   out_3304224532205862059[311] = 0;
   out_3304224532205862059[312] = 0;
   out_3304224532205862059[313] = 0;
   out_3304224532205862059[314] = 0;
   out_3304224532205862059[315] = 0;
   out_3304224532205862059[316] = 0;
   out_3304224532205862059[317] = 0;
   out_3304224532205862059[318] = 0;
   out_3304224532205862059[319] = 0;
   out_3304224532205862059[320] = 0;
   out_3304224532205862059[321] = 0;
   out_3304224532205862059[322] = 0;
   out_3304224532205862059[323] = 1;
}
void h_4(double *state, double *unused, double *out_5378292505758393229) {
   out_5378292505758393229[0] = state[6] + state[9];
   out_5378292505758393229[1] = state[7] + state[10];
   out_5378292505758393229[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2130036104531011967) {
   out_2130036104531011967[0] = 0;
   out_2130036104531011967[1] = 0;
   out_2130036104531011967[2] = 0;
   out_2130036104531011967[3] = 0;
   out_2130036104531011967[4] = 0;
   out_2130036104531011967[5] = 0;
   out_2130036104531011967[6] = 1;
   out_2130036104531011967[7] = 0;
   out_2130036104531011967[8] = 0;
   out_2130036104531011967[9] = 1;
   out_2130036104531011967[10] = 0;
   out_2130036104531011967[11] = 0;
   out_2130036104531011967[12] = 0;
   out_2130036104531011967[13] = 0;
   out_2130036104531011967[14] = 0;
   out_2130036104531011967[15] = 0;
   out_2130036104531011967[16] = 0;
   out_2130036104531011967[17] = 0;
   out_2130036104531011967[18] = 0;
   out_2130036104531011967[19] = 0;
   out_2130036104531011967[20] = 0;
   out_2130036104531011967[21] = 0;
   out_2130036104531011967[22] = 0;
   out_2130036104531011967[23] = 0;
   out_2130036104531011967[24] = 0;
   out_2130036104531011967[25] = 1;
   out_2130036104531011967[26] = 0;
   out_2130036104531011967[27] = 0;
   out_2130036104531011967[28] = 1;
   out_2130036104531011967[29] = 0;
   out_2130036104531011967[30] = 0;
   out_2130036104531011967[31] = 0;
   out_2130036104531011967[32] = 0;
   out_2130036104531011967[33] = 0;
   out_2130036104531011967[34] = 0;
   out_2130036104531011967[35] = 0;
   out_2130036104531011967[36] = 0;
   out_2130036104531011967[37] = 0;
   out_2130036104531011967[38] = 0;
   out_2130036104531011967[39] = 0;
   out_2130036104531011967[40] = 0;
   out_2130036104531011967[41] = 0;
   out_2130036104531011967[42] = 0;
   out_2130036104531011967[43] = 0;
   out_2130036104531011967[44] = 1;
   out_2130036104531011967[45] = 0;
   out_2130036104531011967[46] = 0;
   out_2130036104531011967[47] = 1;
   out_2130036104531011967[48] = 0;
   out_2130036104531011967[49] = 0;
   out_2130036104531011967[50] = 0;
   out_2130036104531011967[51] = 0;
   out_2130036104531011967[52] = 0;
   out_2130036104531011967[53] = 0;
}
void h_10(double *state, double *unused, double *out_2124950356332181460) {
   out_2124950356332181460[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2124950356332181460[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2124950356332181460[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8589748122456887360) {
   out_8589748122456887360[0] = 0;
   out_8589748122456887360[1] = 9.8100000000000005*cos(state[1]);
   out_8589748122456887360[2] = 0;
   out_8589748122456887360[3] = 0;
   out_8589748122456887360[4] = -state[8];
   out_8589748122456887360[5] = state[7];
   out_8589748122456887360[6] = 0;
   out_8589748122456887360[7] = state[5];
   out_8589748122456887360[8] = -state[4];
   out_8589748122456887360[9] = 0;
   out_8589748122456887360[10] = 0;
   out_8589748122456887360[11] = 0;
   out_8589748122456887360[12] = 1;
   out_8589748122456887360[13] = 0;
   out_8589748122456887360[14] = 0;
   out_8589748122456887360[15] = 1;
   out_8589748122456887360[16] = 0;
   out_8589748122456887360[17] = 0;
   out_8589748122456887360[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8589748122456887360[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8589748122456887360[20] = 0;
   out_8589748122456887360[21] = state[8];
   out_8589748122456887360[22] = 0;
   out_8589748122456887360[23] = -state[6];
   out_8589748122456887360[24] = -state[5];
   out_8589748122456887360[25] = 0;
   out_8589748122456887360[26] = state[3];
   out_8589748122456887360[27] = 0;
   out_8589748122456887360[28] = 0;
   out_8589748122456887360[29] = 0;
   out_8589748122456887360[30] = 0;
   out_8589748122456887360[31] = 1;
   out_8589748122456887360[32] = 0;
   out_8589748122456887360[33] = 0;
   out_8589748122456887360[34] = 1;
   out_8589748122456887360[35] = 0;
   out_8589748122456887360[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8589748122456887360[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8589748122456887360[38] = 0;
   out_8589748122456887360[39] = -state[7];
   out_8589748122456887360[40] = state[6];
   out_8589748122456887360[41] = 0;
   out_8589748122456887360[42] = state[4];
   out_8589748122456887360[43] = -state[3];
   out_8589748122456887360[44] = 0;
   out_8589748122456887360[45] = 0;
   out_8589748122456887360[46] = 0;
   out_8589748122456887360[47] = 0;
   out_8589748122456887360[48] = 0;
   out_8589748122456887360[49] = 0;
   out_8589748122456887360[50] = 1;
   out_8589748122456887360[51] = 0;
   out_8589748122456887360[52] = 0;
   out_8589748122456887360[53] = 1;
}
void h_13(double *state, double *unused, double *out_7381262973170721868) {
   out_7381262973170721868[0] = state[3];
   out_7381262973170721868[1] = state[4];
   out_7381262973170721868[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5342309929863344768) {
   out_5342309929863344768[0] = 0;
   out_5342309929863344768[1] = 0;
   out_5342309929863344768[2] = 0;
   out_5342309929863344768[3] = 1;
   out_5342309929863344768[4] = 0;
   out_5342309929863344768[5] = 0;
   out_5342309929863344768[6] = 0;
   out_5342309929863344768[7] = 0;
   out_5342309929863344768[8] = 0;
   out_5342309929863344768[9] = 0;
   out_5342309929863344768[10] = 0;
   out_5342309929863344768[11] = 0;
   out_5342309929863344768[12] = 0;
   out_5342309929863344768[13] = 0;
   out_5342309929863344768[14] = 0;
   out_5342309929863344768[15] = 0;
   out_5342309929863344768[16] = 0;
   out_5342309929863344768[17] = 0;
   out_5342309929863344768[18] = 0;
   out_5342309929863344768[19] = 0;
   out_5342309929863344768[20] = 0;
   out_5342309929863344768[21] = 0;
   out_5342309929863344768[22] = 1;
   out_5342309929863344768[23] = 0;
   out_5342309929863344768[24] = 0;
   out_5342309929863344768[25] = 0;
   out_5342309929863344768[26] = 0;
   out_5342309929863344768[27] = 0;
   out_5342309929863344768[28] = 0;
   out_5342309929863344768[29] = 0;
   out_5342309929863344768[30] = 0;
   out_5342309929863344768[31] = 0;
   out_5342309929863344768[32] = 0;
   out_5342309929863344768[33] = 0;
   out_5342309929863344768[34] = 0;
   out_5342309929863344768[35] = 0;
   out_5342309929863344768[36] = 0;
   out_5342309929863344768[37] = 0;
   out_5342309929863344768[38] = 0;
   out_5342309929863344768[39] = 0;
   out_5342309929863344768[40] = 0;
   out_5342309929863344768[41] = 1;
   out_5342309929863344768[42] = 0;
   out_5342309929863344768[43] = 0;
   out_5342309929863344768[44] = 0;
   out_5342309929863344768[45] = 0;
   out_5342309929863344768[46] = 0;
   out_5342309929863344768[47] = 0;
   out_5342309929863344768[48] = 0;
   out_5342309929863344768[49] = 0;
   out_5342309929863344768[50] = 0;
   out_5342309929863344768[51] = 0;
   out_5342309929863344768[52] = 0;
   out_5342309929863344768[53] = 0;
}
void h_14(double *state, double *unused, double *out_2697166449345768413) {
   out_2697166449345768413[0] = state[6];
   out_2697166449345768413[1] = state[7];
   out_2697166449345768413[2] = state[8];
}
void H_14(double *state, double *unused, double *out_952752327764360329) {
   out_952752327764360329[0] = 0;
   out_952752327764360329[1] = 0;
   out_952752327764360329[2] = 0;
   out_952752327764360329[3] = 0;
   out_952752327764360329[4] = 0;
   out_952752327764360329[5] = 0;
   out_952752327764360329[6] = 1;
   out_952752327764360329[7] = 0;
   out_952752327764360329[8] = 0;
   out_952752327764360329[9] = 0;
   out_952752327764360329[10] = 0;
   out_952752327764360329[11] = 0;
   out_952752327764360329[12] = 0;
   out_952752327764360329[13] = 0;
   out_952752327764360329[14] = 0;
   out_952752327764360329[15] = 0;
   out_952752327764360329[16] = 0;
   out_952752327764360329[17] = 0;
   out_952752327764360329[18] = 0;
   out_952752327764360329[19] = 0;
   out_952752327764360329[20] = 0;
   out_952752327764360329[21] = 0;
   out_952752327764360329[22] = 0;
   out_952752327764360329[23] = 0;
   out_952752327764360329[24] = 0;
   out_952752327764360329[25] = 1;
   out_952752327764360329[26] = 0;
   out_952752327764360329[27] = 0;
   out_952752327764360329[28] = 0;
   out_952752327764360329[29] = 0;
   out_952752327764360329[30] = 0;
   out_952752327764360329[31] = 0;
   out_952752327764360329[32] = 0;
   out_952752327764360329[33] = 0;
   out_952752327764360329[34] = 0;
   out_952752327764360329[35] = 0;
   out_952752327764360329[36] = 0;
   out_952752327764360329[37] = 0;
   out_952752327764360329[38] = 0;
   out_952752327764360329[39] = 0;
   out_952752327764360329[40] = 0;
   out_952752327764360329[41] = 0;
   out_952752327764360329[42] = 0;
   out_952752327764360329[43] = 0;
   out_952752327764360329[44] = 1;
   out_952752327764360329[45] = 0;
   out_952752327764360329[46] = 0;
   out_952752327764360329[47] = 0;
   out_952752327764360329[48] = 0;
   out_952752327764360329[49] = 0;
   out_952752327764360329[50] = 0;
   out_952752327764360329[51] = 0;
   out_952752327764360329[52] = 0;
   out_952752327764360329[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_5549195789352376606) {
  err_fun(nom_x, delta_x, out_5549195789352376606);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6348441241420563996) {
  inv_err_fun(nom_x, true_x, out_6348441241420563996);
}
void pose_H_mod_fun(double *state, double *out_932088622648113863) {
  H_mod_fun(state, out_932088622648113863);
}
void pose_f_fun(double *state, double dt, double *out_65737351960750703) {
  f_fun(state,  dt, out_65737351960750703);
}
void pose_F_fun(double *state, double dt, double *out_3304224532205862059) {
  F_fun(state,  dt, out_3304224532205862059);
}
void pose_h_4(double *state, double *unused, double *out_5378292505758393229) {
  h_4(state, unused, out_5378292505758393229);
}
void pose_H_4(double *state, double *unused, double *out_2130036104531011967) {
  H_4(state, unused, out_2130036104531011967);
}
void pose_h_10(double *state, double *unused, double *out_2124950356332181460) {
  h_10(state, unused, out_2124950356332181460);
}
void pose_H_10(double *state, double *unused, double *out_8589748122456887360) {
  H_10(state, unused, out_8589748122456887360);
}
void pose_h_13(double *state, double *unused, double *out_7381262973170721868) {
  h_13(state, unused, out_7381262973170721868);
}
void pose_H_13(double *state, double *unused, double *out_5342309929863344768) {
  H_13(state, unused, out_5342309929863344768);
}
void pose_h_14(double *state, double *unused, double *out_2697166449345768413) {
  h_14(state, unused, out_2697166449345768413);
}
void pose_H_14(double *state, double *unused, double *out_952752327764360329) {
  H_14(state, unused, out_952752327764360329);
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
