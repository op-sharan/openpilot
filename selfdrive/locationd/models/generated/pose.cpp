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
void err_fun(double *nom_x, double *delta_x, double *out_1334608923131309184) {
   out_1334608923131309184[0] = delta_x[0] + nom_x[0];
   out_1334608923131309184[1] = delta_x[1] + nom_x[1];
   out_1334608923131309184[2] = delta_x[2] + nom_x[2];
   out_1334608923131309184[3] = delta_x[3] + nom_x[3];
   out_1334608923131309184[4] = delta_x[4] + nom_x[4];
   out_1334608923131309184[5] = delta_x[5] + nom_x[5];
   out_1334608923131309184[6] = delta_x[6] + nom_x[6];
   out_1334608923131309184[7] = delta_x[7] + nom_x[7];
   out_1334608923131309184[8] = delta_x[8] + nom_x[8];
   out_1334608923131309184[9] = delta_x[9] + nom_x[9];
   out_1334608923131309184[10] = delta_x[10] + nom_x[10];
   out_1334608923131309184[11] = delta_x[11] + nom_x[11];
   out_1334608923131309184[12] = delta_x[12] + nom_x[12];
   out_1334608923131309184[13] = delta_x[13] + nom_x[13];
   out_1334608923131309184[14] = delta_x[14] + nom_x[14];
   out_1334608923131309184[15] = delta_x[15] + nom_x[15];
   out_1334608923131309184[16] = delta_x[16] + nom_x[16];
   out_1334608923131309184[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5362236298631416382) {
   out_5362236298631416382[0] = -nom_x[0] + true_x[0];
   out_5362236298631416382[1] = -nom_x[1] + true_x[1];
   out_5362236298631416382[2] = -nom_x[2] + true_x[2];
   out_5362236298631416382[3] = -nom_x[3] + true_x[3];
   out_5362236298631416382[4] = -nom_x[4] + true_x[4];
   out_5362236298631416382[5] = -nom_x[5] + true_x[5];
   out_5362236298631416382[6] = -nom_x[6] + true_x[6];
   out_5362236298631416382[7] = -nom_x[7] + true_x[7];
   out_5362236298631416382[8] = -nom_x[8] + true_x[8];
   out_5362236298631416382[9] = -nom_x[9] + true_x[9];
   out_5362236298631416382[10] = -nom_x[10] + true_x[10];
   out_5362236298631416382[11] = -nom_x[11] + true_x[11];
   out_5362236298631416382[12] = -nom_x[12] + true_x[12];
   out_5362236298631416382[13] = -nom_x[13] + true_x[13];
   out_5362236298631416382[14] = -nom_x[14] + true_x[14];
   out_5362236298631416382[15] = -nom_x[15] + true_x[15];
   out_5362236298631416382[16] = -nom_x[16] + true_x[16];
   out_5362236298631416382[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4765044121338691268) {
   out_4765044121338691268[0] = 1.0;
   out_4765044121338691268[1] = 0.0;
   out_4765044121338691268[2] = 0.0;
   out_4765044121338691268[3] = 0.0;
   out_4765044121338691268[4] = 0.0;
   out_4765044121338691268[5] = 0.0;
   out_4765044121338691268[6] = 0.0;
   out_4765044121338691268[7] = 0.0;
   out_4765044121338691268[8] = 0.0;
   out_4765044121338691268[9] = 0.0;
   out_4765044121338691268[10] = 0.0;
   out_4765044121338691268[11] = 0.0;
   out_4765044121338691268[12] = 0.0;
   out_4765044121338691268[13] = 0.0;
   out_4765044121338691268[14] = 0.0;
   out_4765044121338691268[15] = 0.0;
   out_4765044121338691268[16] = 0.0;
   out_4765044121338691268[17] = 0.0;
   out_4765044121338691268[18] = 0.0;
   out_4765044121338691268[19] = 1.0;
   out_4765044121338691268[20] = 0.0;
   out_4765044121338691268[21] = 0.0;
   out_4765044121338691268[22] = 0.0;
   out_4765044121338691268[23] = 0.0;
   out_4765044121338691268[24] = 0.0;
   out_4765044121338691268[25] = 0.0;
   out_4765044121338691268[26] = 0.0;
   out_4765044121338691268[27] = 0.0;
   out_4765044121338691268[28] = 0.0;
   out_4765044121338691268[29] = 0.0;
   out_4765044121338691268[30] = 0.0;
   out_4765044121338691268[31] = 0.0;
   out_4765044121338691268[32] = 0.0;
   out_4765044121338691268[33] = 0.0;
   out_4765044121338691268[34] = 0.0;
   out_4765044121338691268[35] = 0.0;
   out_4765044121338691268[36] = 0.0;
   out_4765044121338691268[37] = 0.0;
   out_4765044121338691268[38] = 1.0;
   out_4765044121338691268[39] = 0.0;
   out_4765044121338691268[40] = 0.0;
   out_4765044121338691268[41] = 0.0;
   out_4765044121338691268[42] = 0.0;
   out_4765044121338691268[43] = 0.0;
   out_4765044121338691268[44] = 0.0;
   out_4765044121338691268[45] = 0.0;
   out_4765044121338691268[46] = 0.0;
   out_4765044121338691268[47] = 0.0;
   out_4765044121338691268[48] = 0.0;
   out_4765044121338691268[49] = 0.0;
   out_4765044121338691268[50] = 0.0;
   out_4765044121338691268[51] = 0.0;
   out_4765044121338691268[52] = 0.0;
   out_4765044121338691268[53] = 0.0;
   out_4765044121338691268[54] = 0.0;
   out_4765044121338691268[55] = 0.0;
   out_4765044121338691268[56] = 0.0;
   out_4765044121338691268[57] = 1.0;
   out_4765044121338691268[58] = 0.0;
   out_4765044121338691268[59] = 0.0;
   out_4765044121338691268[60] = 0.0;
   out_4765044121338691268[61] = 0.0;
   out_4765044121338691268[62] = 0.0;
   out_4765044121338691268[63] = 0.0;
   out_4765044121338691268[64] = 0.0;
   out_4765044121338691268[65] = 0.0;
   out_4765044121338691268[66] = 0.0;
   out_4765044121338691268[67] = 0.0;
   out_4765044121338691268[68] = 0.0;
   out_4765044121338691268[69] = 0.0;
   out_4765044121338691268[70] = 0.0;
   out_4765044121338691268[71] = 0.0;
   out_4765044121338691268[72] = 0.0;
   out_4765044121338691268[73] = 0.0;
   out_4765044121338691268[74] = 0.0;
   out_4765044121338691268[75] = 0.0;
   out_4765044121338691268[76] = 1.0;
   out_4765044121338691268[77] = 0.0;
   out_4765044121338691268[78] = 0.0;
   out_4765044121338691268[79] = 0.0;
   out_4765044121338691268[80] = 0.0;
   out_4765044121338691268[81] = 0.0;
   out_4765044121338691268[82] = 0.0;
   out_4765044121338691268[83] = 0.0;
   out_4765044121338691268[84] = 0.0;
   out_4765044121338691268[85] = 0.0;
   out_4765044121338691268[86] = 0.0;
   out_4765044121338691268[87] = 0.0;
   out_4765044121338691268[88] = 0.0;
   out_4765044121338691268[89] = 0.0;
   out_4765044121338691268[90] = 0.0;
   out_4765044121338691268[91] = 0.0;
   out_4765044121338691268[92] = 0.0;
   out_4765044121338691268[93] = 0.0;
   out_4765044121338691268[94] = 0.0;
   out_4765044121338691268[95] = 1.0;
   out_4765044121338691268[96] = 0.0;
   out_4765044121338691268[97] = 0.0;
   out_4765044121338691268[98] = 0.0;
   out_4765044121338691268[99] = 0.0;
   out_4765044121338691268[100] = 0.0;
   out_4765044121338691268[101] = 0.0;
   out_4765044121338691268[102] = 0.0;
   out_4765044121338691268[103] = 0.0;
   out_4765044121338691268[104] = 0.0;
   out_4765044121338691268[105] = 0.0;
   out_4765044121338691268[106] = 0.0;
   out_4765044121338691268[107] = 0.0;
   out_4765044121338691268[108] = 0.0;
   out_4765044121338691268[109] = 0.0;
   out_4765044121338691268[110] = 0.0;
   out_4765044121338691268[111] = 0.0;
   out_4765044121338691268[112] = 0.0;
   out_4765044121338691268[113] = 0.0;
   out_4765044121338691268[114] = 1.0;
   out_4765044121338691268[115] = 0.0;
   out_4765044121338691268[116] = 0.0;
   out_4765044121338691268[117] = 0.0;
   out_4765044121338691268[118] = 0.0;
   out_4765044121338691268[119] = 0.0;
   out_4765044121338691268[120] = 0.0;
   out_4765044121338691268[121] = 0.0;
   out_4765044121338691268[122] = 0.0;
   out_4765044121338691268[123] = 0.0;
   out_4765044121338691268[124] = 0.0;
   out_4765044121338691268[125] = 0.0;
   out_4765044121338691268[126] = 0.0;
   out_4765044121338691268[127] = 0.0;
   out_4765044121338691268[128] = 0.0;
   out_4765044121338691268[129] = 0.0;
   out_4765044121338691268[130] = 0.0;
   out_4765044121338691268[131] = 0.0;
   out_4765044121338691268[132] = 0.0;
   out_4765044121338691268[133] = 1.0;
   out_4765044121338691268[134] = 0.0;
   out_4765044121338691268[135] = 0.0;
   out_4765044121338691268[136] = 0.0;
   out_4765044121338691268[137] = 0.0;
   out_4765044121338691268[138] = 0.0;
   out_4765044121338691268[139] = 0.0;
   out_4765044121338691268[140] = 0.0;
   out_4765044121338691268[141] = 0.0;
   out_4765044121338691268[142] = 0.0;
   out_4765044121338691268[143] = 0.0;
   out_4765044121338691268[144] = 0.0;
   out_4765044121338691268[145] = 0.0;
   out_4765044121338691268[146] = 0.0;
   out_4765044121338691268[147] = 0.0;
   out_4765044121338691268[148] = 0.0;
   out_4765044121338691268[149] = 0.0;
   out_4765044121338691268[150] = 0.0;
   out_4765044121338691268[151] = 0.0;
   out_4765044121338691268[152] = 1.0;
   out_4765044121338691268[153] = 0.0;
   out_4765044121338691268[154] = 0.0;
   out_4765044121338691268[155] = 0.0;
   out_4765044121338691268[156] = 0.0;
   out_4765044121338691268[157] = 0.0;
   out_4765044121338691268[158] = 0.0;
   out_4765044121338691268[159] = 0.0;
   out_4765044121338691268[160] = 0.0;
   out_4765044121338691268[161] = 0.0;
   out_4765044121338691268[162] = 0.0;
   out_4765044121338691268[163] = 0.0;
   out_4765044121338691268[164] = 0.0;
   out_4765044121338691268[165] = 0.0;
   out_4765044121338691268[166] = 0.0;
   out_4765044121338691268[167] = 0.0;
   out_4765044121338691268[168] = 0.0;
   out_4765044121338691268[169] = 0.0;
   out_4765044121338691268[170] = 0.0;
   out_4765044121338691268[171] = 1.0;
   out_4765044121338691268[172] = 0.0;
   out_4765044121338691268[173] = 0.0;
   out_4765044121338691268[174] = 0.0;
   out_4765044121338691268[175] = 0.0;
   out_4765044121338691268[176] = 0.0;
   out_4765044121338691268[177] = 0.0;
   out_4765044121338691268[178] = 0.0;
   out_4765044121338691268[179] = 0.0;
   out_4765044121338691268[180] = 0.0;
   out_4765044121338691268[181] = 0.0;
   out_4765044121338691268[182] = 0.0;
   out_4765044121338691268[183] = 0.0;
   out_4765044121338691268[184] = 0.0;
   out_4765044121338691268[185] = 0.0;
   out_4765044121338691268[186] = 0.0;
   out_4765044121338691268[187] = 0.0;
   out_4765044121338691268[188] = 0.0;
   out_4765044121338691268[189] = 0.0;
   out_4765044121338691268[190] = 1.0;
   out_4765044121338691268[191] = 0.0;
   out_4765044121338691268[192] = 0.0;
   out_4765044121338691268[193] = 0.0;
   out_4765044121338691268[194] = 0.0;
   out_4765044121338691268[195] = 0.0;
   out_4765044121338691268[196] = 0.0;
   out_4765044121338691268[197] = 0.0;
   out_4765044121338691268[198] = 0.0;
   out_4765044121338691268[199] = 0.0;
   out_4765044121338691268[200] = 0.0;
   out_4765044121338691268[201] = 0.0;
   out_4765044121338691268[202] = 0.0;
   out_4765044121338691268[203] = 0.0;
   out_4765044121338691268[204] = 0.0;
   out_4765044121338691268[205] = 0.0;
   out_4765044121338691268[206] = 0.0;
   out_4765044121338691268[207] = 0.0;
   out_4765044121338691268[208] = 0.0;
   out_4765044121338691268[209] = 1.0;
   out_4765044121338691268[210] = 0.0;
   out_4765044121338691268[211] = 0.0;
   out_4765044121338691268[212] = 0.0;
   out_4765044121338691268[213] = 0.0;
   out_4765044121338691268[214] = 0.0;
   out_4765044121338691268[215] = 0.0;
   out_4765044121338691268[216] = 0.0;
   out_4765044121338691268[217] = 0.0;
   out_4765044121338691268[218] = 0.0;
   out_4765044121338691268[219] = 0.0;
   out_4765044121338691268[220] = 0.0;
   out_4765044121338691268[221] = 0.0;
   out_4765044121338691268[222] = 0.0;
   out_4765044121338691268[223] = 0.0;
   out_4765044121338691268[224] = 0.0;
   out_4765044121338691268[225] = 0.0;
   out_4765044121338691268[226] = 0.0;
   out_4765044121338691268[227] = 0.0;
   out_4765044121338691268[228] = 1.0;
   out_4765044121338691268[229] = 0.0;
   out_4765044121338691268[230] = 0.0;
   out_4765044121338691268[231] = 0.0;
   out_4765044121338691268[232] = 0.0;
   out_4765044121338691268[233] = 0.0;
   out_4765044121338691268[234] = 0.0;
   out_4765044121338691268[235] = 0.0;
   out_4765044121338691268[236] = 0.0;
   out_4765044121338691268[237] = 0.0;
   out_4765044121338691268[238] = 0.0;
   out_4765044121338691268[239] = 0.0;
   out_4765044121338691268[240] = 0.0;
   out_4765044121338691268[241] = 0.0;
   out_4765044121338691268[242] = 0.0;
   out_4765044121338691268[243] = 0.0;
   out_4765044121338691268[244] = 0.0;
   out_4765044121338691268[245] = 0.0;
   out_4765044121338691268[246] = 0.0;
   out_4765044121338691268[247] = 1.0;
   out_4765044121338691268[248] = 0.0;
   out_4765044121338691268[249] = 0.0;
   out_4765044121338691268[250] = 0.0;
   out_4765044121338691268[251] = 0.0;
   out_4765044121338691268[252] = 0.0;
   out_4765044121338691268[253] = 0.0;
   out_4765044121338691268[254] = 0.0;
   out_4765044121338691268[255] = 0.0;
   out_4765044121338691268[256] = 0.0;
   out_4765044121338691268[257] = 0.0;
   out_4765044121338691268[258] = 0.0;
   out_4765044121338691268[259] = 0.0;
   out_4765044121338691268[260] = 0.0;
   out_4765044121338691268[261] = 0.0;
   out_4765044121338691268[262] = 0.0;
   out_4765044121338691268[263] = 0.0;
   out_4765044121338691268[264] = 0.0;
   out_4765044121338691268[265] = 0.0;
   out_4765044121338691268[266] = 1.0;
   out_4765044121338691268[267] = 0.0;
   out_4765044121338691268[268] = 0.0;
   out_4765044121338691268[269] = 0.0;
   out_4765044121338691268[270] = 0.0;
   out_4765044121338691268[271] = 0.0;
   out_4765044121338691268[272] = 0.0;
   out_4765044121338691268[273] = 0.0;
   out_4765044121338691268[274] = 0.0;
   out_4765044121338691268[275] = 0.0;
   out_4765044121338691268[276] = 0.0;
   out_4765044121338691268[277] = 0.0;
   out_4765044121338691268[278] = 0.0;
   out_4765044121338691268[279] = 0.0;
   out_4765044121338691268[280] = 0.0;
   out_4765044121338691268[281] = 0.0;
   out_4765044121338691268[282] = 0.0;
   out_4765044121338691268[283] = 0.0;
   out_4765044121338691268[284] = 0.0;
   out_4765044121338691268[285] = 1.0;
   out_4765044121338691268[286] = 0.0;
   out_4765044121338691268[287] = 0.0;
   out_4765044121338691268[288] = 0.0;
   out_4765044121338691268[289] = 0.0;
   out_4765044121338691268[290] = 0.0;
   out_4765044121338691268[291] = 0.0;
   out_4765044121338691268[292] = 0.0;
   out_4765044121338691268[293] = 0.0;
   out_4765044121338691268[294] = 0.0;
   out_4765044121338691268[295] = 0.0;
   out_4765044121338691268[296] = 0.0;
   out_4765044121338691268[297] = 0.0;
   out_4765044121338691268[298] = 0.0;
   out_4765044121338691268[299] = 0.0;
   out_4765044121338691268[300] = 0.0;
   out_4765044121338691268[301] = 0.0;
   out_4765044121338691268[302] = 0.0;
   out_4765044121338691268[303] = 0.0;
   out_4765044121338691268[304] = 1.0;
   out_4765044121338691268[305] = 0.0;
   out_4765044121338691268[306] = 0.0;
   out_4765044121338691268[307] = 0.0;
   out_4765044121338691268[308] = 0.0;
   out_4765044121338691268[309] = 0.0;
   out_4765044121338691268[310] = 0.0;
   out_4765044121338691268[311] = 0.0;
   out_4765044121338691268[312] = 0.0;
   out_4765044121338691268[313] = 0.0;
   out_4765044121338691268[314] = 0.0;
   out_4765044121338691268[315] = 0.0;
   out_4765044121338691268[316] = 0.0;
   out_4765044121338691268[317] = 0.0;
   out_4765044121338691268[318] = 0.0;
   out_4765044121338691268[319] = 0.0;
   out_4765044121338691268[320] = 0.0;
   out_4765044121338691268[321] = 0.0;
   out_4765044121338691268[322] = 0.0;
   out_4765044121338691268[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2590892121983961236) {
   out_2590892121983961236[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2590892121983961236[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2590892121983961236[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2590892121983961236[3] = dt*state[12] + state[3];
   out_2590892121983961236[4] = dt*state[13] + state[4];
   out_2590892121983961236[5] = dt*state[14] + state[5];
   out_2590892121983961236[6] = state[6];
   out_2590892121983961236[7] = state[7];
   out_2590892121983961236[8] = state[8];
   out_2590892121983961236[9] = state[9];
   out_2590892121983961236[10] = state[10];
   out_2590892121983961236[11] = state[11];
   out_2590892121983961236[12] = state[12];
   out_2590892121983961236[13] = state[13];
   out_2590892121983961236[14] = state[14];
   out_2590892121983961236[15] = state[15];
   out_2590892121983961236[16] = state[16];
   out_2590892121983961236[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6449160121632499508) {
   out_6449160121632499508[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6449160121632499508[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6449160121632499508[2] = 0;
   out_6449160121632499508[3] = 0;
   out_6449160121632499508[4] = 0;
   out_6449160121632499508[5] = 0;
   out_6449160121632499508[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6449160121632499508[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6449160121632499508[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6449160121632499508[9] = 0;
   out_6449160121632499508[10] = 0;
   out_6449160121632499508[11] = 0;
   out_6449160121632499508[12] = 0;
   out_6449160121632499508[13] = 0;
   out_6449160121632499508[14] = 0;
   out_6449160121632499508[15] = 0;
   out_6449160121632499508[16] = 0;
   out_6449160121632499508[17] = 0;
   out_6449160121632499508[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6449160121632499508[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6449160121632499508[20] = 0;
   out_6449160121632499508[21] = 0;
   out_6449160121632499508[22] = 0;
   out_6449160121632499508[23] = 0;
   out_6449160121632499508[24] = 0;
   out_6449160121632499508[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6449160121632499508[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6449160121632499508[27] = 0;
   out_6449160121632499508[28] = 0;
   out_6449160121632499508[29] = 0;
   out_6449160121632499508[30] = 0;
   out_6449160121632499508[31] = 0;
   out_6449160121632499508[32] = 0;
   out_6449160121632499508[33] = 0;
   out_6449160121632499508[34] = 0;
   out_6449160121632499508[35] = 0;
   out_6449160121632499508[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6449160121632499508[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6449160121632499508[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6449160121632499508[39] = 0;
   out_6449160121632499508[40] = 0;
   out_6449160121632499508[41] = 0;
   out_6449160121632499508[42] = 0;
   out_6449160121632499508[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6449160121632499508[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6449160121632499508[45] = 0;
   out_6449160121632499508[46] = 0;
   out_6449160121632499508[47] = 0;
   out_6449160121632499508[48] = 0;
   out_6449160121632499508[49] = 0;
   out_6449160121632499508[50] = 0;
   out_6449160121632499508[51] = 0;
   out_6449160121632499508[52] = 0;
   out_6449160121632499508[53] = 0;
   out_6449160121632499508[54] = 0;
   out_6449160121632499508[55] = 0;
   out_6449160121632499508[56] = 0;
   out_6449160121632499508[57] = 1;
   out_6449160121632499508[58] = 0;
   out_6449160121632499508[59] = 0;
   out_6449160121632499508[60] = 0;
   out_6449160121632499508[61] = 0;
   out_6449160121632499508[62] = 0;
   out_6449160121632499508[63] = 0;
   out_6449160121632499508[64] = 0;
   out_6449160121632499508[65] = 0;
   out_6449160121632499508[66] = dt;
   out_6449160121632499508[67] = 0;
   out_6449160121632499508[68] = 0;
   out_6449160121632499508[69] = 0;
   out_6449160121632499508[70] = 0;
   out_6449160121632499508[71] = 0;
   out_6449160121632499508[72] = 0;
   out_6449160121632499508[73] = 0;
   out_6449160121632499508[74] = 0;
   out_6449160121632499508[75] = 0;
   out_6449160121632499508[76] = 1;
   out_6449160121632499508[77] = 0;
   out_6449160121632499508[78] = 0;
   out_6449160121632499508[79] = 0;
   out_6449160121632499508[80] = 0;
   out_6449160121632499508[81] = 0;
   out_6449160121632499508[82] = 0;
   out_6449160121632499508[83] = 0;
   out_6449160121632499508[84] = 0;
   out_6449160121632499508[85] = dt;
   out_6449160121632499508[86] = 0;
   out_6449160121632499508[87] = 0;
   out_6449160121632499508[88] = 0;
   out_6449160121632499508[89] = 0;
   out_6449160121632499508[90] = 0;
   out_6449160121632499508[91] = 0;
   out_6449160121632499508[92] = 0;
   out_6449160121632499508[93] = 0;
   out_6449160121632499508[94] = 0;
   out_6449160121632499508[95] = 1;
   out_6449160121632499508[96] = 0;
   out_6449160121632499508[97] = 0;
   out_6449160121632499508[98] = 0;
   out_6449160121632499508[99] = 0;
   out_6449160121632499508[100] = 0;
   out_6449160121632499508[101] = 0;
   out_6449160121632499508[102] = 0;
   out_6449160121632499508[103] = 0;
   out_6449160121632499508[104] = dt;
   out_6449160121632499508[105] = 0;
   out_6449160121632499508[106] = 0;
   out_6449160121632499508[107] = 0;
   out_6449160121632499508[108] = 0;
   out_6449160121632499508[109] = 0;
   out_6449160121632499508[110] = 0;
   out_6449160121632499508[111] = 0;
   out_6449160121632499508[112] = 0;
   out_6449160121632499508[113] = 0;
   out_6449160121632499508[114] = 1;
   out_6449160121632499508[115] = 0;
   out_6449160121632499508[116] = 0;
   out_6449160121632499508[117] = 0;
   out_6449160121632499508[118] = 0;
   out_6449160121632499508[119] = 0;
   out_6449160121632499508[120] = 0;
   out_6449160121632499508[121] = 0;
   out_6449160121632499508[122] = 0;
   out_6449160121632499508[123] = 0;
   out_6449160121632499508[124] = 0;
   out_6449160121632499508[125] = 0;
   out_6449160121632499508[126] = 0;
   out_6449160121632499508[127] = 0;
   out_6449160121632499508[128] = 0;
   out_6449160121632499508[129] = 0;
   out_6449160121632499508[130] = 0;
   out_6449160121632499508[131] = 0;
   out_6449160121632499508[132] = 0;
   out_6449160121632499508[133] = 1;
   out_6449160121632499508[134] = 0;
   out_6449160121632499508[135] = 0;
   out_6449160121632499508[136] = 0;
   out_6449160121632499508[137] = 0;
   out_6449160121632499508[138] = 0;
   out_6449160121632499508[139] = 0;
   out_6449160121632499508[140] = 0;
   out_6449160121632499508[141] = 0;
   out_6449160121632499508[142] = 0;
   out_6449160121632499508[143] = 0;
   out_6449160121632499508[144] = 0;
   out_6449160121632499508[145] = 0;
   out_6449160121632499508[146] = 0;
   out_6449160121632499508[147] = 0;
   out_6449160121632499508[148] = 0;
   out_6449160121632499508[149] = 0;
   out_6449160121632499508[150] = 0;
   out_6449160121632499508[151] = 0;
   out_6449160121632499508[152] = 1;
   out_6449160121632499508[153] = 0;
   out_6449160121632499508[154] = 0;
   out_6449160121632499508[155] = 0;
   out_6449160121632499508[156] = 0;
   out_6449160121632499508[157] = 0;
   out_6449160121632499508[158] = 0;
   out_6449160121632499508[159] = 0;
   out_6449160121632499508[160] = 0;
   out_6449160121632499508[161] = 0;
   out_6449160121632499508[162] = 0;
   out_6449160121632499508[163] = 0;
   out_6449160121632499508[164] = 0;
   out_6449160121632499508[165] = 0;
   out_6449160121632499508[166] = 0;
   out_6449160121632499508[167] = 0;
   out_6449160121632499508[168] = 0;
   out_6449160121632499508[169] = 0;
   out_6449160121632499508[170] = 0;
   out_6449160121632499508[171] = 1;
   out_6449160121632499508[172] = 0;
   out_6449160121632499508[173] = 0;
   out_6449160121632499508[174] = 0;
   out_6449160121632499508[175] = 0;
   out_6449160121632499508[176] = 0;
   out_6449160121632499508[177] = 0;
   out_6449160121632499508[178] = 0;
   out_6449160121632499508[179] = 0;
   out_6449160121632499508[180] = 0;
   out_6449160121632499508[181] = 0;
   out_6449160121632499508[182] = 0;
   out_6449160121632499508[183] = 0;
   out_6449160121632499508[184] = 0;
   out_6449160121632499508[185] = 0;
   out_6449160121632499508[186] = 0;
   out_6449160121632499508[187] = 0;
   out_6449160121632499508[188] = 0;
   out_6449160121632499508[189] = 0;
   out_6449160121632499508[190] = 1;
   out_6449160121632499508[191] = 0;
   out_6449160121632499508[192] = 0;
   out_6449160121632499508[193] = 0;
   out_6449160121632499508[194] = 0;
   out_6449160121632499508[195] = 0;
   out_6449160121632499508[196] = 0;
   out_6449160121632499508[197] = 0;
   out_6449160121632499508[198] = 0;
   out_6449160121632499508[199] = 0;
   out_6449160121632499508[200] = 0;
   out_6449160121632499508[201] = 0;
   out_6449160121632499508[202] = 0;
   out_6449160121632499508[203] = 0;
   out_6449160121632499508[204] = 0;
   out_6449160121632499508[205] = 0;
   out_6449160121632499508[206] = 0;
   out_6449160121632499508[207] = 0;
   out_6449160121632499508[208] = 0;
   out_6449160121632499508[209] = 1;
   out_6449160121632499508[210] = 0;
   out_6449160121632499508[211] = 0;
   out_6449160121632499508[212] = 0;
   out_6449160121632499508[213] = 0;
   out_6449160121632499508[214] = 0;
   out_6449160121632499508[215] = 0;
   out_6449160121632499508[216] = 0;
   out_6449160121632499508[217] = 0;
   out_6449160121632499508[218] = 0;
   out_6449160121632499508[219] = 0;
   out_6449160121632499508[220] = 0;
   out_6449160121632499508[221] = 0;
   out_6449160121632499508[222] = 0;
   out_6449160121632499508[223] = 0;
   out_6449160121632499508[224] = 0;
   out_6449160121632499508[225] = 0;
   out_6449160121632499508[226] = 0;
   out_6449160121632499508[227] = 0;
   out_6449160121632499508[228] = 1;
   out_6449160121632499508[229] = 0;
   out_6449160121632499508[230] = 0;
   out_6449160121632499508[231] = 0;
   out_6449160121632499508[232] = 0;
   out_6449160121632499508[233] = 0;
   out_6449160121632499508[234] = 0;
   out_6449160121632499508[235] = 0;
   out_6449160121632499508[236] = 0;
   out_6449160121632499508[237] = 0;
   out_6449160121632499508[238] = 0;
   out_6449160121632499508[239] = 0;
   out_6449160121632499508[240] = 0;
   out_6449160121632499508[241] = 0;
   out_6449160121632499508[242] = 0;
   out_6449160121632499508[243] = 0;
   out_6449160121632499508[244] = 0;
   out_6449160121632499508[245] = 0;
   out_6449160121632499508[246] = 0;
   out_6449160121632499508[247] = 1;
   out_6449160121632499508[248] = 0;
   out_6449160121632499508[249] = 0;
   out_6449160121632499508[250] = 0;
   out_6449160121632499508[251] = 0;
   out_6449160121632499508[252] = 0;
   out_6449160121632499508[253] = 0;
   out_6449160121632499508[254] = 0;
   out_6449160121632499508[255] = 0;
   out_6449160121632499508[256] = 0;
   out_6449160121632499508[257] = 0;
   out_6449160121632499508[258] = 0;
   out_6449160121632499508[259] = 0;
   out_6449160121632499508[260] = 0;
   out_6449160121632499508[261] = 0;
   out_6449160121632499508[262] = 0;
   out_6449160121632499508[263] = 0;
   out_6449160121632499508[264] = 0;
   out_6449160121632499508[265] = 0;
   out_6449160121632499508[266] = 1;
   out_6449160121632499508[267] = 0;
   out_6449160121632499508[268] = 0;
   out_6449160121632499508[269] = 0;
   out_6449160121632499508[270] = 0;
   out_6449160121632499508[271] = 0;
   out_6449160121632499508[272] = 0;
   out_6449160121632499508[273] = 0;
   out_6449160121632499508[274] = 0;
   out_6449160121632499508[275] = 0;
   out_6449160121632499508[276] = 0;
   out_6449160121632499508[277] = 0;
   out_6449160121632499508[278] = 0;
   out_6449160121632499508[279] = 0;
   out_6449160121632499508[280] = 0;
   out_6449160121632499508[281] = 0;
   out_6449160121632499508[282] = 0;
   out_6449160121632499508[283] = 0;
   out_6449160121632499508[284] = 0;
   out_6449160121632499508[285] = 1;
   out_6449160121632499508[286] = 0;
   out_6449160121632499508[287] = 0;
   out_6449160121632499508[288] = 0;
   out_6449160121632499508[289] = 0;
   out_6449160121632499508[290] = 0;
   out_6449160121632499508[291] = 0;
   out_6449160121632499508[292] = 0;
   out_6449160121632499508[293] = 0;
   out_6449160121632499508[294] = 0;
   out_6449160121632499508[295] = 0;
   out_6449160121632499508[296] = 0;
   out_6449160121632499508[297] = 0;
   out_6449160121632499508[298] = 0;
   out_6449160121632499508[299] = 0;
   out_6449160121632499508[300] = 0;
   out_6449160121632499508[301] = 0;
   out_6449160121632499508[302] = 0;
   out_6449160121632499508[303] = 0;
   out_6449160121632499508[304] = 1;
   out_6449160121632499508[305] = 0;
   out_6449160121632499508[306] = 0;
   out_6449160121632499508[307] = 0;
   out_6449160121632499508[308] = 0;
   out_6449160121632499508[309] = 0;
   out_6449160121632499508[310] = 0;
   out_6449160121632499508[311] = 0;
   out_6449160121632499508[312] = 0;
   out_6449160121632499508[313] = 0;
   out_6449160121632499508[314] = 0;
   out_6449160121632499508[315] = 0;
   out_6449160121632499508[316] = 0;
   out_6449160121632499508[317] = 0;
   out_6449160121632499508[318] = 0;
   out_6449160121632499508[319] = 0;
   out_6449160121632499508[320] = 0;
   out_6449160121632499508[321] = 0;
   out_6449160121632499508[322] = 0;
   out_6449160121632499508[323] = 1;
}
void h_4(double *state, double *unused, double *out_7784886013591512861) {
   out_7784886013591512861[0] = state[6] + state[9];
   out_7784886013591512861[1] = state[7] + state[10];
   out_7784886013591512861[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3567096639455793164) {
   out_3567096639455793164[0] = 0;
   out_3567096639455793164[1] = 0;
   out_3567096639455793164[2] = 0;
   out_3567096639455793164[3] = 0;
   out_3567096639455793164[4] = 0;
   out_3567096639455793164[5] = 0;
   out_3567096639455793164[6] = 1;
   out_3567096639455793164[7] = 0;
   out_3567096639455793164[8] = 0;
   out_3567096639455793164[9] = 1;
   out_3567096639455793164[10] = 0;
   out_3567096639455793164[11] = 0;
   out_3567096639455793164[12] = 0;
   out_3567096639455793164[13] = 0;
   out_3567096639455793164[14] = 0;
   out_3567096639455793164[15] = 0;
   out_3567096639455793164[16] = 0;
   out_3567096639455793164[17] = 0;
   out_3567096639455793164[18] = 0;
   out_3567096639455793164[19] = 0;
   out_3567096639455793164[20] = 0;
   out_3567096639455793164[21] = 0;
   out_3567096639455793164[22] = 0;
   out_3567096639455793164[23] = 0;
   out_3567096639455793164[24] = 0;
   out_3567096639455793164[25] = 1;
   out_3567096639455793164[26] = 0;
   out_3567096639455793164[27] = 0;
   out_3567096639455793164[28] = 1;
   out_3567096639455793164[29] = 0;
   out_3567096639455793164[30] = 0;
   out_3567096639455793164[31] = 0;
   out_3567096639455793164[32] = 0;
   out_3567096639455793164[33] = 0;
   out_3567096639455793164[34] = 0;
   out_3567096639455793164[35] = 0;
   out_3567096639455793164[36] = 0;
   out_3567096639455793164[37] = 0;
   out_3567096639455793164[38] = 0;
   out_3567096639455793164[39] = 0;
   out_3567096639455793164[40] = 0;
   out_3567096639455793164[41] = 0;
   out_3567096639455793164[42] = 0;
   out_3567096639455793164[43] = 0;
   out_3567096639455793164[44] = 1;
   out_3567096639455793164[45] = 0;
   out_3567096639455793164[46] = 0;
   out_3567096639455793164[47] = 1;
   out_3567096639455793164[48] = 0;
   out_3567096639455793164[49] = 0;
   out_3567096639455793164[50] = 0;
   out_3567096639455793164[51] = 0;
   out_3567096639455793164[52] = 0;
   out_3567096639455793164[53] = 0;
}
void h_10(double *state, double *unused, double *out_7334008623224553615) {
   out_7334008623224553615[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7334008623224553615[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7334008623224553615[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_157882816050330764) {
   out_157882816050330764[0] = 0;
   out_157882816050330764[1] = 9.8100000000000005*cos(state[1]);
   out_157882816050330764[2] = 0;
   out_157882816050330764[3] = 0;
   out_157882816050330764[4] = -state[8];
   out_157882816050330764[5] = state[7];
   out_157882816050330764[6] = 0;
   out_157882816050330764[7] = state[5];
   out_157882816050330764[8] = -state[4];
   out_157882816050330764[9] = 0;
   out_157882816050330764[10] = 0;
   out_157882816050330764[11] = 0;
   out_157882816050330764[12] = 1;
   out_157882816050330764[13] = 0;
   out_157882816050330764[14] = 0;
   out_157882816050330764[15] = 1;
   out_157882816050330764[16] = 0;
   out_157882816050330764[17] = 0;
   out_157882816050330764[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_157882816050330764[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_157882816050330764[20] = 0;
   out_157882816050330764[21] = state[8];
   out_157882816050330764[22] = 0;
   out_157882816050330764[23] = -state[6];
   out_157882816050330764[24] = -state[5];
   out_157882816050330764[25] = 0;
   out_157882816050330764[26] = state[3];
   out_157882816050330764[27] = 0;
   out_157882816050330764[28] = 0;
   out_157882816050330764[29] = 0;
   out_157882816050330764[30] = 0;
   out_157882816050330764[31] = 1;
   out_157882816050330764[32] = 0;
   out_157882816050330764[33] = 0;
   out_157882816050330764[34] = 1;
   out_157882816050330764[35] = 0;
   out_157882816050330764[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_157882816050330764[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_157882816050330764[38] = 0;
   out_157882816050330764[39] = -state[7];
   out_157882816050330764[40] = state[6];
   out_157882816050330764[41] = 0;
   out_157882816050330764[42] = state[4];
   out_157882816050330764[43] = -state[3];
   out_157882816050330764[44] = 0;
   out_157882816050330764[45] = 0;
   out_157882816050330764[46] = 0;
   out_157882816050330764[47] = 0;
   out_157882816050330764[48] = 0;
   out_157882816050330764[49] = 0;
   out_157882816050330764[50] = 1;
   out_157882816050330764[51] = 0;
   out_157882816050330764[52] = 0;
   out_157882816050330764[53] = 1;
}
void h_13(double *state, double *unused, double *out_7104614274408665918) {
   out_7104614274408665918[0] = state[3];
   out_7104614274408665918[1] = state[4];
   out_7104614274408665918[2] = state[5];
}
void H_13(double *state, double *unused, double *out_354822814123460363) {
   out_354822814123460363[0] = 0;
   out_354822814123460363[1] = 0;
   out_354822814123460363[2] = 0;
   out_354822814123460363[3] = 1;
   out_354822814123460363[4] = 0;
   out_354822814123460363[5] = 0;
   out_354822814123460363[6] = 0;
   out_354822814123460363[7] = 0;
   out_354822814123460363[8] = 0;
   out_354822814123460363[9] = 0;
   out_354822814123460363[10] = 0;
   out_354822814123460363[11] = 0;
   out_354822814123460363[12] = 0;
   out_354822814123460363[13] = 0;
   out_354822814123460363[14] = 0;
   out_354822814123460363[15] = 0;
   out_354822814123460363[16] = 0;
   out_354822814123460363[17] = 0;
   out_354822814123460363[18] = 0;
   out_354822814123460363[19] = 0;
   out_354822814123460363[20] = 0;
   out_354822814123460363[21] = 0;
   out_354822814123460363[22] = 1;
   out_354822814123460363[23] = 0;
   out_354822814123460363[24] = 0;
   out_354822814123460363[25] = 0;
   out_354822814123460363[26] = 0;
   out_354822814123460363[27] = 0;
   out_354822814123460363[28] = 0;
   out_354822814123460363[29] = 0;
   out_354822814123460363[30] = 0;
   out_354822814123460363[31] = 0;
   out_354822814123460363[32] = 0;
   out_354822814123460363[33] = 0;
   out_354822814123460363[34] = 0;
   out_354822814123460363[35] = 0;
   out_354822814123460363[36] = 0;
   out_354822814123460363[37] = 0;
   out_354822814123460363[38] = 0;
   out_354822814123460363[39] = 0;
   out_354822814123460363[40] = 0;
   out_354822814123460363[41] = 1;
   out_354822814123460363[42] = 0;
   out_354822814123460363[43] = 0;
   out_354822814123460363[44] = 0;
   out_354822814123460363[45] = 0;
   out_354822814123460363[46] = 0;
   out_354822814123460363[47] = 0;
   out_354822814123460363[48] = 0;
   out_354822814123460363[49] = 0;
   out_354822814123460363[50] = 0;
   out_354822814123460363[51] = 0;
   out_354822814123460363[52] = 0;
   out_354822814123460363[53] = 0;
}
void h_14(double *state, double *unused, double *out_7017778734957237132) {
   out_7017778734957237132[0] = state[6];
   out_7017778734957237132[1] = state[7];
   out_7017778734957237132[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6649885071751165460) {
   out_6649885071751165460[0] = 0;
   out_6649885071751165460[1] = 0;
   out_6649885071751165460[2] = 0;
   out_6649885071751165460[3] = 0;
   out_6649885071751165460[4] = 0;
   out_6649885071751165460[5] = 0;
   out_6649885071751165460[6] = 1;
   out_6649885071751165460[7] = 0;
   out_6649885071751165460[8] = 0;
   out_6649885071751165460[9] = 0;
   out_6649885071751165460[10] = 0;
   out_6649885071751165460[11] = 0;
   out_6649885071751165460[12] = 0;
   out_6649885071751165460[13] = 0;
   out_6649885071751165460[14] = 0;
   out_6649885071751165460[15] = 0;
   out_6649885071751165460[16] = 0;
   out_6649885071751165460[17] = 0;
   out_6649885071751165460[18] = 0;
   out_6649885071751165460[19] = 0;
   out_6649885071751165460[20] = 0;
   out_6649885071751165460[21] = 0;
   out_6649885071751165460[22] = 0;
   out_6649885071751165460[23] = 0;
   out_6649885071751165460[24] = 0;
   out_6649885071751165460[25] = 1;
   out_6649885071751165460[26] = 0;
   out_6649885071751165460[27] = 0;
   out_6649885071751165460[28] = 0;
   out_6649885071751165460[29] = 0;
   out_6649885071751165460[30] = 0;
   out_6649885071751165460[31] = 0;
   out_6649885071751165460[32] = 0;
   out_6649885071751165460[33] = 0;
   out_6649885071751165460[34] = 0;
   out_6649885071751165460[35] = 0;
   out_6649885071751165460[36] = 0;
   out_6649885071751165460[37] = 0;
   out_6649885071751165460[38] = 0;
   out_6649885071751165460[39] = 0;
   out_6649885071751165460[40] = 0;
   out_6649885071751165460[41] = 0;
   out_6649885071751165460[42] = 0;
   out_6649885071751165460[43] = 0;
   out_6649885071751165460[44] = 1;
   out_6649885071751165460[45] = 0;
   out_6649885071751165460[46] = 0;
   out_6649885071751165460[47] = 0;
   out_6649885071751165460[48] = 0;
   out_6649885071751165460[49] = 0;
   out_6649885071751165460[50] = 0;
   out_6649885071751165460[51] = 0;
   out_6649885071751165460[52] = 0;
   out_6649885071751165460[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_1334608923131309184) {
  err_fun(nom_x, delta_x, out_1334608923131309184);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5362236298631416382) {
  inv_err_fun(nom_x, true_x, out_5362236298631416382);
}
void pose_H_mod_fun(double *state, double *out_4765044121338691268) {
  H_mod_fun(state, out_4765044121338691268);
}
void pose_f_fun(double *state, double dt, double *out_2590892121983961236) {
  f_fun(state,  dt, out_2590892121983961236);
}
void pose_F_fun(double *state, double dt, double *out_6449160121632499508) {
  F_fun(state,  dt, out_6449160121632499508);
}
void pose_h_4(double *state, double *unused, double *out_7784886013591512861) {
  h_4(state, unused, out_7784886013591512861);
}
void pose_H_4(double *state, double *unused, double *out_3567096639455793164) {
  H_4(state, unused, out_3567096639455793164);
}
void pose_h_10(double *state, double *unused, double *out_7334008623224553615) {
  h_10(state, unused, out_7334008623224553615);
}
void pose_H_10(double *state, double *unused, double *out_157882816050330764) {
  H_10(state, unused, out_157882816050330764);
}
void pose_h_13(double *state, double *unused, double *out_7104614274408665918) {
  h_13(state, unused, out_7104614274408665918);
}
void pose_H_13(double *state, double *unused, double *out_354822814123460363) {
  H_13(state, unused, out_354822814123460363);
}
void pose_h_14(double *state, double *unused, double *out_7017778734957237132) {
  h_14(state, unused, out_7017778734957237132);
}
void pose_H_14(double *state, double *unused, double *out_6649885071751165460) {
  H_14(state, unused, out_6649885071751165460);
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
