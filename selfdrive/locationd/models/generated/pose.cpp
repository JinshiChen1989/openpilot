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
void err_fun(double *nom_x, double *delta_x, double *out_5871344221582215751) {
   out_5871344221582215751[0] = delta_x[0] + nom_x[0];
   out_5871344221582215751[1] = delta_x[1] + nom_x[1];
   out_5871344221582215751[2] = delta_x[2] + nom_x[2];
   out_5871344221582215751[3] = delta_x[3] + nom_x[3];
   out_5871344221582215751[4] = delta_x[4] + nom_x[4];
   out_5871344221582215751[5] = delta_x[5] + nom_x[5];
   out_5871344221582215751[6] = delta_x[6] + nom_x[6];
   out_5871344221582215751[7] = delta_x[7] + nom_x[7];
   out_5871344221582215751[8] = delta_x[8] + nom_x[8];
   out_5871344221582215751[9] = delta_x[9] + nom_x[9];
   out_5871344221582215751[10] = delta_x[10] + nom_x[10];
   out_5871344221582215751[11] = delta_x[11] + nom_x[11];
   out_5871344221582215751[12] = delta_x[12] + nom_x[12];
   out_5871344221582215751[13] = delta_x[13] + nom_x[13];
   out_5871344221582215751[14] = delta_x[14] + nom_x[14];
   out_5871344221582215751[15] = delta_x[15] + nom_x[15];
   out_5871344221582215751[16] = delta_x[16] + nom_x[16];
   out_5871344221582215751[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_241695378812077950) {
   out_241695378812077950[0] = -nom_x[0] + true_x[0];
   out_241695378812077950[1] = -nom_x[1] + true_x[1];
   out_241695378812077950[2] = -nom_x[2] + true_x[2];
   out_241695378812077950[3] = -nom_x[3] + true_x[3];
   out_241695378812077950[4] = -nom_x[4] + true_x[4];
   out_241695378812077950[5] = -nom_x[5] + true_x[5];
   out_241695378812077950[6] = -nom_x[6] + true_x[6];
   out_241695378812077950[7] = -nom_x[7] + true_x[7];
   out_241695378812077950[8] = -nom_x[8] + true_x[8];
   out_241695378812077950[9] = -nom_x[9] + true_x[9];
   out_241695378812077950[10] = -nom_x[10] + true_x[10];
   out_241695378812077950[11] = -nom_x[11] + true_x[11];
   out_241695378812077950[12] = -nom_x[12] + true_x[12];
   out_241695378812077950[13] = -nom_x[13] + true_x[13];
   out_241695378812077950[14] = -nom_x[14] + true_x[14];
   out_241695378812077950[15] = -nom_x[15] + true_x[15];
   out_241695378812077950[16] = -nom_x[16] + true_x[16];
   out_241695378812077950[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6164610465264093335) {
   out_6164610465264093335[0] = 1.0;
   out_6164610465264093335[1] = 0.0;
   out_6164610465264093335[2] = 0.0;
   out_6164610465264093335[3] = 0.0;
   out_6164610465264093335[4] = 0.0;
   out_6164610465264093335[5] = 0.0;
   out_6164610465264093335[6] = 0.0;
   out_6164610465264093335[7] = 0.0;
   out_6164610465264093335[8] = 0.0;
   out_6164610465264093335[9] = 0.0;
   out_6164610465264093335[10] = 0.0;
   out_6164610465264093335[11] = 0.0;
   out_6164610465264093335[12] = 0.0;
   out_6164610465264093335[13] = 0.0;
   out_6164610465264093335[14] = 0.0;
   out_6164610465264093335[15] = 0.0;
   out_6164610465264093335[16] = 0.0;
   out_6164610465264093335[17] = 0.0;
   out_6164610465264093335[18] = 0.0;
   out_6164610465264093335[19] = 1.0;
   out_6164610465264093335[20] = 0.0;
   out_6164610465264093335[21] = 0.0;
   out_6164610465264093335[22] = 0.0;
   out_6164610465264093335[23] = 0.0;
   out_6164610465264093335[24] = 0.0;
   out_6164610465264093335[25] = 0.0;
   out_6164610465264093335[26] = 0.0;
   out_6164610465264093335[27] = 0.0;
   out_6164610465264093335[28] = 0.0;
   out_6164610465264093335[29] = 0.0;
   out_6164610465264093335[30] = 0.0;
   out_6164610465264093335[31] = 0.0;
   out_6164610465264093335[32] = 0.0;
   out_6164610465264093335[33] = 0.0;
   out_6164610465264093335[34] = 0.0;
   out_6164610465264093335[35] = 0.0;
   out_6164610465264093335[36] = 0.0;
   out_6164610465264093335[37] = 0.0;
   out_6164610465264093335[38] = 1.0;
   out_6164610465264093335[39] = 0.0;
   out_6164610465264093335[40] = 0.0;
   out_6164610465264093335[41] = 0.0;
   out_6164610465264093335[42] = 0.0;
   out_6164610465264093335[43] = 0.0;
   out_6164610465264093335[44] = 0.0;
   out_6164610465264093335[45] = 0.0;
   out_6164610465264093335[46] = 0.0;
   out_6164610465264093335[47] = 0.0;
   out_6164610465264093335[48] = 0.0;
   out_6164610465264093335[49] = 0.0;
   out_6164610465264093335[50] = 0.0;
   out_6164610465264093335[51] = 0.0;
   out_6164610465264093335[52] = 0.0;
   out_6164610465264093335[53] = 0.0;
   out_6164610465264093335[54] = 0.0;
   out_6164610465264093335[55] = 0.0;
   out_6164610465264093335[56] = 0.0;
   out_6164610465264093335[57] = 1.0;
   out_6164610465264093335[58] = 0.0;
   out_6164610465264093335[59] = 0.0;
   out_6164610465264093335[60] = 0.0;
   out_6164610465264093335[61] = 0.0;
   out_6164610465264093335[62] = 0.0;
   out_6164610465264093335[63] = 0.0;
   out_6164610465264093335[64] = 0.0;
   out_6164610465264093335[65] = 0.0;
   out_6164610465264093335[66] = 0.0;
   out_6164610465264093335[67] = 0.0;
   out_6164610465264093335[68] = 0.0;
   out_6164610465264093335[69] = 0.0;
   out_6164610465264093335[70] = 0.0;
   out_6164610465264093335[71] = 0.0;
   out_6164610465264093335[72] = 0.0;
   out_6164610465264093335[73] = 0.0;
   out_6164610465264093335[74] = 0.0;
   out_6164610465264093335[75] = 0.0;
   out_6164610465264093335[76] = 1.0;
   out_6164610465264093335[77] = 0.0;
   out_6164610465264093335[78] = 0.0;
   out_6164610465264093335[79] = 0.0;
   out_6164610465264093335[80] = 0.0;
   out_6164610465264093335[81] = 0.0;
   out_6164610465264093335[82] = 0.0;
   out_6164610465264093335[83] = 0.0;
   out_6164610465264093335[84] = 0.0;
   out_6164610465264093335[85] = 0.0;
   out_6164610465264093335[86] = 0.0;
   out_6164610465264093335[87] = 0.0;
   out_6164610465264093335[88] = 0.0;
   out_6164610465264093335[89] = 0.0;
   out_6164610465264093335[90] = 0.0;
   out_6164610465264093335[91] = 0.0;
   out_6164610465264093335[92] = 0.0;
   out_6164610465264093335[93] = 0.0;
   out_6164610465264093335[94] = 0.0;
   out_6164610465264093335[95] = 1.0;
   out_6164610465264093335[96] = 0.0;
   out_6164610465264093335[97] = 0.0;
   out_6164610465264093335[98] = 0.0;
   out_6164610465264093335[99] = 0.0;
   out_6164610465264093335[100] = 0.0;
   out_6164610465264093335[101] = 0.0;
   out_6164610465264093335[102] = 0.0;
   out_6164610465264093335[103] = 0.0;
   out_6164610465264093335[104] = 0.0;
   out_6164610465264093335[105] = 0.0;
   out_6164610465264093335[106] = 0.0;
   out_6164610465264093335[107] = 0.0;
   out_6164610465264093335[108] = 0.0;
   out_6164610465264093335[109] = 0.0;
   out_6164610465264093335[110] = 0.0;
   out_6164610465264093335[111] = 0.0;
   out_6164610465264093335[112] = 0.0;
   out_6164610465264093335[113] = 0.0;
   out_6164610465264093335[114] = 1.0;
   out_6164610465264093335[115] = 0.0;
   out_6164610465264093335[116] = 0.0;
   out_6164610465264093335[117] = 0.0;
   out_6164610465264093335[118] = 0.0;
   out_6164610465264093335[119] = 0.0;
   out_6164610465264093335[120] = 0.0;
   out_6164610465264093335[121] = 0.0;
   out_6164610465264093335[122] = 0.0;
   out_6164610465264093335[123] = 0.0;
   out_6164610465264093335[124] = 0.0;
   out_6164610465264093335[125] = 0.0;
   out_6164610465264093335[126] = 0.0;
   out_6164610465264093335[127] = 0.0;
   out_6164610465264093335[128] = 0.0;
   out_6164610465264093335[129] = 0.0;
   out_6164610465264093335[130] = 0.0;
   out_6164610465264093335[131] = 0.0;
   out_6164610465264093335[132] = 0.0;
   out_6164610465264093335[133] = 1.0;
   out_6164610465264093335[134] = 0.0;
   out_6164610465264093335[135] = 0.0;
   out_6164610465264093335[136] = 0.0;
   out_6164610465264093335[137] = 0.0;
   out_6164610465264093335[138] = 0.0;
   out_6164610465264093335[139] = 0.0;
   out_6164610465264093335[140] = 0.0;
   out_6164610465264093335[141] = 0.0;
   out_6164610465264093335[142] = 0.0;
   out_6164610465264093335[143] = 0.0;
   out_6164610465264093335[144] = 0.0;
   out_6164610465264093335[145] = 0.0;
   out_6164610465264093335[146] = 0.0;
   out_6164610465264093335[147] = 0.0;
   out_6164610465264093335[148] = 0.0;
   out_6164610465264093335[149] = 0.0;
   out_6164610465264093335[150] = 0.0;
   out_6164610465264093335[151] = 0.0;
   out_6164610465264093335[152] = 1.0;
   out_6164610465264093335[153] = 0.0;
   out_6164610465264093335[154] = 0.0;
   out_6164610465264093335[155] = 0.0;
   out_6164610465264093335[156] = 0.0;
   out_6164610465264093335[157] = 0.0;
   out_6164610465264093335[158] = 0.0;
   out_6164610465264093335[159] = 0.0;
   out_6164610465264093335[160] = 0.0;
   out_6164610465264093335[161] = 0.0;
   out_6164610465264093335[162] = 0.0;
   out_6164610465264093335[163] = 0.0;
   out_6164610465264093335[164] = 0.0;
   out_6164610465264093335[165] = 0.0;
   out_6164610465264093335[166] = 0.0;
   out_6164610465264093335[167] = 0.0;
   out_6164610465264093335[168] = 0.0;
   out_6164610465264093335[169] = 0.0;
   out_6164610465264093335[170] = 0.0;
   out_6164610465264093335[171] = 1.0;
   out_6164610465264093335[172] = 0.0;
   out_6164610465264093335[173] = 0.0;
   out_6164610465264093335[174] = 0.0;
   out_6164610465264093335[175] = 0.0;
   out_6164610465264093335[176] = 0.0;
   out_6164610465264093335[177] = 0.0;
   out_6164610465264093335[178] = 0.0;
   out_6164610465264093335[179] = 0.0;
   out_6164610465264093335[180] = 0.0;
   out_6164610465264093335[181] = 0.0;
   out_6164610465264093335[182] = 0.0;
   out_6164610465264093335[183] = 0.0;
   out_6164610465264093335[184] = 0.0;
   out_6164610465264093335[185] = 0.0;
   out_6164610465264093335[186] = 0.0;
   out_6164610465264093335[187] = 0.0;
   out_6164610465264093335[188] = 0.0;
   out_6164610465264093335[189] = 0.0;
   out_6164610465264093335[190] = 1.0;
   out_6164610465264093335[191] = 0.0;
   out_6164610465264093335[192] = 0.0;
   out_6164610465264093335[193] = 0.0;
   out_6164610465264093335[194] = 0.0;
   out_6164610465264093335[195] = 0.0;
   out_6164610465264093335[196] = 0.0;
   out_6164610465264093335[197] = 0.0;
   out_6164610465264093335[198] = 0.0;
   out_6164610465264093335[199] = 0.0;
   out_6164610465264093335[200] = 0.0;
   out_6164610465264093335[201] = 0.0;
   out_6164610465264093335[202] = 0.0;
   out_6164610465264093335[203] = 0.0;
   out_6164610465264093335[204] = 0.0;
   out_6164610465264093335[205] = 0.0;
   out_6164610465264093335[206] = 0.0;
   out_6164610465264093335[207] = 0.0;
   out_6164610465264093335[208] = 0.0;
   out_6164610465264093335[209] = 1.0;
   out_6164610465264093335[210] = 0.0;
   out_6164610465264093335[211] = 0.0;
   out_6164610465264093335[212] = 0.0;
   out_6164610465264093335[213] = 0.0;
   out_6164610465264093335[214] = 0.0;
   out_6164610465264093335[215] = 0.0;
   out_6164610465264093335[216] = 0.0;
   out_6164610465264093335[217] = 0.0;
   out_6164610465264093335[218] = 0.0;
   out_6164610465264093335[219] = 0.0;
   out_6164610465264093335[220] = 0.0;
   out_6164610465264093335[221] = 0.0;
   out_6164610465264093335[222] = 0.0;
   out_6164610465264093335[223] = 0.0;
   out_6164610465264093335[224] = 0.0;
   out_6164610465264093335[225] = 0.0;
   out_6164610465264093335[226] = 0.0;
   out_6164610465264093335[227] = 0.0;
   out_6164610465264093335[228] = 1.0;
   out_6164610465264093335[229] = 0.0;
   out_6164610465264093335[230] = 0.0;
   out_6164610465264093335[231] = 0.0;
   out_6164610465264093335[232] = 0.0;
   out_6164610465264093335[233] = 0.0;
   out_6164610465264093335[234] = 0.0;
   out_6164610465264093335[235] = 0.0;
   out_6164610465264093335[236] = 0.0;
   out_6164610465264093335[237] = 0.0;
   out_6164610465264093335[238] = 0.0;
   out_6164610465264093335[239] = 0.0;
   out_6164610465264093335[240] = 0.0;
   out_6164610465264093335[241] = 0.0;
   out_6164610465264093335[242] = 0.0;
   out_6164610465264093335[243] = 0.0;
   out_6164610465264093335[244] = 0.0;
   out_6164610465264093335[245] = 0.0;
   out_6164610465264093335[246] = 0.0;
   out_6164610465264093335[247] = 1.0;
   out_6164610465264093335[248] = 0.0;
   out_6164610465264093335[249] = 0.0;
   out_6164610465264093335[250] = 0.0;
   out_6164610465264093335[251] = 0.0;
   out_6164610465264093335[252] = 0.0;
   out_6164610465264093335[253] = 0.0;
   out_6164610465264093335[254] = 0.0;
   out_6164610465264093335[255] = 0.0;
   out_6164610465264093335[256] = 0.0;
   out_6164610465264093335[257] = 0.0;
   out_6164610465264093335[258] = 0.0;
   out_6164610465264093335[259] = 0.0;
   out_6164610465264093335[260] = 0.0;
   out_6164610465264093335[261] = 0.0;
   out_6164610465264093335[262] = 0.0;
   out_6164610465264093335[263] = 0.0;
   out_6164610465264093335[264] = 0.0;
   out_6164610465264093335[265] = 0.0;
   out_6164610465264093335[266] = 1.0;
   out_6164610465264093335[267] = 0.0;
   out_6164610465264093335[268] = 0.0;
   out_6164610465264093335[269] = 0.0;
   out_6164610465264093335[270] = 0.0;
   out_6164610465264093335[271] = 0.0;
   out_6164610465264093335[272] = 0.0;
   out_6164610465264093335[273] = 0.0;
   out_6164610465264093335[274] = 0.0;
   out_6164610465264093335[275] = 0.0;
   out_6164610465264093335[276] = 0.0;
   out_6164610465264093335[277] = 0.0;
   out_6164610465264093335[278] = 0.0;
   out_6164610465264093335[279] = 0.0;
   out_6164610465264093335[280] = 0.0;
   out_6164610465264093335[281] = 0.0;
   out_6164610465264093335[282] = 0.0;
   out_6164610465264093335[283] = 0.0;
   out_6164610465264093335[284] = 0.0;
   out_6164610465264093335[285] = 1.0;
   out_6164610465264093335[286] = 0.0;
   out_6164610465264093335[287] = 0.0;
   out_6164610465264093335[288] = 0.0;
   out_6164610465264093335[289] = 0.0;
   out_6164610465264093335[290] = 0.0;
   out_6164610465264093335[291] = 0.0;
   out_6164610465264093335[292] = 0.0;
   out_6164610465264093335[293] = 0.0;
   out_6164610465264093335[294] = 0.0;
   out_6164610465264093335[295] = 0.0;
   out_6164610465264093335[296] = 0.0;
   out_6164610465264093335[297] = 0.0;
   out_6164610465264093335[298] = 0.0;
   out_6164610465264093335[299] = 0.0;
   out_6164610465264093335[300] = 0.0;
   out_6164610465264093335[301] = 0.0;
   out_6164610465264093335[302] = 0.0;
   out_6164610465264093335[303] = 0.0;
   out_6164610465264093335[304] = 1.0;
   out_6164610465264093335[305] = 0.0;
   out_6164610465264093335[306] = 0.0;
   out_6164610465264093335[307] = 0.0;
   out_6164610465264093335[308] = 0.0;
   out_6164610465264093335[309] = 0.0;
   out_6164610465264093335[310] = 0.0;
   out_6164610465264093335[311] = 0.0;
   out_6164610465264093335[312] = 0.0;
   out_6164610465264093335[313] = 0.0;
   out_6164610465264093335[314] = 0.0;
   out_6164610465264093335[315] = 0.0;
   out_6164610465264093335[316] = 0.0;
   out_6164610465264093335[317] = 0.0;
   out_6164610465264093335[318] = 0.0;
   out_6164610465264093335[319] = 0.0;
   out_6164610465264093335[320] = 0.0;
   out_6164610465264093335[321] = 0.0;
   out_6164610465264093335[322] = 0.0;
   out_6164610465264093335[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1969944452673306282) {
   out_1969944452673306282[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1969944452673306282[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1969944452673306282[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1969944452673306282[3] = dt*state[12] + state[3];
   out_1969944452673306282[4] = dt*state[13] + state[4];
   out_1969944452673306282[5] = dt*state[14] + state[5];
   out_1969944452673306282[6] = state[6];
   out_1969944452673306282[7] = state[7];
   out_1969944452673306282[8] = state[8];
   out_1969944452673306282[9] = state[9];
   out_1969944452673306282[10] = state[10];
   out_1969944452673306282[11] = state[11];
   out_1969944452673306282[12] = state[12];
   out_1969944452673306282[13] = state[13];
   out_1969944452673306282[14] = state[14];
   out_1969944452673306282[15] = state[15];
   out_1969944452673306282[16] = state[16];
   out_1969944452673306282[17] = state[17];
}
void F_fun(double *state, double dt, double *out_381226404886081050) {
   out_381226404886081050[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_381226404886081050[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_381226404886081050[2] = 0;
   out_381226404886081050[3] = 0;
   out_381226404886081050[4] = 0;
   out_381226404886081050[5] = 0;
   out_381226404886081050[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_381226404886081050[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_381226404886081050[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_381226404886081050[9] = 0;
   out_381226404886081050[10] = 0;
   out_381226404886081050[11] = 0;
   out_381226404886081050[12] = 0;
   out_381226404886081050[13] = 0;
   out_381226404886081050[14] = 0;
   out_381226404886081050[15] = 0;
   out_381226404886081050[16] = 0;
   out_381226404886081050[17] = 0;
   out_381226404886081050[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_381226404886081050[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_381226404886081050[20] = 0;
   out_381226404886081050[21] = 0;
   out_381226404886081050[22] = 0;
   out_381226404886081050[23] = 0;
   out_381226404886081050[24] = 0;
   out_381226404886081050[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_381226404886081050[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_381226404886081050[27] = 0;
   out_381226404886081050[28] = 0;
   out_381226404886081050[29] = 0;
   out_381226404886081050[30] = 0;
   out_381226404886081050[31] = 0;
   out_381226404886081050[32] = 0;
   out_381226404886081050[33] = 0;
   out_381226404886081050[34] = 0;
   out_381226404886081050[35] = 0;
   out_381226404886081050[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_381226404886081050[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_381226404886081050[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_381226404886081050[39] = 0;
   out_381226404886081050[40] = 0;
   out_381226404886081050[41] = 0;
   out_381226404886081050[42] = 0;
   out_381226404886081050[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_381226404886081050[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_381226404886081050[45] = 0;
   out_381226404886081050[46] = 0;
   out_381226404886081050[47] = 0;
   out_381226404886081050[48] = 0;
   out_381226404886081050[49] = 0;
   out_381226404886081050[50] = 0;
   out_381226404886081050[51] = 0;
   out_381226404886081050[52] = 0;
   out_381226404886081050[53] = 0;
   out_381226404886081050[54] = 0;
   out_381226404886081050[55] = 0;
   out_381226404886081050[56] = 0;
   out_381226404886081050[57] = 1;
   out_381226404886081050[58] = 0;
   out_381226404886081050[59] = 0;
   out_381226404886081050[60] = 0;
   out_381226404886081050[61] = 0;
   out_381226404886081050[62] = 0;
   out_381226404886081050[63] = 0;
   out_381226404886081050[64] = 0;
   out_381226404886081050[65] = 0;
   out_381226404886081050[66] = dt;
   out_381226404886081050[67] = 0;
   out_381226404886081050[68] = 0;
   out_381226404886081050[69] = 0;
   out_381226404886081050[70] = 0;
   out_381226404886081050[71] = 0;
   out_381226404886081050[72] = 0;
   out_381226404886081050[73] = 0;
   out_381226404886081050[74] = 0;
   out_381226404886081050[75] = 0;
   out_381226404886081050[76] = 1;
   out_381226404886081050[77] = 0;
   out_381226404886081050[78] = 0;
   out_381226404886081050[79] = 0;
   out_381226404886081050[80] = 0;
   out_381226404886081050[81] = 0;
   out_381226404886081050[82] = 0;
   out_381226404886081050[83] = 0;
   out_381226404886081050[84] = 0;
   out_381226404886081050[85] = dt;
   out_381226404886081050[86] = 0;
   out_381226404886081050[87] = 0;
   out_381226404886081050[88] = 0;
   out_381226404886081050[89] = 0;
   out_381226404886081050[90] = 0;
   out_381226404886081050[91] = 0;
   out_381226404886081050[92] = 0;
   out_381226404886081050[93] = 0;
   out_381226404886081050[94] = 0;
   out_381226404886081050[95] = 1;
   out_381226404886081050[96] = 0;
   out_381226404886081050[97] = 0;
   out_381226404886081050[98] = 0;
   out_381226404886081050[99] = 0;
   out_381226404886081050[100] = 0;
   out_381226404886081050[101] = 0;
   out_381226404886081050[102] = 0;
   out_381226404886081050[103] = 0;
   out_381226404886081050[104] = dt;
   out_381226404886081050[105] = 0;
   out_381226404886081050[106] = 0;
   out_381226404886081050[107] = 0;
   out_381226404886081050[108] = 0;
   out_381226404886081050[109] = 0;
   out_381226404886081050[110] = 0;
   out_381226404886081050[111] = 0;
   out_381226404886081050[112] = 0;
   out_381226404886081050[113] = 0;
   out_381226404886081050[114] = 1;
   out_381226404886081050[115] = 0;
   out_381226404886081050[116] = 0;
   out_381226404886081050[117] = 0;
   out_381226404886081050[118] = 0;
   out_381226404886081050[119] = 0;
   out_381226404886081050[120] = 0;
   out_381226404886081050[121] = 0;
   out_381226404886081050[122] = 0;
   out_381226404886081050[123] = 0;
   out_381226404886081050[124] = 0;
   out_381226404886081050[125] = 0;
   out_381226404886081050[126] = 0;
   out_381226404886081050[127] = 0;
   out_381226404886081050[128] = 0;
   out_381226404886081050[129] = 0;
   out_381226404886081050[130] = 0;
   out_381226404886081050[131] = 0;
   out_381226404886081050[132] = 0;
   out_381226404886081050[133] = 1;
   out_381226404886081050[134] = 0;
   out_381226404886081050[135] = 0;
   out_381226404886081050[136] = 0;
   out_381226404886081050[137] = 0;
   out_381226404886081050[138] = 0;
   out_381226404886081050[139] = 0;
   out_381226404886081050[140] = 0;
   out_381226404886081050[141] = 0;
   out_381226404886081050[142] = 0;
   out_381226404886081050[143] = 0;
   out_381226404886081050[144] = 0;
   out_381226404886081050[145] = 0;
   out_381226404886081050[146] = 0;
   out_381226404886081050[147] = 0;
   out_381226404886081050[148] = 0;
   out_381226404886081050[149] = 0;
   out_381226404886081050[150] = 0;
   out_381226404886081050[151] = 0;
   out_381226404886081050[152] = 1;
   out_381226404886081050[153] = 0;
   out_381226404886081050[154] = 0;
   out_381226404886081050[155] = 0;
   out_381226404886081050[156] = 0;
   out_381226404886081050[157] = 0;
   out_381226404886081050[158] = 0;
   out_381226404886081050[159] = 0;
   out_381226404886081050[160] = 0;
   out_381226404886081050[161] = 0;
   out_381226404886081050[162] = 0;
   out_381226404886081050[163] = 0;
   out_381226404886081050[164] = 0;
   out_381226404886081050[165] = 0;
   out_381226404886081050[166] = 0;
   out_381226404886081050[167] = 0;
   out_381226404886081050[168] = 0;
   out_381226404886081050[169] = 0;
   out_381226404886081050[170] = 0;
   out_381226404886081050[171] = 1;
   out_381226404886081050[172] = 0;
   out_381226404886081050[173] = 0;
   out_381226404886081050[174] = 0;
   out_381226404886081050[175] = 0;
   out_381226404886081050[176] = 0;
   out_381226404886081050[177] = 0;
   out_381226404886081050[178] = 0;
   out_381226404886081050[179] = 0;
   out_381226404886081050[180] = 0;
   out_381226404886081050[181] = 0;
   out_381226404886081050[182] = 0;
   out_381226404886081050[183] = 0;
   out_381226404886081050[184] = 0;
   out_381226404886081050[185] = 0;
   out_381226404886081050[186] = 0;
   out_381226404886081050[187] = 0;
   out_381226404886081050[188] = 0;
   out_381226404886081050[189] = 0;
   out_381226404886081050[190] = 1;
   out_381226404886081050[191] = 0;
   out_381226404886081050[192] = 0;
   out_381226404886081050[193] = 0;
   out_381226404886081050[194] = 0;
   out_381226404886081050[195] = 0;
   out_381226404886081050[196] = 0;
   out_381226404886081050[197] = 0;
   out_381226404886081050[198] = 0;
   out_381226404886081050[199] = 0;
   out_381226404886081050[200] = 0;
   out_381226404886081050[201] = 0;
   out_381226404886081050[202] = 0;
   out_381226404886081050[203] = 0;
   out_381226404886081050[204] = 0;
   out_381226404886081050[205] = 0;
   out_381226404886081050[206] = 0;
   out_381226404886081050[207] = 0;
   out_381226404886081050[208] = 0;
   out_381226404886081050[209] = 1;
   out_381226404886081050[210] = 0;
   out_381226404886081050[211] = 0;
   out_381226404886081050[212] = 0;
   out_381226404886081050[213] = 0;
   out_381226404886081050[214] = 0;
   out_381226404886081050[215] = 0;
   out_381226404886081050[216] = 0;
   out_381226404886081050[217] = 0;
   out_381226404886081050[218] = 0;
   out_381226404886081050[219] = 0;
   out_381226404886081050[220] = 0;
   out_381226404886081050[221] = 0;
   out_381226404886081050[222] = 0;
   out_381226404886081050[223] = 0;
   out_381226404886081050[224] = 0;
   out_381226404886081050[225] = 0;
   out_381226404886081050[226] = 0;
   out_381226404886081050[227] = 0;
   out_381226404886081050[228] = 1;
   out_381226404886081050[229] = 0;
   out_381226404886081050[230] = 0;
   out_381226404886081050[231] = 0;
   out_381226404886081050[232] = 0;
   out_381226404886081050[233] = 0;
   out_381226404886081050[234] = 0;
   out_381226404886081050[235] = 0;
   out_381226404886081050[236] = 0;
   out_381226404886081050[237] = 0;
   out_381226404886081050[238] = 0;
   out_381226404886081050[239] = 0;
   out_381226404886081050[240] = 0;
   out_381226404886081050[241] = 0;
   out_381226404886081050[242] = 0;
   out_381226404886081050[243] = 0;
   out_381226404886081050[244] = 0;
   out_381226404886081050[245] = 0;
   out_381226404886081050[246] = 0;
   out_381226404886081050[247] = 1;
   out_381226404886081050[248] = 0;
   out_381226404886081050[249] = 0;
   out_381226404886081050[250] = 0;
   out_381226404886081050[251] = 0;
   out_381226404886081050[252] = 0;
   out_381226404886081050[253] = 0;
   out_381226404886081050[254] = 0;
   out_381226404886081050[255] = 0;
   out_381226404886081050[256] = 0;
   out_381226404886081050[257] = 0;
   out_381226404886081050[258] = 0;
   out_381226404886081050[259] = 0;
   out_381226404886081050[260] = 0;
   out_381226404886081050[261] = 0;
   out_381226404886081050[262] = 0;
   out_381226404886081050[263] = 0;
   out_381226404886081050[264] = 0;
   out_381226404886081050[265] = 0;
   out_381226404886081050[266] = 1;
   out_381226404886081050[267] = 0;
   out_381226404886081050[268] = 0;
   out_381226404886081050[269] = 0;
   out_381226404886081050[270] = 0;
   out_381226404886081050[271] = 0;
   out_381226404886081050[272] = 0;
   out_381226404886081050[273] = 0;
   out_381226404886081050[274] = 0;
   out_381226404886081050[275] = 0;
   out_381226404886081050[276] = 0;
   out_381226404886081050[277] = 0;
   out_381226404886081050[278] = 0;
   out_381226404886081050[279] = 0;
   out_381226404886081050[280] = 0;
   out_381226404886081050[281] = 0;
   out_381226404886081050[282] = 0;
   out_381226404886081050[283] = 0;
   out_381226404886081050[284] = 0;
   out_381226404886081050[285] = 1;
   out_381226404886081050[286] = 0;
   out_381226404886081050[287] = 0;
   out_381226404886081050[288] = 0;
   out_381226404886081050[289] = 0;
   out_381226404886081050[290] = 0;
   out_381226404886081050[291] = 0;
   out_381226404886081050[292] = 0;
   out_381226404886081050[293] = 0;
   out_381226404886081050[294] = 0;
   out_381226404886081050[295] = 0;
   out_381226404886081050[296] = 0;
   out_381226404886081050[297] = 0;
   out_381226404886081050[298] = 0;
   out_381226404886081050[299] = 0;
   out_381226404886081050[300] = 0;
   out_381226404886081050[301] = 0;
   out_381226404886081050[302] = 0;
   out_381226404886081050[303] = 0;
   out_381226404886081050[304] = 1;
   out_381226404886081050[305] = 0;
   out_381226404886081050[306] = 0;
   out_381226404886081050[307] = 0;
   out_381226404886081050[308] = 0;
   out_381226404886081050[309] = 0;
   out_381226404886081050[310] = 0;
   out_381226404886081050[311] = 0;
   out_381226404886081050[312] = 0;
   out_381226404886081050[313] = 0;
   out_381226404886081050[314] = 0;
   out_381226404886081050[315] = 0;
   out_381226404886081050[316] = 0;
   out_381226404886081050[317] = 0;
   out_381226404886081050[318] = 0;
   out_381226404886081050[319] = 0;
   out_381226404886081050[320] = 0;
   out_381226404886081050[321] = 0;
   out_381226404886081050[322] = 0;
   out_381226404886081050[323] = 1;
}
void h_4(double *state, double *unused, double *out_6288155756564553297) {
   out_6288155756564553297[0] = state[6] + state[9];
   out_6288155756564553297[1] = state[7] + state[10];
   out_6288155756564553297[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5410449463209380728) {
   out_5410449463209380728[0] = 0;
   out_5410449463209380728[1] = 0;
   out_5410449463209380728[2] = 0;
   out_5410449463209380728[3] = 0;
   out_5410449463209380728[4] = 0;
   out_5410449463209380728[5] = 0;
   out_5410449463209380728[6] = 1;
   out_5410449463209380728[7] = 0;
   out_5410449463209380728[8] = 0;
   out_5410449463209380728[9] = 1;
   out_5410449463209380728[10] = 0;
   out_5410449463209380728[11] = 0;
   out_5410449463209380728[12] = 0;
   out_5410449463209380728[13] = 0;
   out_5410449463209380728[14] = 0;
   out_5410449463209380728[15] = 0;
   out_5410449463209380728[16] = 0;
   out_5410449463209380728[17] = 0;
   out_5410449463209380728[18] = 0;
   out_5410449463209380728[19] = 0;
   out_5410449463209380728[20] = 0;
   out_5410449463209380728[21] = 0;
   out_5410449463209380728[22] = 0;
   out_5410449463209380728[23] = 0;
   out_5410449463209380728[24] = 0;
   out_5410449463209380728[25] = 1;
   out_5410449463209380728[26] = 0;
   out_5410449463209380728[27] = 0;
   out_5410449463209380728[28] = 1;
   out_5410449463209380728[29] = 0;
   out_5410449463209380728[30] = 0;
   out_5410449463209380728[31] = 0;
   out_5410449463209380728[32] = 0;
   out_5410449463209380728[33] = 0;
   out_5410449463209380728[34] = 0;
   out_5410449463209380728[35] = 0;
   out_5410449463209380728[36] = 0;
   out_5410449463209380728[37] = 0;
   out_5410449463209380728[38] = 0;
   out_5410449463209380728[39] = 0;
   out_5410449463209380728[40] = 0;
   out_5410449463209380728[41] = 0;
   out_5410449463209380728[42] = 0;
   out_5410449463209380728[43] = 0;
   out_5410449463209380728[44] = 1;
   out_5410449463209380728[45] = 0;
   out_5410449463209380728[46] = 0;
   out_5410449463209380728[47] = 1;
   out_5410449463209380728[48] = 0;
   out_5410449463209380728[49] = 0;
   out_5410449463209380728[50] = 0;
   out_5410449463209380728[51] = 0;
   out_5410449463209380728[52] = 0;
   out_5410449463209380728[53] = 0;
}
void h_10(double *state, double *unused, double *out_8113307794108590847) {
   out_8113307794108590847[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8113307794108590847[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8113307794108590847[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_781315721660582811) {
   out_781315721660582811[0] = 0;
   out_781315721660582811[1] = 9.8100000000000005*cos(state[1]);
   out_781315721660582811[2] = 0;
   out_781315721660582811[3] = 0;
   out_781315721660582811[4] = -state[8];
   out_781315721660582811[5] = state[7];
   out_781315721660582811[6] = 0;
   out_781315721660582811[7] = state[5];
   out_781315721660582811[8] = -state[4];
   out_781315721660582811[9] = 0;
   out_781315721660582811[10] = 0;
   out_781315721660582811[11] = 0;
   out_781315721660582811[12] = 1;
   out_781315721660582811[13] = 0;
   out_781315721660582811[14] = 0;
   out_781315721660582811[15] = 1;
   out_781315721660582811[16] = 0;
   out_781315721660582811[17] = 0;
   out_781315721660582811[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_781315721660582811[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_781315721660582811[20] = 0;
   out_781315721660582811[21] = state[8];
   out_781315721660582811[22] = 0;
   out_781315721660582811[23] = -state[6];
   out_781315721660582811[24] = -state[5];
   out_781315721660582811[25] = 0;
   out_781315721660582811[26] = state[3];
   out_781315721660582811[27] = 0;
   out_781315721660582811[28] = 0;
   out_781315721660582811[29] = 0;
   out_781315721660582811[30] = 0;
   out_781315721660582811[31] = 1;
   out_781315721660582811[32] = 0;
   out_781315721660582811[33] = 0;
   out_781315721660582811[34] = 1;
   out_781315721660582811[35] = 0;
   out_781315721660582811[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_781315721660582811[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_781315721660582811[38] = 0;
   out_781315721660582811[39] = -state[7];
   out_781315721660582811[40] = state[6];
   out_781315721660582811[41] = 0;
   out_781315721660582811[42] = state[4];
   out_781315721660582811[43] = -state[3];
   out_781315721660582811[44] = 0;
   out_781315721660582811[45] = 0;
   out_781315721660582811[46] = 0;
   out_781315721660582811[47] = 0;
   out_781315721660582811[48] = 0;
   out_781315721660582811[49] = 0;
   out_781315721660582811[50] = 1;
   out_781315721660582811[51] = 0;
   out_781315721660582811[52] = 0;
   out_781315721660582811[53] = 1;
}
void h_13(double *state, double *unused, double *out_5617290323579445144) {
   out_5617290323579445144[0] = state[3];
   out_5617290323579445144[1] = state[4];
   out_5617290323579445144[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2200181745107320201) {
   out_2200181745107320201[0] = 0;
   out_2200181745107320201[1] = 0;
   out_2200181745107320201[2] = 0;
   out_2200181745107320201[3] = 1;
   out_2200181745107320201[4] = 0;
   out_2200181745107320201[5] = 0;
   out_2200181745107320201[6] = 0;
   out_2200181745107320201[7] = 0;
   out_2200181745107320201[8] = 0;
   out_2200181745107320201[9] = 0;
   out_2200181745107320201[10] = 0;
   out_2200181745107320201[11] = 0;
   out_2200181745107320201[12] = 0;
   out_2200181745107320201[13] = 0;
   out_2200181745107320201[14] = 0;
   out_2200181745107320201[15] = 0;
   out_2200181745107320201[16] = 0;
   out_2200181745107320201[17] = 0;
   out_2200181745107320201[18] = 0;
   out_2200181745107320201[19] = 0;
   out_2200181745107320201[20] = 0;
   out_2200181745107320201[21] = 0;
   out_2200181745107320201[22] = 1;
   out_2200181745107320201[23] = 0;
   out_2200181745107320201[24] = 0;
   out_2200181745107320201[25] = 0;
   out_2200181745107320201[26] = 0;
   out_2200181745107320201[27] = 0;
   out_2200181745107320201[28] = 0;
   out_2200181745107320201[29] = 0;
   out_2200181745107320201[30] = 0;
   out_2200181745107320201[31] = 0;
   out_2200181745107320201[32] = 0;
   out_2200181745107320201[33] = 0;
   out_2200181745107320201[34] = 0;
   out_2200181745107320201[35] = 0;
   out_2200181745107320201[36] = 0;
   out_2200181745107320201[37] = 0;
   out_2200181745107320201[38] = 0;
   out_2200181745107320201[39] = 0;
   out_2200181745107320201[40] = 0;
   out_2200181745107320201[41] = 1;
   out_2200181745107320201[42] = 0;
   out_2200181745107320201[43] = 0;
   out_2200181745107320201[44] = 0;
   out_2200181745107320201[45] = 0;
   out_2200181745107320201[46] = 0;
   out_2200181745107320201[47] = 0;
   out_2200181745107320201[48] = 0;
   out_2200181745107320201[49] = 0;
   out_2200181745107320201[50] = 0;
   out_2200181745107320201[51] = 0;
   out_2200181745107320201[52] = 0;
   out_2200181745107320201[53] = 0;
}
void h_14(double *state, double *unused, double *out_8750371077921366803) {
   out_8750371077921366803[0] = state[6];
   out_8750371077921366803[1] = state[7];
   out_8750371077921366803[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1447208606869896199) {
   out_1447208606869896199[0] = 0;
   out_1447208606869896199[1] = 0;
   out_1447208606869896199[2] = 0;
   out_1447208606869896199[3] = 0;
   out_1447208606869896199[4] = 0;
   out_1447208606869896199[5] = 0;
   out_1447208606869896199[6] = 1;
   out_1447208606869896199[7] = 0;
   out_1447208606869896199[8] = 0;
   out_1447208606869896199[9] = 0;
   out_1447208606869896199[10] = 0;
   out_1447208606869896199[11] = 0;
   out_1447208606869896199[12] = 0;
   out_1447208606869896199[13] = 0;
   out_1447208606869896199[14] = 0;
   out_1447208606869896199[15] = 0;
   out_1447208606869896199[16] = 0;
   out_1447208606869896199[17] = 0;
   out_1447208606869896199[18] = 0;
   out_1447208606869896199[19] = 0;
   out_1447208606869896199[20] = 0;
   out_1447208606869896199[21] = 0;
   out_1447208606869896199[22] = 0;
   out_1447208606869896199[23] = 0;
   out_1447208606869896199[24] = 0;
   out_1447208606869896199[25] = 1;
   out_1447208606869896199[26] = 0;
   out_1447208606869896199[27] = 0;
   out_1447208606869896199[28] = 0;
   out_1447208606869896199[29] = 0;
   out_1447208606869896199[30] = 0;
   out_1447208606869896199[31] = 0;
   out_1447208606869896199[32] = 0;
   out_1447208606869896199[33] = 0;
   out_1447208606869896199[34] = 0;
   out_1447208606869896199[35] = 0;
   out_1447208606869896199[36] = 0;
   out_1447208606869896199[37] = 0;
   out_1447208606869896199[38] = 0;
   out_1447208606869896199[39] = 0;
   out_1447208606869896199[40] = 0;
   out_1447208606869896199[41] = 0;
   out_1447208606869896199[42] = 0;
   out_1447208606869896199[43] = 0;
   out_1447208606869896199[44] = 1;
   out_1447208606869896199[45] = 0;
   out_1447208606869896199[46] = 0;
   out_1447208606869896199[47] = 0;
   out_1447208606869896199[48] = 0;
   out_1447208606869896199[49] = 0;
   out_1447208606869896199[50] = 0;
   out_1447208606869896199[51] = 0;
   out_1447208606869896199[52] = 0;
   out_1447208606869896199[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_5871344221582215751) {
  err_fun(nom_x, delta_x, out_5871344221582215751);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_241695378812077950) {
  inv_err_fun(nom_x, true_x, out_241695378812077950);
}
void pose_H_mod_fun(double *state, double *out_6164610465264093335) {
  H_mod_fun(state, out_6164610465264093335);
}
void pose_f_fun(double *state, double dt, double *out_1969944452673306282) {
  f_fun(state,  dt, out_1969944452673306282);
}
void pose_F_fun(double *state, double dt, double *out_381226404886081050) {
  F_fun(state,  dt, out_381226404886081050);
}
void pose_h_4(double *state, double *unused, double *out_6288155756564553297) {
  h_4(state, unused, out_6288155756564553297);
}
void pose_H_4(double *state, double *unused, double *out_5410449463209380728) {
  H_4(state, unused, out_5410449463209380728);
}
void pose_h_10(double *state, double *unused, double *out_8113307794108590847) {
  h_10(state, unused, out_8113307794108590847);
}
void pose_H_10(double *state, double *unused, double *out_781315721660582811) {
  H_10(state, unused, out_781315721660582811);
}
void pose_h_13(double *state, double *unused, double *out_5617290323579445144) {
  h_13(state, unused, out_5617290323579445144);
}
void pose_H_13(double *state, double *unused, double *out_2200181745107320201) {
  H_13(state, unused, out_2200181745107320201);
}
void pose_h_14(double *state, double *unused, double *out_8750371077921366803) {
  h_14(state, unused, out_8750371077921366803);
}
void pose_H_14(double *state, double *unused, double *out_1447208606869896199) {
  H_14(state, unused, out_1447208606869896199);
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
