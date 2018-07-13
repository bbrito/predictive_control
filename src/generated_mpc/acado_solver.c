/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[4] = acadoVariables.mu[lRun1 * 4];
acadoWorkspace.state[5] = acadoVariables.mu[lRun1 * 4 + 1];
acadoWorkspace.state[6] = acadoVariables.mu[lRun1 * 4 + 2];
acadoWorkspace.state[7] = acadoVariables.mu[lRun1 * 4 + 3];
acadoWorkspace.state[64] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[65] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[66] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.state[67] = acadoVariables.od[lRun1 * 25];
acadoWorkspace.state[68] = acadoVariables.od[lRun1 * 25 + 1];
acadoWorkspace.state[69] = acadoVariables.od[lRun1 * 25 + 2];
acadoWorkspace.state[70] = acadoVariables.od[lRun1 * 25 + 3];
acadoWorkspace.state[71] = acadoVariables.od[lRun1 * 25 + 4];
acadoWorkspace.state[72] = acadoVariables.od[lRun1 * 25 + 5];
acadoWorkspace.state[73] = acadoVariables.od[lRun1 * 25 + 6];
acadoWorkspace.state[74] = acadoVariables.od[lRun1 * 25 + 7];
acadoWorkspace.state[75] = acadoVariables.od[lRun1 * 25 + 8];
acadoWorkspace.state[76] = acadoVariables.od[lRun1 * 25 + 9];
acadoWorkspace.state[77] = acadoVariables.od[lRun1 * 25 + 10];
acadoWorkspace.state[78] = acadoVariables.od[lRun1 * 25 + 11];
acadoWorkspace.state[79] = acadoVariables.od[lRun1 * 25 + 12];
acadoWorkspace.state[80] = acadoVariables.od[lRun1 * 25 + 13];
acadoWorkspace.state[81] = acadoVariables.od[lRun1 * 25 + 14];
acadoWorkspace.state[82] = acadoVariables.od[lRun1 * 25 + 15];
acadoWorkspace.state[83] = acadoVariables.od[lRun1 * 25 + 16];
acadoWorkspace.state[84] = acadoVariables.od[lRun1 * 25 + 17];
acadoWorkspace.state[85] = acadoVariables.od[lRun1 * 25 + 18];
acadoWorkspace.state[86] = acadoVariables.od[lRun1 * 25 + 19];
acadoWorkspace.state[87] = acadoVariables.od[lRun1 * 25 + 20];
acadoWorkspace.state[88] = acadoVariables.od[lRun1 * 25 + 21];
acadoWorkspace.state[89] = acadoVariables.od[lRun1 * 25 + 22];
acadoWorkspace.state[90] = acadoVariables.od[lRun1 * 25 + 23];
acadoWorkspace.state[91] = acadoVariables.od[lRun1 * 25 + 24];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[23];

acadoWorkspace.evGu[lRun1 * 12] = acadoWorkspace.state[24];
acadoWorkspace.evGu[lRun1 * 12 + 1] = acadoWorkspace.state[25];
acadoWorkspace.evGu[lRun1 * 12 + 2] = acadoWorkspace.state[26];
acadoWorkspace.evGu[lRun1 * 12 + 3] = acadoWorkspace.state[27];
acadoWorkspace.evGu[lRun1 * 12 + 4] = acadoWorkspace.state[28];
acadoWorkspace.evGu[lRun1 * 12 + 5] = acadoWorkspace.state[29];
acadoWorkspace.evGu[lRun1 * 12 + 6] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 12 + 7] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 12 + 8] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 12 + 9] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 12 + 10] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 12 + 11] = acadoWorkspace.state[35];
acadoWorkspace.EH[lRun1 * 49] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[36];
acadoWorkspace.EH[lRun1 * 49 + 7] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[37];
acadoWorkspace.EH[lRun1 * 49 + 1] = acadoWorkspace.EH[lRun1 * 49 + 7];
acadoWorkspace.EH[lRun1 * 49 + 8] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[38];
acadoWorkspace.EH[lRun1 * 49 + 14] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[39];
acadoWorkspace.EH[lRun1 * 49 + 2] = acadoWorkspace.EH[lRun1 * 49 + 14];
acadoWorkspace.EH[lRun1 * 49 + 15] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[40];
acadoWorkspace.EH[lRun1 * 49 + 9] = acadoWorkspace.EH[lRun1 * 49 + 15];
acadoWorkspace.EH[lRun1 * 49 + 16] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[41];
acadoWorkspace.EH[lRun1 * 49 + 21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[42];
acadoWorkspace.EH[lRun1 * 49 + 3] = acadoWorkspace.EH[lRun1 * 49 + 21];
acadoWorkspace.EH[lRun1 * 49 + 22] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[43];
acadoWorkspace.EH[lRun1 * 49 + 10] = acadoWorkspace.EH[lRun1 * 49 + 22];
acadoWorkspace.EH[lRun1 * 49 + 23] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[44];
acadoWorkspace.EH[lRun1 * 49 + 17] = acadoWorkspace.EH[lRun1 * 49 + 23];
acadoWorkspace.EH[lRun1 * 49 + 24] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[45];
acadoWorkspace.EH[lRun1 * 49 + 28] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[46];
acadoWorkspace.EH[lRun1 * 49 + 4] = acadoWorkspace.EH[lRun1 * 49 + 28];
acadoWorkspace.EH[lRun1 * 49 + 29] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[47];
acadoWorkspace.EH[lRun1 * 49 + 11] = acadoWorkspace.EH[lRun1 * 49 + 29];
acadoWorkspace.EH[lRun1 * 49 + 30] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[48];
acadoWorkspace.EH[lRun1 * 49 + 18] = acadoWorkspace.EH[lRun1 * 49 + 30];
acadoWorkspace.EH[lRun1 * 49 + 31] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[49];
acadoWorkspace.EH[lRun1 * 49 + 25] = acadoWorkspace.EH[lRun1 * 49 + 31];
acadoWorkspace.EH[lRun1 * 49 + 32] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[50];
acadoWorkspace.EH[lRun1 * 49 + 35] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[51];
acadoWorkspace.EH[lRun1 * 49 + 5] = acadoWorkspace.EH[lRun1 * 49 + 35];
acadoWorkspace.EH[lRun1 * 49 + 36] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[52];
acadoWorkspace.EH[lRun1 * 49 + 12] = acadoWorkspace.EH[lRun1 * 49 + 36];
acadoWorkspace.EH[lRun1 * 49 + 37] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[53];
acadoWorkspace.EH[lRun1 * 49 + 19] = acadoWorkspace.EH[lRun1 * 49 + 37];
acadoWorkspace.EH[lRun1 * 49 + 38] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[54];
acadoWorkspace.EH[lRun1 * 49 + 26] = acadoWorkspace.EH[lRun1 * 49 + 38];
acadoWorkspace.EH[lRun1 * 49 + 39] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[55];
acadoWorkspace.EH[lRun1 * 49 + 33] = acadoWorkspace.EH[lRun1 * 49 + 39];
acadoWorkspace.EH[lRun1 * 49 + 40] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[56];
acadoWorkspace.EH[lRun1 * 49 + 42] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[57];
acadoWorkspace.EH[lRun1 * 49 + 6] = acadoWorkspace.EH[lRun1 * 49 + 42];
acadoWorkspace.EH[lRun1 * 49 + 43] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[58];
acadoWorkspace.EH[lRun1 * 49 + 13] = acadoWorkspace.EH[lRun1 * 49 + 43];
acadoWorkspace.EH[lRun1 * 49 + 44] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[59];
acadoWorkspace.EH[lRun1 * 49 + 20] = acadoWorkspace.EH[lRun1 * 49 + 44];
acadoWorkspace.EH[lRun1 * 49 + 45] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[60];
acadoWorkspace.EH[lRun1 * 49 + 27] = acadoWorkspace.EH[lRun1 * 49 + 45];
acadoWorkspace.EH[lRun1 * 49 + 46] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[61];
acadoWorkspace.EH[lRun1 * 49 + 34] = acadoWorkspace.EH[lRun1 * 49 + 46];
acadoWorkspace.EH[lRun1 * 49 + 47] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[62];
acadoWorkspace.EH[lRun1 * 49 + 41] = acadoWorkspace.EH[lRun1 * 49 + 47];
acadoWorkspace.EH[lRun1 * 49 + 48] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[63];
}
return ret;
}

void acado_evaluateLagrange(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
const real_t* od = in + 7;
/* Vector of auxiliary variables; number of elements: 86. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (xd[0]-od[0]);
a[1] = (od[3]*a[0]);
a[2] = (od[3]*(xd[0]-od[0]));
a[3] = (a[1]+a[2]);
a[4] = (xd[0]-od[15]);
a[5] = (xd[0]-od[15]);
a[6] = (a[4]+a[5]);
a[7] = ((real_t)(1.0000000000000000e+00)/((((xd[0]-od[15])*(xd[0]-od[15]))+((xd[1]-od[16])*(xd[1]-od[16])))+(real_t)(1.0000000000000000e-04)));
a[8] = (a[7]*a[7]);
a[9] = ((real_t)(-1.0000000000000000e+00)*a[8]);
a[10] = (a[9]*(real_t)(1.0000000000000000e+00));
a[11] = (a[6]*a[10]);
a[12] = (xd[0]-od[20]);
a[13] = (xd[0]-od[20]);
a[14] = (a[12]+a[13]);
a[15] = ((real_t)(1.0000000000000000e+00)/((((xd[0]-od[20])*(xd[0]-od[20]))+((xd[1]-od[21])*(xd[1]-od[21])))+(real_t)(1.0000000000000000e-04)));
a[16] = (a[15]*a[15]);
a[17] = ((real_t)(-1.0000000000000000e+00)*a[16]);
a[18] = (a[17]*(real_t)(1.0000000000000000e+00));
a[19] = (a[14]*a[18]);
a[20] = ((a[11]+a[19])*od[12]);
a[21] = (a[3]+a[20]);
a[22] = (xd[1]-od[1]);
a[23] = (od[4]*a[22]);
a[24] = (od[4]*(xd[1]-od[1]));
a[25] = (a[23]+a[24]);
a[26] = (xd[1]-od[16]);
a[27] = (xd[1]-od[16]);
a[28] = (a[26]+a[27]);
a[29] = (a[28]*a[10]);
a[30] = (xd[1]-od[21]);
a[31] = (xd[1]-od[21]);
a[32] = (a[30]+a[31]);
a[33] = (a[32]*a[18]);
a[34] = ((a[29]+a[33])*od[12]);
a[35] = (a[25]+a[34]);
a[36] = (xd[2]-od[2]);
a[37] = (od[5]*a[36]);
a[38] = (od[5]*(xd[2]-od[2]));
a[39] = (a[37]+a[38]);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (od[6]*u[0]);
a[42] = (od[6]*u[0]);
a[43] = (a[41]+a[42]);
a[44] = (od[7]*u[1]);
a[45] = (od[7]*u[1]);
a[46] = (a[44]+a[45]);
a[47] = (od[11]*u[2]);
a[48] = (od[11]*u[2]);
a[49] = (a[47]+a[48]);
a[50] = (a[7]*a[8]);
a[51] = (((real_t)(2.0000000000000000e+00)*a[50])*(real_t)(1.0000000000000000e+00));
a[52] = (od[12]*a[51]);
a[53] = (od[12]*a[10]);
a[54] = (a[15]*a[16]);
a[55] = (((real_t)(2.0000000000000000e+00)*a[54])*(real_t)(1.0000000000000000e+00));
a[56] = (od[12]*a[55]);
a[57] = (od[12]*a[18]);
a[58] = ((od[3]*(real_t)(2.0000000000000000e+00))+(((((a[6])*(a[6]))*a[52])+((real_t)(2.0000000000000000e+00)*a[53]))+((((a[14])*(a[14]))*a[56])+((real_t)(2.0000000000000000e+00)*a[57]))));
a[59] = (((a[28]*a[6])*a[52])+((a[32]*a[14])*a[56]));
a[60] = (real_t)(0.0000000000000000e+00);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = ((od[4]*(real_t)(2.0000000000000000e+00))+(((((a[28])*(a[28]))*a[52])+((real_t)(2.0000000000000000e+00)*a[53]))+((((a[32])*(a[32]))*a[56])+((real_t)(2.0000000000000000e+00)*a[57]))));
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (od[5]*(real_t)(2.0000000000000000e+00));
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = (real_t)(0.0000000000000000e+00);
a[68] = (real_t)(0.0000000000000000e+00);
a[69] = (real_t)(0.0000000000000000e+00);
a[70] = (real_t)(0.0000000000000000e+00);
a[71] = (real_t)(0.0000000000000000e+00);
a[72] = (real_t)(0.0000000000000000e+00);
a[73] = (real_t)(0.0000000000000000e+00);
a[74] = (real_t)(0.0000000000000000e+00);
a[75] = (real_t)(0.0000000000000000e+00);
a[76] = (real_t)(0.0000000000000000e+00);
a[77] = (real_t)(0.0000000000000000e+00);
a[78] = (real_t)(0.0000000000000000e+00);
a[79] = (real_t)(0.0000000000000000e+00);
a[80] = (od[6]*(real_t)(2.0000000000000000e+00));
a[81] = (real_t)(0.0000000000000000e+00);
a[82] = (real_t)(0.0000000000000000e+00);
a[83] = (od[7]*(real_t)(2.0000000000000000e+00));
a[84] = (real_t)(0.0000000000000000e+00);
a[85] = (od[11]*(real_t)(2.0000000000000000e+00));

/* Compute outputs: */
out[0] = ((((((((od[3]*(xd[0]-od[0]))*(xd[0]-od[0]))+((od[4]*(xd[1]-od[1]))*(xd[1]-od[1])))+((od[5]*(xd[2]-od[2]))*(xd[2]-od[2])))+((od[6]*u[0])*u[0]))+((od[7]*u[1])*u[1]))+((od[11]*u[2])*u[2]))+(od[12]*(((real_t)(1.0000000000000000e+00)/((((xd[0]-od[15])*(xd[0]-od[15]))+((xd[1]-od[16])*(xd[1]-od[16])))+(real_t)(1.0000000000000000e-04)))+((real_t)(1.0000000000000000e+00)/((((xd[0]-od[20])*(xd[0]-od[20]))+((xd[1]-od[21])*(xd[1]-od[21])))+(real_t)(1.0000000000000000e-04))))));
out[1] = a[21];
out[2] = a[35];
out[3] = a[39];
out[4] = a[40];
out[5] = a[43];
out[6] = a[46];
out[7] = a[49];
out[8] = a[58];
out[9] = a[59];
out[10] = a[60];
out[11] = a[61];
out[12] = a[59];
out[13] = a[62];
out[14] = a[63];
out[15] = a[64];
out[16] = a[60];
out[17] = a[63];
out[18] = a[65];
out[19] = a[66];
out[20] = a[61];
out[21] = a[64];
out[22] = a[66];
out[23] = a[67];
out[24] = a[68];
out[25] = a[69];
out[26] = a[70];
out[27] = a[71];
out[28] = a[72];
out[29] = a[73];
out[30] = a[74];
out[31] = a[75];
out[32] = a[76];
out[33] = a[77];
out[34] = a[78];
out[35] = a[79];
out[36] = a[80];
out[37] = a[81];
out[38] = a[82];
out[39] = a[81];
out[40] = a[83];
out[41] = a[84];
out[42] = a[82];
out[43] = a[84];
out[44] = a[85];
}

void acado_evaluateMayer(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 4;
/* Vector of auxiliary variables; number of elements: 23. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (xd[0]-od[0]);
a[1] = (od[8]*a[0]);
a[2] = (od[8]*(xd[0]-od[0]));
a[3] = (a[1]+a[2]);
a[4] = (xd[1]-od[1]);
a[5] = (od[9]*a[4]);
a[6] = (od[9]*(xd[1]-od[1]));
a[7] = (a[5]+a[6]);
a[8] = (xd[2]-od[2]);
a[9] = (od[10]*a[8]);
a[10] = (od[10]*(xd[2]-od[2]));
a[11] = (a[9]+a[10]);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (od[8]*(real_t)(2.0000000000000000e+00));
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (od[9]*(real_t)(2.0000000000000000e+00));
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (od[10]*(real_t)(2.0000000000000000e+00));
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = ((((od[8]*(xd[0]-od[0]))*(xd[0]-od[0]))+((od[9]*(xd[1]-od[1]))*(xd[1]-od[1])))+((od[10]*(xd[2]-od[2]))*(xd[2]-od[2])));
out[1] = a[3];
out[2] = a[7];
out[3] = a[11];
out[4] = a[12];
out[5] = a[13];
out[6] = a[14];
out[7] = a[15];
out[8] = a[16];
out[9] = a[14];
out[10] = a[17];
out[11] = a[18];
out[12] = a[19];
out[13] = a[15];
out[14] = a[18];
out[15] = a[20];
out[16] = a[21];
out[17] = a[16];
out[18] = a[19];
out[19] = a[21];
out[20] = a[22];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;
const real_t* od = in + 9;
/* Vector of auxiliary variables; number of elements: 134. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (xd[0]-od[15]);
a[1] = (cos(od[17]));
a[2] = (a[0]*a[1]);
a[3] = (xd[1]-od[16]);
a[4] = (sin(od[17]));
a[5] = ((real_t)(0.0000000000000000e+00)-a[4]);
a[6] = (a[2]+(a[3]*a[5]));
a[7] = ((real_t)(1.0000000000000000e+00)/((od[18]+od[13])*(od[18]+od[13])));
a[8] = (a[6]*a[7]);
a[9] = (a[8]*a[1]);
a[10] = (sin(od[17]));
a[11] = (a[0]*a[10]);
a[12] = (cos(od[17]));
a[13] = (a[11]+(a[3]*a[12]));
a[14] = ((real_t)(1.0000000000000000e+00)/((od[19]+od[13])*(od[19]+od[13])));
a[15] = (a[13]*a[14]);
a[16] = (a[9]+(a[15]*a[10]));
a[17] = (a[16]*a[0]);
a[18] = (a[8]*a[5]);
a[19] = (a[18]+(a[15]*a[12]));
a[20] = (a[17]+(a[19]*a[3]));
a[21] = (xd[0]-od[20]);
a[22] = (cos(od[22]));
a[23] = (a[21]*a[22]);
a[24] = (xd[1]-od[21]);
a[25] = (sin(od[22]));
a[26] = ((real_t)(0.0000000000000000e+00)-a[25]);
a[27] = (a[23]+(a[24]*a[26]));
a[28] = ((real_t)(1.0000000000000000e+00)/((od[23]+od[13])*(od[23]+od[13])));
a[29] = (a[27]*a[28]);
a[30] = (a[29]*a[22]);
a[31] = (sin(od[22]));
a[32] = (a[21]*a[31]);
a[33] = (cos(od[22]));
a[34] = (a[32]+(a[24]*a[33]));
a[35] = ((real_t)(1.0000000000000000e+00)/((od[24]+od[13])*(od[24]+od[13])));
a[36] = (a[34]*a[35]);
a[37] = (a[30]+(a[36]*a[31]));
a[38] = (a[37]*a[21]);
a[39] = (a[29]*a[26]);
a[40] = (a[39]+(a[36]*a[33]));
a[41] = (a[38]+(a[40]*a[24]));
a[42] = (a[1]*a[7]);
a[43] = (a[42]*a[1]);
a[44] = (a[10]*a[14]);
a[45] = (a[44]*a[10]);
a[46] = (a[43]+a[45]);
a[47] = (a[46]*a[0]);
a[48] = (a[47]+a[16]);
a[49] = (a[42]*a[5]);
a[50] = (a[44]*a[12]);
a[51] = (a[49]+a[50]);
a[52] = (a[51]*a[3]);
a[53] = (a[48]+a[52]);
a[54] = (a[5]*a[7]);
a[55] = (a[54]*a[1]);
a[56] = (a[12]*a[14]);
a[57] = (a[56]*a[10]);
a[58] = (a[55]+a[57]);
a[59] = (a[58]*a[0]);
a[60] = (a[54]*a[5]);
a[61] = (a[56]*a[12]);
a[62] = (a[60]+a[61]);
a[63] = (a[62]*a[3]);
a[64] = (a[63]+a[19]);
a[65] = (a[59]+a[64]);
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = (real_t)(0.0000000000000000e+00);
a[68] = (a[22]*a[28]);
a[69] = (a[68]*a[22]);
a[70] = (a[31]*a[35]);
a[71] = (a[70]*a[31]);
a[72] = (a[69]+a[71]);
a[73] = (a[72]*a[21]);
a[74] = (a[73]+a[37]);
a[75] = (a[68]*a[26]);
a[76] = (a[70]*a[33]);
a[77] = (a[75]+a[76]);
a[78] = (a[77]*a[24]);
a[79] = (a[74]+a[78]);
a[80] = (a[26]*a[28]);
a[81] = (a[80]*a[22]);
a[82] = (a[33]*a[35]);
a[83] = (a[82]*a[31]);
a[84] = (a[81]+a[83]);
a[85] = (a[84]*a[21]);
a[86] = (a[80]*a[26]);
a[87] = (a[82]*a[33]);
a[88] = (a[86]+a[87]);
a[89] = (a[88]*a[24]);
a[90] = (a[89]+a[40]);
a[91] = (a[85]+a[90]);
a[92] = (real_t)(0.0000000000000000e+00);
a[93] = (real_t)(0.0000000000000000e+00);
a[94] = (real_t)(0.0000000000000000e+00);
a[95] = (real_t)(0.0000000000000000e+00);
a[96] = (real_t)(-1.0000000000000000e+00);
a[97] = (real_t)(0.0000000000000000e+00);
a[98] = (real_t)(0.0000000000000000e+00);
a[99] = (real_t)(-1.0000000000000000e+00);
a[100] = (a[46]*(real_t)(2.0000000000000000e+00));
a[101] = (a[72]*(real_t)(2.0000000000000000e+00));
a[102] = ((a[100]*xd[4])+(a[101]*xd[5]));
a[103] = (a[58]+a[51]);
a[104] = (a[84]+a[77]);
a[105] = ((a[103]*xd[4])+(a[104]*xd[5]));
a[106] = (real_t)(0.0000000000000000e+00);
a[107] = (real_t)(0.0000000000000000e+00);
a[108] = (real_t)(0.0000000000000000e+00);
a[109] = (real_t)(0.0000000000000000e+00);
a[110] = (real_t)(0.0000000000000000e+00);
a[111] = (a[62]*(real_t)(2.0000000000000000e+00));
a[112] = (a[88]*(real_t)(2.0000000000000000e+00));
a[113] = ((a[111]*xd[4])+(a[112]*xd[5]));
a[114] = (real_t)(0.0000000000000000e+00);
a[115] = (real_t)(0.0000000000000000e+00);
a[116] = (real_t)(0.0000000000000000e+00);
a[117] = (real_t)(0.0000000000000000e+00);
a[118] = (real_t)(0.0000000000000000e+00);
a[119] = (real_t)(0.0000000000000000e+00);
a[120] = (real_t)(0.0000000000000000e+00);
a[121] = (real_t)(0.0000000000000000e+00);
a[122] = (real_t)(0.0000000000000000e+00);
a[123] = (real_t)(0.0000000000000000e+00);
a[124] = (real_t)(0.0000000000000000e+00);
a[125] = (real_t)(0.0000000000000000e+00);
a[126] = (real_t)(0.0000000000000000e+00);
a[127] = (real_t)(0.0000000000000000e+00);
a[128] = (real_t)(0.0000000000000000e+00);
a[129] = (real_t)(0.0000000000000000e+00);
a[130] = (real_t)(0.0000000000000000e+00);
a[131] = (real_t)(0.0000000000000000e+00);
a[132] = (real_t)(0.0000000000000000e+00);
a[133] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (a[20]-u[2]);
out[1] = (a[41]-u[2]);
out[2] = a[53];
out[3] = a[65];
out[4] = a[66];
out[5] = a[67];
out[6] = a[79];
out[7] = a[91];
out[8] = a[92];
out[9] = a[93];
out[10] = a[94];
out[11] = a[95];
out[12] = a[96];
out[13] = a[97];
out[14] = a[98];
out[15] = a[99];
out[16] = a[102];
out[17] = a[105];
out[18] = a[106];
out[19] = a[107];
out[20] = a[108];
out[21] = a[109];
out[22] = a[110];
out[23] = a[105];
out[24] = a[113];
out[25] = a[114];
out[26] = a[115];
out[27] = a[116];
out[28] = a[117];
out[29] = a[118];
out[30] = a[106];
out[31] = a[114];
out[32] = a[119];
out[33] = a[120];
out[34] = a[121];
out[35] = a[122];
out[36] = a[123];
out[37] = a[107];
out[38] = a[115];
out[39] = a[120];
out[40] = a[124];
out[41] = a[125];
out[42] = a[126];
out[43] = a[127];
out[44] = a[108];
out[45] = a[116];
out[46] = a[121];
out[47] = a[125];
out[48] = a[128];
out[49] = a[129];
out[50] = a[130];
out[51] = a[109];
out[52] = a[117];
out[53] = a[122];
out[54] = a[126];
out[55] = a[129];
out[56] = a[131];
out[57] = a[132];
out[58] = a[110];
out[59] = a[118];
out[60] = a[123];
out[61] = a[127];
out[62] = a[130];
out[63] = a[132];
out[64] = a[133];
}

void acado_addObjTerm( real_t* const tmpFxx, real_t* const tmpFxu, real_t* const tmpFuu, real_t* const tmpEH )
{
tmpEH[0] += tmpFxx[0];
tmpEH[1] += tmpFxx[1];
tmpEH[2] += tmpFxx[2];
tmpEH[3] += tmpFxx[3];
tmpEH[7] += tmpFxx[4];
tmpEH[8] += tmpFxx[5];
tmpEH[9] += tmpFxx[6];
tmpEH[10] += tmpFxx[7];
tmpEH[14] += tmpFxx[8];
tmpEH[15] += tmpFxx[9];
tmpEH[16] += tmpFxx[10];
tmpEH[17] += tmpFxx[11];
tmpEH[21] += tmpFxx[12];
tmpEH[22] += tmpFxx[13];
tmpEH[23] += tmpFxx[14];
tmpEH[24] += tmpFxx[15];
tmpEH[4] += tmpFxu[0];
tmpEH[5] += tmpFxu[1];
tmpEH[6] += tmpFxu[2];
tmpEH[11] += tmpFxu[3];
tmpEH[12] += tmpFxu[4];
tmpEH[13] += tmpFxu[5];
tmpEH[18] += tmpFxu[6];
tmpEH[19] += tmpFxu[7];
tmpEH[20] += tmpFxu[8];
tmpEH[25] += tmpFxu[9];
tmpEH[26] += tmpFxu[10];
tmpEH[27] += tmpFxu[11];
tmpEH[28] += tmpFxu[0];
tmpEH[29] += tmpFxu[3];
tmpEH[30] += tmpFxu[6];
tmpEH[31] += tmpFxu[9];
tmpEH[35] += tmpFxu[1];
tmpEH[36] += tmpFxu[4];
tmpEH[37] += tmpFxu[7];
tmpEH[38] += tmpFxu[10];
tmpEH[42] += tmpFxu[2];
tmpEH[43] += tmpFxu[5];
tmpEH[44] += tmpFxu[8];
tmpEH[45] += tmpFxu[11];
tmpEH[32] += tmpFuu[0];
tmpEH[33] += tmpFuu[1];
tmpEH[34] += tmpFuu[2];
tmpEH[39] += tmpFuu[3];
tmpEH[40] += tmpFuu[4];
tmpEH[41] += tmpFuu[5];
tmpEH[46] += tmpFuu[6];
tmpEH[47] += tmpFuu[7];
tmpEH[48] += tmpFuu[8];
}

void acado_addObjLinearTerm( real_t* const tmpDx, real_t* const tmpDu, real_t* const tmpDF )
{
tmpDx[0] = tmpDF[0];
tmpDx[1] = tmpDF[1];
tmpDx[2] = tmpDF[2];
tmpDx[3] = tmpDF[3];
tmpDu[0] = tmpDF[4];
tmpDu[1] = tmpDF[5];
tmpDu[2] = tmpDF[6];
}

void acado_addObjEndTerm( real_t* const tmpFxxEnd, real_t* const tmpEH_N )
{
tmpEH_N[0] = tmpFxxEnd[0];
tmpEH_N[1] = tmpFxxEnd[1];
tmpEH_N[2] = tmpFxxEnd[2];
tmpEH_N[3] = tmpFxxEnd[3];
tmpEH_N[4] = tmpFxxEnd[4];
tmpEH_N[5] = tmpFxxEnd[5];
tmpEH_N[6] = tmpFxxEnd[6];
tmpEH_N[7] = tmpFxxEnd[7];
tmpEH_N[8] = tmpFxxEnd[8];
tmpEH_N[9] = tmpFxxEnd[9];
tmpEH_N[10] = tmpFxxEnd[10];
tmpEH_N[11] = tmpFxxEnd[11];
tmpEH_N[12] = tmpFxxEnd[12];
tmpEH_N[13] = tmpFxxEnd[13];
tmpEH_N[14] = tmpFxxEnd[14];
tmpEH_N[15] = tmpFxxEnd[15];
}

void acado_evaluateObjective(  )
{
int lRun2;
int runObj;
for (runObj = 0; runObj < 25; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 25];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 25 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 25 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 25 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 25 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 25 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 25 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 25 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 25 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 25 + 9];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 25 + 10];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 25 + 11];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 25 + 12];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 25 + 13];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 25 + 14];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 25 + 15];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 25 + 16];
acadoWorkspace.objValueIn[24] = acadoVariables.od[runObj * 25 + 17];
acadoWorkspace.objValueIn[25] = acadoVariables.od[runObj * 25 + 18];
acadoWorkspace.objValueIn[26] = acadoVariables.od[runObj * 25 + 19];
acadoWorkspace.objValueIn[27] = acadoVariables.od[runObj * 25 + 20];
acadoWorkspace.objValueIn[28] = acadoVariables.od[runObj * 25 + 21];
acadoWorkspace.objValueIn[29] = acadoVariables.od[runObj * 25 + 22];
acadoWorkspace.objValueIn[30] = acadoVariables.od[runObj * 25 + 23];
acadoWorkspace.objValueIn[31] = acadoVariables.od[runObj * 25 + 24];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjTerm( &(acadoWorkspace.objValueOut[ 8 ]), &(acadoWorkspace.objValueOut[ 24 ]), &(acadoWorkspace.objValueOut[ 36 ]), &(acadoWorkspace.EH[ runObj * 49 ]) );
acado_addObjLinearTerm( &(acadoWorkspace.QDy[ runObj * 4 ]), &(acadoWorkspace.g[ runObj * 3 ]), &(acadoWorkspace.objValueOut[ 1 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[100];
acadoWorkspace.objValueIn[1] = acadoVariables.x[101];
acadoWorkspace.objValueIn[2] = acadoVariables.x[102];
acadoWorkspace.objValueIn[3] = acadoVariables.x[103];
acadoWorkspace.objValueIn[4] = acadoVariables.od[625];
acadoWorkspace.objValueIn[5] = acadoVariables.od[626];
acadoWorkspace.objValueIn[6] = acadoVariables.od[627];
acadoWorkspace.objValueIn[7] = acadoVariables.od[628];
acadoWorkspace.objValueIn[8] = acadoVariables.od[629];
acadoWorkspace.objValueIn[9] = acadoVariables.od[630];
acadoWorkspace.objValueIn[10] = acadoVariables.od[631];
acadoWorkspace.objValueIn[11] = acadoVariables.od[632];
acadoWorkspace.objValueIn[12] = acadoVariables.od[633];
acadoWorkspace.objValueIn[13] = acadoVariables.od[634];
acadoWorkspace.objValueIn[14] = acadoVariables.od[635];
acadoWorkspace.objValueIn[15] = acadoVariables.od[636];
acadoWorkspace.objValueIn[16] = acadoVariables.od[637];
acadoWorkspace.objValueIn[17] = acadoVariables.od[638];
acadoWorkspace.objValueIn[18] = acadoVariables.od[639];
acadoWorkspace.objValueIn[19] = acadoVariables.od[640];
acadoWorkspace.objValueIn[20] = acadoVariables.od[641];
acadoWorkspace.objValueIn[21] = acadoVariables.od[642];
acadoWorkspace.objValueIn[22] = acadoVariables.od[643];
acadoWorkspace.objValueIn[23] = acadoVariables.od[644];
acadoWorkspace.objValueIn[24] = acadoVariables.od[645];
acadoWorkspace.objValueIn[25] = acadoVariables.od[646];
acadoWorkspace.objValueIn[26] = acadoVariables.od[647];
acadoWorkspace.objValueIn[27] = acadoVariables.od[648];
acadoWorkspace.objValueIn[28] = acadoVariables.od[649];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjEndTerm( &(acadoWorkspace.objValueOut[ 5 ]), acadoWorkspace.EH_N );
acadoWorkspace.QDy[100] = acadoWorkspace.objValueOut[1];
acadoWorkspace.QDy[101] = acadoWorkspace.objValueOut[2];
acadoWorkspace.QDy[102] = acadoWorkspace.objValueOut[3];
acadoWorkspace.QDy[103] = acadoWorkspace.objValueOut[4];

for (lRun2 = 0; lRun2 < 25; ++lRun2)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun2 * 4];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun2 * 4 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun2 * 4 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun2 * 4 + 3];
acadoWorkspace.conValueIn[4] = acadoWorkspace.y[lRun2 * 2 + 75];
acadoWorkspace.conValueIn[5] = acadoWorkspace.y[lRun2 * 2 + 76];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun2 * 3];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun2 * 3 + 1];
acadoWorkspace.conValueIn[8] = acadoVariables.u[lRun2 * 3 + 2];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun2 * 25];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun2 * 25 + 1];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun2 * 25 + 2];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun2 * 25 + 3];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun2 * 25 + 4];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun2 * 25 + 5];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun2 * 25 + 6];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun2 * 25 + 7];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun2 * 25 + 8];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun2 * 25 + 9];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun2 * 25 + 10];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun2 * 25 + 11];
acadoWorkspace.conValueIn[21] = acadoVariables.od[lRun2 * 25 + 12];
acadoWorkspace.conValueIn[22] = acadoVariables.od[lRun2 * 25 + 13];
acadoWorkspace.conValueIn[23] = acadoVariables.od[lRun2 * 25 + 14];
acadoWorkspace.conValueIn[24] = acadoVariables.od[lRun2 * 25 + 15];
acadoWorkspace.conValueIn[25] = acadoVariables.od[lRun2 * 25 + 16];
acadoWorkspace.conValueIn[26] = acadoVariables.od[lRun2 * 25 + 17];
acadoWorkspace.conValueIn[27] = acadoVariables.od[lRun2 * 25 + 18];
acadoWorkspace.conValueIn[28] = acadoVariables.od[lRun2 * 25 + 19];
acadoWorkspace.conValueIn[29] = acadoVariables.od[lRun2 * 25 + 20];
acadoWorkspace.conValueIn[30] = acadoVariables.od[lRun2 * 25 + 21];
acadoWorkspace.conValueIn[31] = acadoVariables.od[lRun2 * 25 + 22];
acadoWorkspace.conValueIn[32] = acadoVariables.od[lRun2 * 25 + 23];
acadoWorkspace.conValueIn[33] = acadoVariables.od[lRun2 * 25 + 24];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun2 * 2] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun2 * 2 + 1] = acadoWorkspace.conValueOut[1];

acadoWorkspace.evHx[lRun2 * 8] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun2 * 8 + 1] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun2 * 8 + 2] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun2 * 8 + 3] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun2 * 8 + 4] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun2 * 8 + 5] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun2 * 8 + 6] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun2 * 8 + 7] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHu[lRun2 * 6] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHu[lRun2 * 6 + 1] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHu[lRun2 * 6 + 2] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHu[lRun2 * 6 + 3] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHu[lRun2 * 6 + 4] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHu[lRun2 * 6 + 5] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evDDH[0] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evDDH[1] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evDDH[2] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evDDH[3] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evDDH[4] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evDDH[5] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evDDH[6] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evDDH[7] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evDDH[8] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evDDH[9] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evDDH[10] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evDDH[11] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evDDH[12] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evDDH[13] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evDDH[14] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evDDH[15] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evDDH[16] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evDDH[17] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evDDH[18] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evDDH[19] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evDDH[20] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evDDH[21] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evDDH[22] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evDDH[23] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evDDH[24] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evDDH[25] = acadoWorkspace.conValueOut[41];
acadoWorkspace.evDDH[26] = acadoWorkspace.conValueOut[42];
acadoWorkspace.evDDH[27] = acadoWorkspace.conValueOut[43];
acadoWorkspace.evDDH[28] = acadoWorkspace.conValueOut[44];
acadoWorkspace.evDDH[29] = acadoWorkspace.conValueOut[45];
acadoWorkspace.evDDH[30] = acadoWorkspace.conValueOut[46];
acadoWorkspace.evDDH[31] = acadoWorkspace.conValueOut[47];
acadoWorkspace.evDDH[32] = acadoWorkspace.conValueOut[48];
acadoWorkspace.evDDH[33] = acadoWorkspace.conValueOut[49];
acadoWorkspace.evDDH[34] = acadoWorkspace.conValueOut[50];
acadoWorkspace.evDDH[35] = acadoWorkspace.conValueOut[51];
acadoWorkspace.evDDH[36] = acadoWorkspace.conValueOut[52];
acadoWorkspace.evDDH[37] = acadoWorkspace.conValueOut[53];
acadoWorkspace.evDDH[38] = acadoWorkspace.conValueOut[54];
acadoWorkspace.evDDH[39] = acadoWorkspace.conValueOut[55];
acadoWorkspace.evDDH[40] = acadoWorkspace.conValueOut[56];
acadoWorkspace.evDDH[41] = acadoWorkspace.conValueOut[57];
acadoWorkspace.evDDH[42] = acadoWorkspace.conValueOut[58];
acadoWorkspace.evDDH[43] = acadoWorkspace.conValueOut[59];
acadoWorkspace.evDDH[44] = acadoWorkspace.conValueOut[60];
acadoWorkspace.evDDH[45] = acadoWorkspace.conValueOut[61];
acadoWorkspace.evDDH[46] = acadoWorkspace.conValueOut[62];
acadoWorkspace.evDDH[47] = acadoWorkspace.conValueOut[63];
acadoWorkspace.evDDH[48] = acadoWorkspace.conValueOut[64];
acadoWorkspace.EH[lRun2 * 49] += acadoWorkspace.evDDH[0];
acadoWorkspace.EH[lRun2 * 49 + 1] += acadoWorkspace.evDDH[1];
acadoWorkspace.EH[lRun2 * 49 + 2] += acadoWorkspace.evDDH[2];
acadoWorkspace.EH[lRun2 * 49 + 3] += acadoWorkspace.evDDH[3];
acadoWorkspace.EH[lRun2 * 49 + 4] += acadoWorkspace.evDDH[4];
acadoWorkspace.EH[lRun2 * 49 + 5] += acadoWorkspace.evDDH[5];
acadoWorkspace.EH[lRun2 * 49 + 6] += acadoWorkspace.evDDH[6];
acadoWorkspace.EH[lRun2 * 49 + 7] += acadoWorkspace.evDDH[7];
acadoWorkspace.EH[lRun2 * 49 + 8] += acadoWorkspace.evDDH[8];
acadoWorkspace.EH[lRun2 * 49 + 9] += acadoWorkspace.evDDH[9];
acadoWorkspace.EH[lRun2 * 49 + 10] += acadoWorkspace.evDDH[10];
acadoWorkspace.EH[lRun2 * 49 + 11] += acadoWorkspace.evDDH[11];
acadoWorkspace.EH[lRun2 * 49 + 12] += acadoWorkspace.evDDH[12];
acadoWorkspace.EH[lRun2 * 49 + 13] += acadoWorkspace.evDDH[13];
acadoWorkspace.EH[lRun2 * 49 + 14] += acadoWorkspace.evDDH[14];
acadoWorkspace.EH[lRun2 * 49 + 15] += acadoWorkspace.evDDH[15];
acadoWorkspace.EH[lRun2 * 49 + 16] += acadoWorkspace.evDDH[16];
acadoWorkspace.EH[lRun2 * 49 + 17] += acadoWorkspace.evDDH[17];
acadoWorkspace.EH[lRun2 * 49 + 18] += acadoWorkspace.evDDH[18];
acadoWorkspace.EH[lRun2 * 49 + 19] += acadoWorkspace.evDDH[19];
acadoWorkspace.EH[lRun2 * 49 + 20] += acadoWorkspace.evDDH[20];
acadoWorkspace.EH[lRun2 * 49 + 21] += acadoWorkspace.evDDH[21];
acadoWorkspace.EH[lRun2 * 49 + 22] += acadoWorkspace.evDDH[22];
acadoWorkspace.EH[lRun2 * 49 + 23] += acadoWorkspace.evDDH[23];
acadoWorkspace.EH[lRun2 * 49 + 24] += acadoWorkspace.evDDH[24];
acadoWorkspace.EH[lRun2 * 49 + 25] += acadoWorkspace.evDDH[25];
acadoWorkspace.EH[lRun2 * 49 + 26] += acadoWorkspace.evDDH[26];
acadoWorkspace.EH[lRun2 * 49 + 27] += acadoWorkspace.evDDH[27];
acadoWorkspace.EH[lRun2 * 49 + 28] += acadoWorkspace.evDDH[28];
acadoWorkspace.EH[lRun2 * 49 + 29] += acadoWorkspace.evDDH[29];
acadoWorkspace.EH[lRun2 * 49 + 30] += acadoWorkspace.evDDH[30];
acadoWorkspace.EH[lRun2 * 49 + 31] += acadoWorkspace.evDDH[31];
acadoWorkspace.EH[lRun2 * 49 + 32] += acadoWorkspace.evDDH[32];
acadoWorkspace.EH[lRun2 * 49 + 33] += acadoWorkspace.evDDH[33];
acadoWorkspace.EH[lRun2 * 49 + 34] += acadoWorkspace.evDDH[34];
acadoWorkspace.EH[lRun2 * 49 + 35] += acadoWorkspace.evDDH[35];
acadoWorkspace.EH[lRun2 * 49 + 36] += acadoWorkspace.evDDH[36];
acadoWorkspace.EH[lRun2 * 49 + 37] += acadoWorkspace.evDDH[37];
acadoWorkspace.EH[lRun2 * 49 + 38] += acadoWorkspace.evDDH[38];
acadoWorkspace.EH[lRun2 * 49 + 39] += acadoWorkspace.evDDH[39];
acadoWorkspace.EH[lRun2 * 49 + 40] += acadoWorkspace.evDDH[40];
acadoWorkspace.EH[lRun2 * 49 + 41] += acadoWorkspace.evDDH[41];
acadoWorkspace.EH[lRun2 * 49 + 42] += acadoWorkspace.evDDH[42];
acadoWorkspace.EH[lRun2 * 49 + 43] += acadoWorkspace.evDDH[43];
acadoWorkspace.EH[lRun2 * 49 + 44] += acadoWorkspace.evDDH[44];
acadoWorkspace.EH[lRun2 * 49 + 45] += acadoWorkspace.evDDH[45];
acadoWorkspace.EH[lRun2 * 49 + 46] += acadoWorkspace.evDDH[46];
acadoWorkspace.EH[lRun2 * 49 + 47] += acadoWorkspace.evDDH[47];
acadoWorkspace.EH[lRun2 * 49 + 48] += acadoWorkspace.evDDH[48];
}

}

void acado_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acado_regularize( &(acadoWorkspace.EH[ lRun1 * 49 ]) );
acadoWorkspace.Q1[lRun1 * 16] = acadoWorkspace.EH[lRun1 * 49];
acadoWorkspace.Q1[lRun1 * 16 + 1] = acadoWorkspace.EH[lRun1 * 49 + 1];
acadoWorkspace.Q1[lRun1 * 16 + 2] = acadoWorkspace.EH[lRun1 * 49 + 2];
acadoWorkspace.Q1[lRun1 * 16 + 3] = acadoWorkspace.EH[lRun1 * 49 + 3];
acadoWorkspace.Q1[lRun1 * 16 + 4] = acadoWorkspace.EH[lRun1 * 49 + 7];
acadoWorkspace.Q1[lRun1 * 16 + 5] = acadoWorkspace.EH[lRun1 * 49 + 8];
acadoWorkspace.Q1[lRun1 * 16 + 6] = acadoWorkspace.EH[lRun1 * 49 + 9];
acadoWorkspace.Q1[lRun1 * 16 + 7] = acadoWorkspace.EH[lRun1 * 49 + 10];
acadoWorkspace.Q1[lRun1 * 16 + 8] = acadoWorkspace.EH[lRun1 * 49 + 14];
acadoWorkspace.Q1[lRun1 * 16 + 9] = acadoWorkspace.EH[lRun1 * 49 + 15];
acadoWorkspace.Q1[lRun1 * 16 + 10] = acadoWorkspace.EH[lRun1 * 49 + 16];
acadoWorkspace.Q1[lRun1 * 16 + 11] = acadoWorkspace.EH[lRun1 * 49 + 17];
acadoWorkspace.Q1[lRun1 * 16 + 12] = acadoWorkspace.EH[lRun1 * 49 + 21];
acadoWorkspace.Q1[lRun1 * 16 + 13] = acadoWorkspace.EH[lRun1 * 49 + 22];
acadoWorkspace.Q1[lRun1 * 16 + 14] = acadoWorkspace.EH[lRun1 * 49 + 23];
acadoWorkspace.Q1[lRun1 * 16 + 15] = acadoWorkspace.EH[lRun1 * 49 + 24];
acadoWorkspace.S1[lRun1 * 12] = acadoWorkspace.EH[lRun1 * 49 + 4];
acadoWorkspace.S1[lRun1 * 12 + 1] = acadoWorkspace.EH[lRun1 * 49 + 5];
acadoWorkspace.S1[lRun1 * 12 + 2] = acadoWorkspace.EH[lRun1 * 49 + 6];
acadoWorkspace.S1[lRun1 * 12 + 3] = acadoWorkspace.EH[lRun1 * 49 + 11];
acadoWorkspace.S1[lRun1 * 12 + 4] = acadoWorkspace.EH[lRun1 * 49 + 12];
acadoWorkspace.S1[lRun1 * 12 + 5] = acadoWorkspace.EH[lRun1 * 49 + 13];
acadoWorkspace.S1[lRun1 * 12 + 6] = acadoWorkspace.EH[lRun1 * 49 + 18];
acadoWorkspace.S1[lRun1 * 12 + 7] = acadoWorkspace.EH[lRun1 * 49 + 19];
acadoWorkspace.S1[lRun1 * 12 + 8] = acadoWorkspace.EH[lRun1 * 49 + 20];
acadoWorkspace.S1[lRun1 * 12 + 9] = acadoWorkspace.EH[lRun1 * 49 + 25];
acadoWorkspace.S1[lRun1 * 12 + 10] = acadoWorkspace.EH[lRun1 * 49 + 26];
acadoWorkspace.S1[lRun1 * 12 + 11] = acadoWorkspace.EH[lRun1 * 49 + 27];
acadoWorkspace.R1[lRun1 * 9] = acadoWorkspace.EH[lRun1 * 49 + 32];
acadoWorkspace.R1[lRun1 * 9 + 1] = acadoWorkspace.EH[lRun1 * 49 + 33];
acadoWorkspace.R1[lRun1 * 9 + 2] = acadoWorkspace.EH[lRun1 * 49 + 34];
acadoWorkspace.R1[lRun1 * 9 + 3] = acadoWorkspace.EH[lRun1 * 49 + 39];
acadoWorkspace.R1[lRun1 * 9 + 4] = acadoWorkspace.EH[lRun1 * 49 + 40];
acadoWorkspace.R1[lRun1 * 9 + 5] = acadoWorkspace.EH[lRun1 * 49 + 41];
acadoWorkspace.R1[lRun1 * 9 + 6] = acadoWorkspace.EH[lRun1 * 49 + 46];
acadoWorkspace.R1[lRun1 * 9 + 7] = acadoWorkspace.EH[lRun1 * 49 + 47];
acadoWorkspace.R1[lRun1 * 9 + 8] = acadoWorkspace.EH[lRun1 * 49 + 48];
}
acadoWorkspace.QN1[0] = acadoWorkspace.EH_N[0];
acadoWorkspace.QN1[1] = acadoWorkspace.EH_N[1];
acadoWorkspace.QN1[2] = acadoWorkspace.EH_N[2];
acadoWorkspace.QN1[3] = acadoWorkspace.EH_N[3];
acadoWorkspace.QN1[4] = acadoWorkspace.EH_N[4];
acadoWorkspace.QN1[5] = acadoWorkspace.EH_N[5];
acadoWorkspace.QN1[6] = acadoWorkspace.EH_N[6];
acadoWorkspace.QN1[7] = acadoWorkspace.EH_N[7];
acadoWorkspace.QN1[8] = acadoWorkspace.EH_N[8];
acadoWorkspace.QN1[9] = acadoWorkspace.EH_N[9];
acadoWorkspace.QN1[10] = acadoWorkspace.EH_N[10];
acadoWorkspace.QN1[11] = acadoWorkspace.EH_N[11];
acadoWorkspace.QN1[12] = acadoWorkspace.EH_N[12];
acadoWorkspace.QN1[13] = acadoWorkspace.EH_N[13];
acadoWorkspace.QN1[14] = acadoWorkspace.EH_N[14];
acadoWorkspace.QN1[15] = acadoWorkspace.EH_N[15];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[8] + Gx1[3]*Gx2[12];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[9] + Gx1[3]*Gx2[13];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[14];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[15];
Gx3[4] = + Gx1[4]*Gx2[0] + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[12];
Gx3[5] = + Gx1[4]*Gx2[1] + Gx1[5]*Gx2[5] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[13];
Gx3[6] = + Gx1[4]*Gx2[2] + Gx1[5]*Gx2[6] + Gx1[6]*Gx2[10] + Gx1[7]*Gx2[14];
Gx3[7] = + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[7] + Gx1[6]*Gx2[11] + Gx1[7]*Gx2[15];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[8] + Gx1[11]*Gx2[12];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[13];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[14];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[15];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[4] + Gx1[14]*Gx2[8] + Gx1[15]*Gx2[12];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[5] + Gx1[14]*Gx2[9] + Gx1[15]*Gx2[13];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[10] + Gx1[15]*Gx2[14];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[11] + Gx1[15]*Gx2[15];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11];
Gu2[3] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[3] + Gx1[6]*Gu1[6] + Gx1[7]*Gu1[9];
Gu2[4] = + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[4] + Gx1[6]*Gu1[7] + Gx1[7]*Gu1[10];
Gu2[5] = + Gx1[4]*Gu1[2] + Gx1[5]*Gu1[5] + Gx1[6]*Gu1[8] + Gx1[7]*Gu1[11];
Gu2[6] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[6] + Gx1[11]*Gu1[9];
Gu2[7] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[10]*Gu1[7] + Gx1[11]*Gu1[10];
Gu2[8] = + Gx1[8]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[11];
Gu2[9] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[6] + Gx1[15]*Gu1[9];
Gu2[10] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[7] + Gx1[15]*Gu1[10];
Gu2[11] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[11];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 225) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 225) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 228] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + R11[0];
acadoWorkspace.H[iRow * 228 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + R11[1];
acadoWorkspace.H[iRow * 228 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + R11[2];
acadoWorkspace.H[iRow * 228 + 75] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + R11[3];
acadoWorkspace.H[iRow * 228 + 76] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + R11[4];
acadoWorkspace.H[iRow * 228 + 77] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + R11[5];
acadoWorkspace.H[iRow * 228 + 150] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + R11[6];
acadoWorkspace.H[iRow * 228 + 151] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + R11[7];
acadoWorkspace.H[iRow * 228 + 152] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + R11[8];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[4]*Gu1[3] + Gx1[8]*Gu1[6] + Gx1[12]*Gu1[9];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[4]*Gu1[4] + Gx1[8]*Gu1[7] + Gx1[12]*Gu1[10];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[4]*Gu1[5] + Gx1[8]*Gu1[8] + Gx1[12]*Gu1[11];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[5]*Gu1[3] + Gx1[9]*Gu1[6] + Gx1[13]*Gu1[9];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[5]*Gu1[4] + Gx1[9]*Gu1[7] + Gx1[13]*Gu1[10];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[5]*Gu1[5] + Gx1[9]*Gu1[8] + Gx1[13]*Gu1[11];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[6]*Gu1[3] + Gx1[10]*Gu1[6] + Gx1[14]*Gu1[9];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[6]*Gu1[4] + Gx1[10]*Gu1[7] + Gx1[14]*Gu1[10];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[6]*Gu1[5] + Gx1[10]*Gu1[8] + Gx1[14]*Gu1[11];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[7]*Gu1[3] + Gx1[11]*Gu1[6] + Gx1[15]*Gu1[9];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[7]*Gu1[4] + Gx1[11]*Gu1[7] + Gx1[15]*Gu1[10];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[7]*Gu1[5] + Gx1[11]*Gu1[8] + Gx1[15]*Gu1[11];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Gu2[2];
Gu3[3] = + Q11[4]*Gu1[0] + Q11[5]*Gu1[3] + Q11[6]*Gu1[6] + Q11[7]*Gu1[9] + Gu2[3];
Gu3[4] = + Q11[4]*Gu1[1] + Q11[5]*Gu1[4] + Q11[6]*Gu1[7] + Q11[7]*Gu1[10] + Gu2[4];
Gu3[5] = + Q11[4]*Gu1[2] + Q11[5]*Gu1[5] + Q11[6]*Gu1[8] + Q11[7]*Gu1[11] + Gu2[5];
Gu3[6] = + Q11[8]*Gu1[0] + Q11[9]*Gu1[3] + Q11[10]*Gu1[6] + Q11[11]*Gu1[9] + Gu2[6];
Gu3[7] = + Q11[8]*Gu1[1] + Q11[9]*Gu1[4] + Q11[10]*Gu1[7] + Q11[11]*Gu1[10] + Gu2[7];
Gu3[8] = + Q11[8]*Gu1[2] + Q11[9]*Gu1[5] + Q11[10]*Gu1[8] + Q11[11]*Gu1[11] + Gu2[8];
Gu3[9] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[3] + Q11[14]*Gu1[6] + Q11[15]*Gu1[9] + Gu2[9];
Gu3[10] = + Q11[12]*Gu1[1] + Q11[13]*Gu1[4] + Q11[14]*Gu1[7] + Q11[15]*Gu1[10] + Gu2[10];
Gu3[11] = + Q11[12]*Gu1[2] + Q11[13]*Gu1[5] + Q11[14]*Gu1[8] + Q11[15]*Gu1[11] + Gu2[11];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[4]*w11[1] + Gx1[8]*w11[2] + Gx1[12]*w11[3] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[5]*w11[1] + Gx1[9]*w11[2] + Gx1[13]*w11[3] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[6]*w11[1] + Gx1[10]*w11[2] + Gx1[14]*w11[3] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[7]*w11[1] + Gx1[11]*w11[2] + Gx1[15]*w11[3] + w12[3];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + w12[0];
w13[1] = + Q11[4]*w11[0] + Q11[5]*w11[1] + Q11[6]*w11[2] + Q11[7]*w11[3] + w12[1];
w13[2] = + Q11[8]*w11[0] + Q11[9]*w11[1] + Q11[10]*w11[2] + Q11[11]*w11[3] + w12[2];
w13[3] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + w12[3];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
}

void acado_expansionStep2( real_t* const QDy1, real_t* const Q11, real_t* const w11, real_t* const Gu1, real_t* const U1, real_t* const Gx1, real_t* const mu1, real_t* const mu2 )
{
mu1[0] += QDy1[0];
mu1[1] += QDy1[1];
mu1[2] += QDy1[2];
mu1[3] += QDy1[3];
mu1[0] += + w11[0]*Q11[0] + w11[1]*Q11[1] + w11[2]*Q11[2] + w11[3]*Q11[3];
mu1[1] += + w11[0]*Q11[4] + w11[1]*Q11[5] + w11[2]*Q11[6] + w11[3]*Q11[7];
mu1[2] += + w11[0]*Q11[8] + w11[1]*Q11[9] + w11[2]*Q11[10] + w11[3]*Q11[11];
mu1[3] += + w11[0]*Q11[12] + w11[1]*Q11[13] + w11[2]*Q11[14] + w11[3]*Q11[15];
mu1[0] += + U1[0]*Gu1[0] + U1[1]*Gu1[1] + U1[2]*Gu1[2];
mu1[1] += + U1[0]*Gu1[3] + U1[1]*Gu1[4] + U1[2]*Gu1[5];
mu1[2] += + U1[0]*Gu1[6] + U1[1]*Gu1[7] + U1[2]*Gu1[8];
mu1[3] += + U1[0]*Gu1[9] + U1[1]*Gu1[10] + U1[2]*Gu1[11];
mu1[0] += + mu2[0]*Gx1[0] + mu2[1]*Gx1[4] + mu2[2]*Gx1[8] + mu2[3]*Gx1[12];
mu1[1] += + mu2[0]*Gx1[1] + mu2[1]*Gx1[5] + mu2[2]*Gx1[9] + mu2[3]*Gx1[13];
mu1[2] += + mu2[0]*Gx1[2] + mu2[1]*Gx1[6] + mu2[2]*Gx1[10] + mu2[3]*Gx1[14];
mu1[3] += + mu2[0]*Gx1[3] + mu2[1]*Gx1[7] + mu2[2]*Gx1[11] + mu2[3]*Gx1[15];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 225) + (iCol * 3)] = acadoWorkspace.H[(iCol * 225) + (iRow * 3)];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 225 + 75) + (iRow * 3)];
acadoWorkspace.H[(iRow * 225) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 225 + 150) + (iRow * 3)];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3)] = acadoWorkspace.H[(iCol * 225) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 225 + 75) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 225 + 75) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 225 + 150) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3)] = acadoWorkspace.H[(iCol * 225) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 225 + 75) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 225 + 150) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 225 + 150) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const RDy1 )
{
}

void acado_multQDy( real_t* const QDy1 )
{
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 150) + (col * 3)] = + Hx[0]*E[0] + Hx[1]*E[3] + Hx[2]*E[6] + Hx[3]*E[9];
acadoWorkspace.A[(row * 150) + (col * 3 + 1)] = + Hx[0]*E[1] + Hx[1]*E[4] + Hx[2]*E[7] + Hx[3]*E[10];
acadoWorkspace.A[(row * 150) + (col * 3 + 2)] = + Hx[0]*E[2] + Hx[1]*E[5] + Hx[2]*E[8] + Hx[3]*E[11];
acadoWorkspace.A[(row * 150 + 75) + (col * 3)] = + Hx[4]*E[0] + Hx[5]*E[3] + Hx[6]*E[6] + Hx[7]*E[9];
acadoWorkspace.A[(row * 150 + 75) + (col * 3 + 1)] = + Hx[4]*E[1] + Hx[5]*E[4] + Hx[6]*E[7] + Hx[7]*E[10];
acadoWorkspace.A[(row * 150 + 75) + (col * 3 + 2)] = + Hx[4]*E[2] + Hx[5]*E[5] + Hx[6]*E[8] + Hx[7]*E[11];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3];
acadoWorkspace.evHxd[1] = + Hx[4]*tmpd[0] + Hx[5]*tmpd[1] + Hx[6]*tmpd[2] + Hx[7]*tmpd[3];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 16 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.C[ 16 ]), &(acadoWorkspace.C[ 32 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.C[ 32 ]), &(acadoWorkspace.C[ 48 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.C[ 48 ]), &(acadoWorkspace.C[ 64 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.C[ 64 ]), &(acadoWorkspace.C[ 80 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.C[ 80 ]), &(acadoWorkspace.C[ 96 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.C[ 96 ]), &(acadoWorkspace.C[ 112 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.C[ 112 ]), &(acadoWorkspace.C[ 128 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.C[ 128 ]), &(acadoWorkspace.C[ 144 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.C[ 144 ]), &(acadoWorkspace.C[ 160 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.C[ 160 ]), &(acadoWorkspace.C[ 176 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.C[ 176 ]), &(acadoWorkspace.C[ 192 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.C[ 192 ]), &(acadoWorkspace.C[ 208 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.C[ 208 ]), &(acadoWorkspace.C[ 224 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.C[ 224 ]), &(acadoWorkspace.C[ 240 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.C[ 240 ]), &(acadoWorkspace.C[ 256 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.C[ 256 ]), &(acadoWorkspace.C[ 272 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.C[ 272 ]), &(acadoWorkspace.C[ 288 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.C[ 288 ]), &(acadoWorkspace.C[ 304 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.C[ 304 ]), &(acadoWorkspace.C[ 320 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.C[ 320 ]), &(acadoWorkspace.C[ 336 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.C[ 336 ]), &(acadoWorkspace.C[ 352 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.C[ 352 ]), &(acadoWorkspace.C[ 368 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.C[ 368 ]), &(acadoWorkspace.C[ 384 ]) );
for (lRun2 = 0; lRun2 < 25; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 51)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 12 ]), &(acadoWorkspace.E[ lRun3 * 12 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 25; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (4)) * (4)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (4)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (4)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (25)) - (1)) * (4)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 24; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 12 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 12 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (3)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 16 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 12 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

acadoWorkspace.sbar[4] = acadoWorkspace.d[0];
acadoWorkspace.sbar[5] = acadoWorkspace.d[1];
acadoWorkspace.sbar[6] = acadoWorkspace.d[2];
acadoWorkspace.sbar[7] = acadoWorkspace.d[3];
acadoWorkspace.sbar[8] = acadoWorkspace.d[4];
acadoWorkspace.sbar[9] = acadoWorkspace.d[5];
acadoWorkspace.sbar[10] = acadoWorkspace.d[6];
acadoWorkspace.sbar[11] = acadoWorkspace.d[7];
acadoWorkspace.sbar[12] = acadoWorkspace.d[8];
acadoWorkspace.sbar[13] = acadoWorkspace.d[9];
acadoWorkspace.sbar[14] = acadoWorkspace.d[10];
acadoWorkspace.sbar[15] = acadoWorkspace.d[11];
acadoWorkspace.sbar[16] = acadoWorkspace.d[12];
acadoWorkspace.sbar[17] = acadoWorkspace.d[13];
acadoWorkspace.sbar[18] = acadoWorkspace.d[14];
acadoWorkspace.sbar[19] = acadoWorkspace.d[15];
acadoWorkspace.sbar[20] = acadoWorkspace.d[16];
acadoWorkspace.sbar[21] = acadoWorkspace.d[17];
acadoWorkspace.sbar[22] = acadoWorkspace.d[18];
acadoWorkspace.sbar[23] = acadoWorkspace.d[19];
acadoWorkspace.sbar[24] = acadoWorkspace.d[20];
acadoWorkspace.sbar[25] = acadoWorkspace.d[21];
acadoWorkspace.sbar[26] = acadoWorkspace.d[22];
acadoWorkspace.sbar[27] = acadoWorkspace.d[23];
acadoWorkspace.sbar[28] = acadoWorkspace.d[24];
acadoWorkspace.sbar[29] = acadoWorkspace.d[25];
acadoWorkspace.sbar[30] = acadoWorkspace.d[26];
acadoWorkspace.sbar[31] = acadoWorkspace.d[27];
acadoWorkspace.sbar[32] = acadoWorkspace.d[28];
acadoWorkspace.sbar[33] = acadoWorkspace.d[29];
acadoWorkspace.sbar[34] = acadoWorkspace.d[30];
acadoWorkspace.sbar[35] = acadoWorkspace.d[31];
acadoWorkspace.sbar[36] = acadoWorkspace.d[32];
acadoWorkspace.sbar[37] = acadoWorkspace.d[33];
acadoWorkspace.sbar[38] = acadoWorkspace.d[34];
acadoWorkspace.sbar[39] = acadoWorkspace.d[35];
acadoWorkspace.sbar[40] = acadoWorkspace.d[36];
acadoWorkspace.sbar[41] = acadoWorkspace.d[37];
acadoWorkspace.sbar[42] = acadoWorkspace.d[38];
acadoWorkspace.sbar[43] = acadoWorkspace.d[39];
acadoWorkspace.sbar[44] = acadoWorkspace.d[40];
acadoWorkspace.sbar[45] = acadoWorkspace.d[41];
acadoWorkspace.sbar[46] = acadoWorkspace.d[42];
acadoWorkspace.sbar[47] = acadoWorkspace.d[43];
acadoWorkspace.sbar[48] = acadoWorkspace.d[44];
acadoWorkspace.sbar[49] = acadoWorkspace.d[45];
acadoWorkspace.sbar[50] = acadoWorkspace.d[46];
acadoWorkspace.sbar[51] = acadoWorkspace.d[47];
acadoWorkspace.sbar[52] = acadoWorkspace.d[48];
acadoWorkspace.sbar[53] = acadoWorkspace.d[49];
acadoWorkspace.sbar[54] = acadoWorkspace.d[50];
acadoWorkspace.sbar[55] = acadoWorkspace.d[51];
acadoWorkspace.sbar[56] = acadoWorkspace.d[52];
acadoWorkspace.sbar[57] = acadoWorkspace.d[53];
acadoWorkspace.sbar[58] = acadoWorkspace.d[54];
acadoWorkspace.sbar[59] = acadoWorkspace.d[55];
acadoWorkspace.sbar[60] = acadoWorkspace.d[56];
acadoWorkspace.sbar[61] = acadoWorkspace.d[57];
acadoWorkspace.sbar[62] = acadoWorkspace.d[58];
acadoWorkspace.sbar[63] = acadoWorkspace.d[59];
acadoWorkspace.sbar[64] = acadoWorkspace.d[60];
acadoWorkspace.sbar[65] = acadoWorkspace.d[61];
acadoWorkspace.sbar[66] = acadoWorkspace.d[62];
acadoWorkspace.sbar[67] = acadoWorkspace.d[63];
acadoWorkspace.sbar[68] = acadoWorkspace.d[64];
acadoWorkspace.sbar[69] = acadoWorkspace.d[65];
acadoWorkspace.sbar[70] = acadoWorkspace.d[66];
acadoWorkspace.sbar[71] = acadoWorkspace.d[67];
acadoWorkspace.sbar[72] = acadoWorkspace.d[68];
acadoWorkspace.sbar[73] = acadoWorkspace.d[69];
acadoWorkspace.sbar[74] = acadoWorkspace.d[70];
acadoWorkspace.sbar[75] = acadoWorkspace.d[71];
acadoWorkspace.sbar[76] = acadoWorkspace.d[72];
acadoWorkspace.sbar[77] = acadoWorkspace.d[73];
acadoWorkspace.sbar[78] = acadoWorkspace.d[74];
acadoWorkspace.sbar[79] = acadoWorkspace.d[75];
acadoWorkspace.sbar[80] = acadoWorkspace.d[76];
acadoWorkspace.sbar[81] = acadoWorkspace.d[77];
acadoWorkspace.sbar[82] = acadoWorkspace.d[78];
acadoWorkspace.sbar[83] = acadoWorkspace.d[79];
acadoWorkspace.sbar[84] = acadoWorkspace.d[80];
acadoWorkspace.sbar[85] = acadoWorkspace.d[81];
acadoWorkspace.sbar[86] = acadoWorkspace.d[82];
acadoWorkspace.sbar[87] = acadoWorkspace.d[83];
acadoWorkspace.sbar[88] = acadoWorkspace.d[84];
acadoWorkspace.sbar[89] = acadoWorkspace.d[85];
acadoWorkspace.sbar[90] = acadoWorkspace.d[86];
acadoWorkspace.sbar[91] = acadoWorkspace.d[87];
acadoWorkspace.sbar[92] = acadoWorkspace.d[88];
acadoWorkspace.sbar[93] = acadoWorkspace.d[89];
acadoWorkspace.sbar[94] = acadoWorkspace.d[90];
acadoWorkspace.sbar[95] = acadoWorkspace.d[91];
acadoWorkspace.sbar[96] = acadoWorkspace.d[92];
acadoWorkspace.sbar[97] = acadoWorkspace.d[93];
acadoWorkspace.sbar[98] = acadoWorkspace.d[94];
acadoWorkspace.sbar[99] = acadoWorkspace.d[95];
acadoWorkspace.sbar[100] = acadoWorkspace.d[96];
acadoWorkspace.sbar[101] = acadoWorkspace.d[97];
acadoWorkspace.sbar[102] = acadoWorkspace.d[98];
acadoWorkspace.sbar[103] = acadoWorkspace.d[99];



for (lRun1 = 0; lRun1 < 24; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun2) * (lRun2 * -1 + 49)) / (2)) + (lRun1);
lRun4 = lRun1 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun1 * 8 + 8 ]), &(acadoWorkspace.E[ lRun3 * 12 ]), lRun4, lRun2 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[75] = acadoWorkspace.evHu[3];
acadoWorkspace.A[76] = acadoWorkspace.evHu[4];
acadoWorkspace.A[77] = acadoWorkspace.evHu[5];
acadoWorkspace.A[153] = acadoWorkspace.evHu[6];
acadoWorkspace.A[154] = acadoWorkspace.evHu[7];
acadoWorkspace.A[155] = acadoWorkspace.evHu[8];
acadoWorkspace.A[228] = acadoWorkspace.evHu[9];
acadoWorkspace.A[229] = acadoWorkspace.evHu[10];
acadoWorkspace.A[230] = acadoWorkspace.evHu[11];
acadoWorkspace.A[306] = acadoWorkspace.evHu[12];
acadoWorkspace.A[307] = acadoWorkspace.evHu[13];
acadoWorkspace.A[308] = acadoWorkspace.evHu[14];
acadoWorkspace.A[381] = acadoWorkspace.evHu[15];
acadoWorkspace.A[382] = acadoWorkspace.evHu[16];
acadoWorkspace.A[383] = acadoWorkspace.evHu[17];
acadoWorkspace.A[459] = acadoWorkspace.evHu[18];
acadoWorkspace.A[460] = acadoWorkspace.evHu[19];
acadoWorkspace.A[461] = acadoWorkspace.evHu[20];
acadoWorkspace.A[534] = acadoWorkspace.evHu[21];
acadoWorkspace.A[535] = acadoWorkspace.evHu[22];
acadoWorkspace.A[536] = acadoWorkspace.evHu[23];
acadoWorkspace.A[612] = acadoWorkspace.evHu[24];
acadoWorkspace.A[613] = acadoWorkspace.evHu[25];
acadoWorkspace.A[614] = acadoWorkspace.evHu[26];
acadoWorkspace.A[687] = acadoWorkspace.evHu[27];
acadoWorkspace.A[688] = acadoWorkspace.evHu[28];
acadoWorkspace.A[689] = acadoWorkspace.evHu[29];
acadoWorkspace.A[765] = acadoWorkspace.evHu[30];
acadoWorkspace.A[766] = acadoWorkspace.evHu[31];
acadoWorkspace.A[767] = acadoWorkspace.evHu[32];
acadoWorkspace.A[840] = acadoWorkspace.evHu[33];
acadoWorkspace.A[841] = acadoWorkspace.evHu[34];
acadoWorkspace.A[842] = acadoWorkspace.evHu[35];
acadoWorkspace.A[918] = acadoWorkspace.evHu[36];
acadoWorkspace.A[919] = acadoWorkspace.evHu[37];
acadoWorkspace.A[920] = acadoWorkspace.evHu[38];
acadoWorkspace.A[993] = acadoWorkspace.evHu[39];
acadoWorkspace.A[994] = acadoWorkspace.evHu[40];
acadoWorkspace.A[995] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1071] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1072] = acadoWorkspace.evHu[43];
acadoWorkspace.A[1073] = acadoWorkspace.evHu[44];
acadoWorkspace.A[1146] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1147] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1148] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1224] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1225] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1226] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1299] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1300] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1301] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1377] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1378] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1379] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1452] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1453] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1454] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1530] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1531] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1532] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1605] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1606] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1607] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1683] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1684] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1685] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1758] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1759] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1760] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1836] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1837] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1838] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1911] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1912] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1913] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1989] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1990] = acadoWorkspace.evHu[79];
acadoWorkspace.A[1991] = acadoWorkspace.evHu[80];
acadoWorkspace.A[2064] = acadoWorkspace.evHu[81];
acadoWorkspace.A[2065] = acadoWorkspace.evHu[82];
acadoWorkspace.A[2066] = acadoWorkspace.evHu[83];
acadoWorkspace.A[2142] = acadoWorkspace.evHu[84];
acadoWorkspace.A[2143] = acadoWorkspace.evHu[85];
acadoWorkspace.A[2144] = acadoWorkspace.evHu[86];
acadoWorkspace.A[2217] = acadoWorkspace.evHu[87];
acadoWorkspace.A[2218] = acadoWorkspace.evHu[88];
acadoWorkspace.A[2219] = acadoWorkspace.evHu[89];
acadoWorkspace.A[2295] = acadoWorkspace.evHu[90];
acadoWorkspace.A[2296] = acadoWorkspace.evHu[91];
acadoWorkspace.A[2297] = acadoWorkspace.evHu[92];
acadoWorkspace.A[2370] = acadoWorkspace.evHu[93];
acadoWorkspace.A[2371] = acadoWorkspace.evHu[94];
acadoWorkspace.A[2372] = acadoWorkspace.evHu[95];
acadoWorkspace.A[2448] = acadoWorkspace.evHu[96];
acadoWorkspace.A[2449] = acadoWorkspace.evHu[97];
acadoWorkspace.A[2450] = acadoWorkspace.evHu[98];
acadoWorkspace.A[2523] = acadoWorkspace.evHu[99];
acadoWorkspace.A[2524] = acadoWorkspace.evHu[100];
acadoWorkspace.A[2525] = acadoWorkspace.evHu[101];
acadoWorkspace.A[2601] = acadoWorkspace.evHu[102];
acadoWorkspace.A[2602] = acadoWorkspace.evHu[103];
acadoWorkspace.A[2603] = acadoWorkspace.evHu[104];
acadoWorkspace.A[2676] = acadoWorkspace.evHu[105];
acadoWorkspace.A[2677] = acadoWorkspace.evHu[106];
acadoWorkspace.A[2678] = acadoWorkspace.evHu[107];
acadoWorkspace.A[2754] = acadoWorkspace.evHu[108];
acadoWorkspace.A[2755] = acadoWorkspace.evHu[109];
acadoWorkspace.A[2756] = acadoWorkspace.evHu[110];
acadoWorkspace.A[2829] = acadoWorkspace.evHu[111];
acadoWorkspace.A[2830] = acadoWorkspace.evHu[112];
acadoWorkspace.A[2831] = acadoWorkspace.evHu[113];
acadoWorkspace.A[2907] = acadoWorkspace.evHu[114];
acadoWorkspace.A[2908] = acadoWorkspace.evHu[115];
acadoWorkspace.A[2909] = acadoWorkspace.evHu[116];
acadoWorkspace.A[2982] = acadoWorkspace.evHu[117];
acadoWorkspace.A[2983] = acadoWorkspace.evHu[118];
acadoWorkspace.A[2984] = acadoWorkspace.evHu[119];
acadoWorkspace.A[3060] = acadoWorkspace.evHu[120];
acadoWorkspace.A[3061] = acadoWorkspace.evHu[121];
acadoWorkspace.A[3062] = acadoWorkspace.evHu[122];
acadoWorkspace.A[3135] = acadoWorkspace.evHu[123];
acadoWorkspace.A[3136] = acadoWorkspace.evHu[124];
acadoWorkspace.A[3137] = acadoWorkspace.evHu[125];
acadoWorkspace.A[3213] = acadoWorkspace.evHu[126];
acadoWorkspace.A[3214] = acadoWorkspace.evHu[127];
acadoWorkspace.A[3215] = acadoWorkspace.evHu[128];
acadoWorkspace.A[3288] = acadoWorkspace.evHu[129];
acadoWorkspace.A[3289] = acadoWorkspace.evHu[130];
acadoWorkspace.A[3290] = acadoWorkspace.evHu[131];
acadoWorkspace.A[3366] = acadoWorkspace.evHu[132];
acadoWorkspace.A[3367] = acadoWorkspace.evHu[133];
acadoWorkspace.A[3368] = acadoWorkspace.evHu[134];
acadoWorkspace.A[3441] = acadoWorkspace.evHu[135];
acadoWorkspace.A[3442] = acadoWorkspace.evHu[136];
acadoWorkspace.A[3443] = acadoWorkspace.evHu[137];
acadoWorkspace.A[3519] = acadoWorkspace.evHu[138];
acadoWorkspace.A[3520] = acadoWorkspace.evHu[139];
acadoWorkspace.A[3521] = acadoWorkspace.evHu[140];
acadoWorkspace.A[3594] = acadoWorkspace.evHu[141];
acadoWorkspace.A[3595] = acadoWorkspace.evHu[142];
acadoWorkspace.A[3596] = acadoWorkspace.evHu[143];
acadoWorkspace.A[3672] = acadoWorkspace.evHu[144];
acadoWorkspace.A[3673] = acadoWorkspace.evHu[145];
acadoWorkspace.A[3674] = acadoWorkspace.evHu[146];
acadoWorkspace.A[3747] = acadoWorkspace.evHu[147];
acadoWorkspace.A[3748] = acadoWorkspace.evHu[148];
acadoWorkspace.A[3749] = acadoWorkspace.evHu[149];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - acadoWorkspace.evH[49];

acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - acadoWorkspace.evH[49];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 44 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 68 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 76 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 92 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 100 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[100];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[101];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[102];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[100] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[101] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[102] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[103] + acadoWorkspace.QDy[103];
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 276 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 276 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 368 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 92 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 264 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 264 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 352 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 88 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 228 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 228 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 76 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 204 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 204 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 68 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 192 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 168 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 156 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 132 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 44 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 12 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];


acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 88 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 104 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 136 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 152 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.lbA[ 38 ]), &(acadoWorkspace.ubA[ 38 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 176 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 184 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.d[0];
acadoWorkspace.sbar[5] = acadoWorkspace.d[1];
acadoWorkspace.sbar[6] = acadoWorkspace.d[2];
acadoWorkspace.sbar[7] = acadoWorkspace.d[3];
acadoWorkspace.sbar[8] = acadoWorkspace.d[4];
acadoWorkspace.sbar[9] = acadoWorkspace.d[5];
acadoWorkspace.sbar[10] = acadoWorkspace.d[6];
acadoWorkspace.sbar[11] = acadoWorkspace.d[7];
acadoWorkspace.sbar[12] = acadoWorkspace.d[8];
acadoWorkspace.sbar[13] = acadoWorkspace.d[9];
acadoWorkspace.sbar[14] = acadoWorkspace.d[10];
acadoWorkspace.sbar[15] = acadoWorkspace.d[11];
acadoWorkspace.sbar[16] = acadoWorkspace.d[12];
acadoWorkspace.sbar[17] = acadoWorkspace.d[13];
acadoWorkspace.sbar[18] = acadoWorkspace.d[14];
acadoWorkspace.sbar[19] = acadoWorkspace.d[15];
acadoWorkspace.sbar[20] = acadoWorkspace.d[16];
acadoWorkspace.sbar[21] = acadoWorkspace.d[17];
acadoWorkspace.sbar[22] = acadoWorkspace.d[18];
acadoWorkspace.sbar[23] = acadoWorkspace.d[19];
acadoWorkspace.sbar[24] = acadoWorkspace.d[20];
acadoWorkspace.sbar[25] = acadoWorkspace.d[21];
acadoWorkspace.sbar[26] = acadoWorkspace.d[22];
acadoWorkspace.sbar[27] = acadoWorkspace.d[23];
acadoWorkspace.sbar[28] = acadoWorkspace.d[24];
acadoWorkspace.sbar[29] = acadoWorkspace.d[25];
acadoWorkspace.sbar[30] = acadoWorkspace.d[26];
acadoWorkspace.sbar[31] = acadoWorkspace.d[27];
acadoWorkspace.sbar[32] = acadoWorkspace.d[28];
acadoWorkspace.sbar[33] = acadoWorkspace.d[29];
acadoWorkspace.sbar[34] = acadoWorkspace.d[30];
acadoWorkspace.sbar[35] = acadoWorkspace.d[31];
acadoWorkspace.sbar[36] = acadoWorkspace.d[32];
acadoWorkspace.sbar[37] = acadoWorkspace.d[33];
acadoWorkspace.sbar[38] = acadoWorkspace.d[34];
acadoWorkspace.sbar[39] = acadoWorkspace.d[35];
acadoWorkspace.sbar[40] = acadoWorkspace.d[36];
acadoWorkspace.sbar[41] = acadoWorkspace.d[37];
acadoWorkspace.sbar[42] = acadoWorkspace.d[38];
acadoWorkspace.sbar[43] = acadoWorkspace.d[39];
acadoWorkspace.sbar[44] = acadoWorkspace.d[40];
acadoWorkspace.sbar[45] = acadoWorkspace.d[41];
acadoWorkspace.sbar[46] = acadoWorkspace.d[42];
acadoWorkspace.sbar[47] = acadoWorkspace.d[43];
acadoWorkspace.sbar[48] = acadoWorkspace.d[44];
acadoWorkspace.sbar[49] = acadoWorkspace.d[45];
acadoWorkspace.sbar[50] = acadoWorkspace.d[46];
acadoWorkspace.sbar[51] = acadoWorkspace.d[47];
acadoWorkspace.sbar[52] = acadoWorkspace.d[48];
acadoWorkspace.sbar[53] = acadoWorkspace.d[49];
acadoWorkspace.sbar[54] = acadoWorkspace.d[50];
acadoWorkspace.sbar[55] = acadoWorkspace.d[51];
acadoWorkspace.sbar[56] = acadoWorkspace.d[52];
acadoWorkspace.sbar[57] = acadoWorkspace.d[53];
acadoWorkspace.sbar[58] = acadoWorkspace.d[54];
acadoWorkspace.sbar[59] = acadoWorkspace.d[55];
acadoWorkspace.sbar[60] = acadoWorkspace.d[56];
acadoWorkspace.sbar[61] = acadoWorkspace.d[57];
acadoWorkspace.sbar[62] = acadoWorkspace.d[58];
acadoWorkspace.sbar[63] = acadoWorkspace.d[59];
acadoWorkspace.sbar[64] = acadoWorkspace.d[60];
acadoWorkspace.sbar[65] = acadoWorkspace.d[61];
acadoWorkspace.sbar[66] = acadoWorkspace.d[62];
acadoWorkspace.sbar[67] = acadoWorkspace.d[63];
acadoWorkspace.sbar[68] = acadoWorkspace.d[64];
acadoWorkspace.sbar[69] = acadoWorkspace.d[65];
acadoWorkspace.sbar[70] = acadoWorkspace.d[66];
acadoWorkspace.sbar[71] = acadoWorkspace.d[67];
acadoWorkspace.sbar[72] = acadoWorkspace.d[68];
acadoWorkspace.sbar[73] = acadoWorkspace.d[69];
acadoWorkspace.sbar[74] = acadoWorkspace.d[70];
acadoWorkspace.sbar[75] = acadoWorkspace.d[71];
acadoWorkspace.sbar[76] = acadoWorkspace.d[72];
acadoWorkspace.sbar[77] = acadoWorkspace.d[73];
acadoWorkspace.sbar[78] = acadoWorkspace.d[74];
acadoWorkspace.sbar[79] = acadoWorkspace.d[75];
acadoWorkspace.sbar[80] = acadoWorkspace.d[76];
acadoWorkspace.sbar[81] = acadoWorkspace.d[77];
acadoWorkspace.sbar[82] = acadoWorkspace.d[78];
acadoWorkspace.sbar[83] = acadoWorkspace.d[79];
acadoWorkspace.sbar[84] = acadoWorkspace.d[80];
acadoWorkspace.sbar[85] = acadoWorkspace.d[81];
acadoWorkspace.sbar[86] = acadoWorkspace.d[82];
acadoWorkspace.sbar[87] = acadoWorkspace.d[83];
acadoWorkspace.sbar[88] = acadoWorkspace.d[84];
acadoWorkspace.sbar[89] = acadoWorkspace.d[85];
acadoWorkspace.sbar[90] = acadoWorkspace.d[86];
acadoWorkspace.sbar[91] = acadoWorkspace.d[87];
acadoWorkspace.sbar[92] = acadoWorkspace.d[88];
acadoWorkspace.sbar[93] = acadoWorkspace.d[89];
acadoWorkspace.sbar[94] = acadoWorkspace.d[90];
acadoWorkspace.sbar[95] = acadoWorkspace.d[91];
acadoWorkspace.sbar[96] = acadoWorkspace.d[92];
acadoWorkspace.sbar[97] = acadoWorkspace.d[93];
acadoWorkspace.sbar[98] = acadoWorkspace.d[94];
acadoWorkspace.sbar[99] = acadoWorkspace.d[95];
acadoWorkspace.sbar[100] = acadoWorkspace.d[96];
acadoWorkspace.sbar[101] = acadoWorkspace.d[97];
acadoWorkspace.sbar[102] = acadoWorkspace.d[98];
acadoWorkspace.sbar[103] = acadoWorkspace.d[99];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 44 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 68 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 76 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.evGu[ 252 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.evGu[ 264 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 92 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.evGu[ 276 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 100 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
acadoVariables.x[55] += acadoWorkspace.sbar[55];
acadoVariables.x[56] += acadoWorkspace.sbar[56];
acadoVariables.x[57] += acadoWorkspace.sbar[57];
acadoVariables.x[58] += acadoWorkspace.sbar[58];
acadoVariables.x[59] += acadoWorkspace.sbar[59];
acadoVariables.x[60] += acadoWorkspace.sbar[60];
acadoVariables.x[61] += acadoWorkspace.sbar[61];
acadoVariables.x[62] += acadoWorkspace.sbar[62];
acadoVariables.x[63] += acadoWorkspace.sbar[63];
acadoVariables.x[64] += acadoWorkspace.sbar[64];
acadoVariables.x[65] += acadoWorkspace.sbar[65];
acadoVariables.x[66] += acadoWorkspace.sbar[66];
acadoVariables.x[67] += acadoWorkspace.sbar[67];
acadoVariables.x[68] += acadoWorkspace.sbar[68];
acadoVariables.x[69] += acadoWorkspace.sbar[69];
acadoVariables.x[70] += acadoWorkspace.sbar[70];
acadoVariables.x[71] += acadoWorkspace.sbar[71];
acadoVariables.x[72] += acadoWorkspace.sbar[72];
acadoVariables.x[73] += acadoWorkspace.sbar[73];
acadoVariables.x[74] += acadoWorkspace.sbar[74];
acadoVariables.x[75] += acadoWorkspace.sbar[75];
acadoVariables.x[76] += acadoWorkspace.sbar[76];
acadoVariables.x[77] += acadoWorkspace.sbar[77];
acadoVariables.x[78] += acadoWorkspace.sbar[78];
acadoVariables.x[79] += acadoWorkspace.sbar[79];
acadoVariables.x[80] += acadoWorkspace.sbar[80];
acadoVariables.x[81] += acadoWorkspace.sbar[81];
acadoVariables.x[82] += acadoWorkspace.sbar[82];
acadoVariables.x[83] += acadoWorkspace.sbar[83];
acadoVariables.x[84] += acadoWorkspace.sbar[84];
acadoVariables.x[85] += acadoWorkspace.sbar[85];
acadoVariables.x[86] += acadoWorkspace.sbar[86];
acadoVariables.x[87] += acadoWorkspace.sbar[87];
acadoVariables.x[88] += acadoWorkspace.sbar[88];
acadoVariables.x[89] += acadoWorkspace.sbar[89];
acadoVariables.x[90] += acadoWorkspace.sbar[90];
acadoVariables.x[91] += acadoWorkspace.sbar[91];
acadoVariables.x[92] += acadoWorkspace.sbar[92];
acadoVariables.x[93] += acadoWorkspace.sbar[93];
acadoVariables.x[94] += acadoWorkspace.sbar[94];
acadoVariables.x[95] += acadoWorkspace.sbar[95];
acadoVariables.x[96] += acadoWorkspace.sbar[96];
acadoVariables.x[97] += acadoWorkspace.sbar[97];
acadoVariables.x[98] += acadoWorkspace.sbar[98];
acadoVariables.x[99] += acadoWorkspace.sbar[99];
acadoVariables.x[100] += acadoWorkspace.sbar[100];
acadoVariables.x[101] += acadoWorkspace.sbar[101];
acadoVariables.x[102] += acadoWorkspace.sbar[102];
acadoVariables.x[103] += acadoWorkspace.sbar[103];
acadoVariables.mu[96] = 0.0000000000000000e+00;
acadoVariables.mu[97] = 0.0000000000000000e+00;
acadoVariables.mu[98] = 0.0000000000000000e+00;
acadoVariables.mu[99] = 0.0000000000000000e+00;
acadoVariables.mu[96] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[8] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[12];
acadoVariables.mu[97] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[9] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[13];
acadoVariables.mu[98] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[6] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[10] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[14];
acadoVariables.mu[99] += + acadoWorkspace.sbar[100]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[101]*acadoWorkspace.QN1[7] + acadoWorkspace.sbar[102]*acadoWorkspace.QN1[11] + acadoWorkspace.sbar[103]*acadoWorkspace.QN1[15];
acadoVariables.mu[96] += acadoWorkspace.QDy[100];
acadoVariables.mu[97] += acadoWorkspace.QDy[101];
acadoVariables.mu[98] += acadoWorkspace.QDy[102];
acadoVariables.mu[99] += acadoWorkspace.QDy[103];
acadoVariables.mu[92] = 0.0000000000000000e+00;
acadoVariables.mu[92] -= + acadoWorkspace.y[123]*acadoWorkspace.evHx[192] + acadoWorkspace.y[124]*acadoWorkspace.evHx[196];
acadoVariables.mu[93] = 0.0000000000000000e+00;
acadoVariables.mu[93] -= + acadoWorkspace.y[123]*acadoWorkspace.evHx[193] + acadoWorkspace.y[124]*acadoWorkspace.evHx[197];
acadoVariables.mu[94] = 0.0000000000000000e+00;
acadoVariables.mu[94] -= + acadoWorkspace.y[123]*acadoWorkspace.evHx[194] + acadoWorkspace.y[124]*acadoWorkspace.evHx[198];
acadoVariables.mu[95] = 0.0000000000000000e+00;
acadoVariables.mu[95] -= + acadoWorkspace.y[123]*acadoWorkspace.evHx[195] + acadoWorkspace.y[124]*acadoWorkspace.evHx[199];
acado_expansionStep2( &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.S1[ 288 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoVariables.mu[ 92 ]), &(acadoVariables.mu[ 96 ]) );
acadoVariables.mu[88] = 0.0000000000000000e+00;
acadoVariables.mu[88] -= + acadoWorkspace.y[121]*acadoWorkspace.evHx[184] + acadoWorkspace.y[122]*acadoWorkspace.evHx[188];
acadoVariables.mu[89] = 0.0000000000000000e+00;
acadoVariables.mu[89] -= + acadoWorkspace.y[121]*acadoWorkspace.evHx[185] + acadoWorkspace.y[122]*acadoWorkspace.evHx[189];
acadoVariables.mu[90] = 0.0000000000000000e+00;
acadoVariables.mu[90] -= + acadoWorkspace.y[121]*acadoWorkspace.evHx[186] + acadoWorkspace.y[122]*acadoWorkspace.evHx[190];
acadoVariables.mu[91] = 0.0000000000000000e+00;
acadoVariables.mu[91] -= + acadoWorkspace.y[121]*acadoWorkspace.evHx[187] + acadoWorkspace.y[122]*acadoWorkspace.evHx[191];
acado_expansionStep2( &(acadoWorkspace.QDy[ 92 ]), &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.S1[ 276 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.evGx[ 368 ]), &(acadoVariables.mu[ 88 ]), &(acadoVariables.mu[ 92 ]) );
acadoVariables.mu[84] = 0.0000000000000000e+00;
acadoVariables.mu[84] -= + acadoWorkspace.y[119]*acadoWorkspace.evHx[176] + acadoWorkspace.y[120]*acadoWorkspace.evHx[180];
acadoVariables.mu[85] = 0.0000000000000000e+00;
acadoVariables.mu[85] -= + acadoWorkspace.y[119]*acadoWorkspace.evHx[177] + acadoWorkspace.y[120]*acadoWorkspace.evHx[181];
acadoVariables.mu[86] = 0.0000000000000000e+00;
acadoVariables.mu[86] -= + acadoWorkspace.y[119]*acadoWorkspace.evHx[178] + acadoWorkspace.y[120]*acadoWorkspace.evHx[182];
acadoVariables.mu[87] = 0.0000000000000000e+00;
acadoVariables.mu[87] -= + acadoWorkspace.y[119]*acadoWorkspace.evHx[179] + acadoWorkspace.y[120]*acadoWorkspace.evHx[183];
acado_expansionStep2( &(acadoWorkspace.QDy[ 88 ]), &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.S1[ 264 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.evGx[ 352 ]), &(acadoVariables.mu[ 84 ]), &(acadoVariables.mu[ 88 ]) );
acadoVariables.mu[80] = 0.0000000000000000e+00;
acadoVariables.mu[80] -= + acadoWorkspace.y[117]*acadoWorkspace.evHx[168] + acadoWorkspace.y[118]*acadoWorkspace.evHx[172];
acadoVariables.mu[81] = 0.0000000000000000e+00;
acadoVariables.mu[81] -= + acadoWorkspace.y[117]*acadoWorkspace.evHx[169] + acadoWorkspace.y[118]*acadoWorkspace.evHx[173];
acadoVariables.mu[82] = 0.0000000000000000e+00;
acadoVariables.mu[82] -= + acadoWorkspace.y[117]*acadoWorkspace.evHx[170] + acadoWorkspace.y[118]*acadoWorkspace.evHx[174];
acadoVariables.mu[83] = 0.0000000000000000e+00;
acadoVariables.mu[83] -= + acadoWorkspace.y[117]*acadoWorkspace.evHx[171] + acadoWorkspace.y[118]*acadoWorkspace.evHx[175];
acado_expansionStep2( &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.S1[ 252 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.evGx[ 336 ]), &(acadoVariables.mu[ 80 ]), &(acadoVariables.mu[ 84 ]) );
acadoVariables.mu[76] = 0.0000000000000000e+00;
acadoVariables.mu[76] -= + acadoWorkspace.y[115]*acadoWorkspace.evHx[160] + acadoWorkspace.y[116]*acadoWorkspace.evHx[164];
acadoVariables.mu[77] = 0.0000000000000000e+00;
acadoVariables.mu[77] -= + acadoWorkspace.y[115]*acadoWorkspace.evHx[161] + acadoWorkspace.y[116]*acadoWorkspace.evHx[165];
acadoVariables.mu[78] = 0.0000000000000000e+00;
acadoVariables.mu[78] -= + acadoWorkspace.y[115]*acadoWorkspace.evHx[162] + acadoWorkspace.y[116]*acadoWorkspace.evHx[166];
acadoVariables.mu[79] = 0.0000000000000000e+00;
acadoVariables.mu[79] -= + acadoWorkspace.y[115]*acadoWorkspace.evHx[163] + acadoWorkspace.y[116]*acadoWorkspace.evHx[167];
acado_expansionStep2( &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoVariables.mu[ 76 ]), &(acadoVariables.mu[ 80 ]) );
acadoVariables.mu[72] = 0.0000000000000000e+00;
acadoVariables.mu[72] -= + acadoWorkspace.y[113]*acadoWorkspace.evHx[152] + acadoWorkspace.y[114]*acadoWorkspace.evHx[156];
acadoVariables.mu[73] = 0.0000000000000000e+00;
acadoVariables.mu[73] -= + acadoWorkspace.y[113]*acadoWorkspace.evHx[153] + acadoWorkspace.y[114]*acadoWorkspace.evHx[157];
acadoVariables.mu[74] = 0.0000000000000000e+00;
acadoVariables.mu[74] -= + acadoWorkspace.y[113]*acadoWorkspace.evHx[154] + acadoWorkspace.y[114]*acadoWorkspace.evHx[158];
acadoVariables.mu[75] = 0.0000000000000000e+00;
acadoVariables.mu[75] -= + acadoWorkspace.y[113]*acadoWorkspace.evHx[155] + acadoWorkspace.y[114]*acadoWorkspace.evHx[159];
acado_expansionStep2( &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.S1[ 228 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoVariables.mu[ 72 ]), &(acadoVariables.mu[ 76 ]) );
acadoVariables.mu[68] = 0.0000000000000000e+00;
acadoVariables.mu[68] -= + acadoWorkspace.y[111]*acadoWorkspace.evHx[144] + acadoWorkspace.y[112]*acadoWorkspace.evHx[148];
acadoVariables.mu[69] = 0.0000000000000000e+00;
acadoVariables.mu[69] -= + acadoWorkspace.y[111]*acadoWorkspace.evHx[145] + acadoWorkspace.y[112]*acadoWorkspace.evHx[149];
acadoVariables.mu[70] = 0.0000000000000000e+00;
acadoVariables.mu[70] -= + acadoWorkspace.y[111]*acadoWorkspace.evHx[146] + acadoWorkspace.y[112]*acadoWorkspace.evHx[150];
acadoVariables.mu[71] = 0.0000000000000000e+00;
acadoVariables.mu[71] -= + acadoWorkspace.y[111]*acadoWorkspace.evHx[147] + acadoWorkspace.y[112]*acadoWorkspace.evHx[151];
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoVariables.mu[ 68 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[64] = 0.0000000000000000e+00;
acadoVariables.mu[64] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[136] + acadoWorkspace.y[110]*acadoWorkspace.evHx[140];
acadoVariables.mu[65] = 0.0000000000000000e+00;
acadoVariables.mu[65] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[137] + acadoWorkspace.y[110]*acadoWorkspace.evHx[141];
acadoVariables.mu[66] = 0.0000000000000000e+00;
acadoVariables.mu[66] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[138] + acadoWorkspace.y[110]*acadoWorkspace.evHx[142];
acadoVariables.mu[67] = 0.0000000000000000e+00;
acadoVariables.mu[67] -= + acadoWorkspace.y[109]*acadoWorkspace.evHx[139] + acadoWorkspace.y[110]*acadoWorkspace.evHx[143];
acado_expansionStep2( &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.S1[ 204 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoVariables.mu[ 64 ]), &(acadoVariables.mu[ 68 ]) );
acadoVariables.mu[60] = 0.0000000000000000e+00;
acadoVariables.mu[60] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[128] + acadoWorkspace.y[108]*acadoWorkspace.evHx[132];
acadoVariables.mu[61] = 0.0000000000000000e+00;
acadoVariables.mu[61] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[129] + acadoWorkspace.y[108]*acadoWorkspace.evHx[133];
acadoVariables.mu[62] = 0.0000000000000000e+00;
acadoVariables.mu[62] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[130] + acadoWorkspace.y[108]*acadoWorkspace.evHx[134];
acadoVariables.mu[63] = 0.0000000000000000e+00;
acadoVariables.mu[63] -= + acadoWorkspace.y[107]*acadoWorkspace.evHx[131] + acadoWorkspace.y[108]*acadoWorkspace.evHx[135];
acado_expansionStep2( &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.S1[ 192 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoVariables.mu[ 60 ]), &(acadoVariables.mu[ 64 ]) );
acadoVariables.mu[56] = 0.0000000000000000e+00;
acadoVariables.mu[56] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[120] + acadoWorkspace.y[106]*acadoWorkspace.evHx[124];
acadoVariables.mu[57] = 0.0000000000000000e+00;
acadoVariables.mu[57] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[121] + acadoWorkspace.y[106]*acadoWorkspace.evHx[125];
acadoVariables.mu[58] = 0.0000000000000000e+00;
acadoVariables.mu[58] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[122] + acadoWorkspace.y[106]*acadoWorkspace.evHx[126];
acadoVariables.mu[59] = 0.0000000000000000e+00;
acadoVariables.mu[59] -= + acadoWorkspace.y[105]*acadoWorkspace.evHx[123] + acadoWorkspace.y[106]*acadoWorkspace.evHx[127];
acado_expansionStep2( &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.S1[ 180 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoVariables.mu[ 56 ]), &(acadoVariables.mu[ 60 ]) );
acadoVariables.mu[52] = 0.0000000000000000e+00;
acadoVariables.mu[52] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[112] + acadoWorkspace.y[104]*acadoWorkspace.evHx[116];
acadoVariables.mu[53] = 0.0000000000000000e+00;
acadoVariables.mu[53] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[113] + acadoWorkspace.y[104]*acadoWorkspace.evHx[117];
acadoVariables.mu[54] = 0.0000000000000000e+00;
acadoVariables.mu[54] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[114] + acadoWorkspace.y[104]*acadoWorkspace.evHx[118];
acadoVariables.mu[55] = 0.0000000000000000e+00;
acadoVariables.mu[55] -= + acadoWorkspace.y[103]*acadoWorkspace.evHx[115] + acadoWorkspace.y[104]*acadoWorkspace.evHx[119];
acado_expansionStep2( &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.S1[ 168 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoVariables.mu[ 52 ]), &(acadoVariables.mu[ 56 ]) );
acadoVariables.mu[48] = 0.0000000000000000e+00;
acadoVariables.mu[48] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[104] + acadoWorkspace.y[102]*acadoWorkspace.evHx[108];
acadoVariables.mu[49] = 0.0000000000000000e+00;
acadoVariables.mu[49] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[105] + acadoWorkspace.y[102]*acadoWorkspace.evHx[109];
acadoVariables.mu[50] = 0.0000000000000000e+00;
acadoVariables.mu[50] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[106] + acadoWorkspace.y[102]*acadoWorkspace.evHx[110];
acadoVariables.mu[51] = 0.0000000000000000e+00;
acadoVariables.mu[51] -= + acadoWorkspace.y[101]*acadoWorkspace.evHx[107] + acadoWorkspace.y[102]*acadoWorkspace.evHx[111];
acado_expansionStep2( &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.S1[ 156 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoVariables.mu[ 48 ]), &(acadoVariables.mu[ 52 ]) );
acadoVariables.mu[44] = 0.0000000000000000e+00;
acadoVariables.mu[44] -= + acadoWorkspace.y[99]*acadoWorkspace.evHx[96] + acadoWorkspace.y[100]*acadoWorkspace.evHx[100];
acadoVariables.mu[45] = 0.0000000000000000e+00;
acadoVariables.mu[45] -= + acadoWorkspace.y[99]*acadoWorkspace.evHx[97] + acadoWorkspace.y[100]*acadoWorkspace.evHx[101];
acadoVariables.mu[46] = 0.0000000000000000e+00;
acadoVariables.mu[46] -= + acadoWorkspace.y[99]*acadoWorkspace.evHx[98] + acadoWorkspace.y[100]*acadoWorkspace.evHx[102];
acadoVariables.mu[47] = 0.0000000000000000e+00;
acadoVariables.mu[47] -= + acadoWorkspace.y[99]*acadoWorkspace.evHx[99] + acadoWorkspace.y[100]*acadoWorkspace.evHx[103];
acado_expansionStep2( &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoVariables.mu[ 44 ]), &(acadoVariables.mu[ 48 ]) );
acadoVariables.mu[40] = 0.0000000000000000e+00;
acadoVariables.mu[40] -= + acadoWorkspace.y[97]*acadoWorkspace.evHx[88] + acadoWorkspace.y[98]*acadoWorkspace.evHx[92];
acadoVariables.mu[41] = 0.0000000000000000e+00;
acadoVariables.mu[41] -= + acadoWorkspace.y[97]*acadoWorkspace.evHx[89] + acadoWorkspace.y[98]*acadoWorkspace.evHx[93];
acadoVariables.mu[42] = 0.0000000000000000e+00;
acadoVariables.mu[42] -= + acadoWorkspace.y[97]*acadoWorkspace.evHx[90] + acadoWorkspace.y[98]*acadoWorkspace.evHx[94];
acadoVariables.mu[43] = 0.0000000000000000e+00;
acadoVariables.mu[43] -= + acadoWorkspace.y[97]*acadoWorkspace.evHx[91] + acadoWorkspace.y[98]*acadoWorkspace.evHx[95];
acado_expansionStep2( &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.S1[ 132 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoVariables.mu[ 40 ]), &(acadoVariables.mu[ 44 ]) );
acadoVariables.mu[36] = 0.0000000000000000e+00;
acadoVariables.mu[36] -= + acadoWorkspace.y[95]*acadoWorkspace.evHx[80] + acadoWorkspace.y[96]*acadoWorkspace.evHx[84];
acadoVariables.mu[37] = 0.0000000000000000e+00;
acadoVariables.mu[37] -= + acadoWorkspace.y[95]*acadoWorkspace.evHx[81] + acadoWorkspace.y[96]*acadoWorkspace.evHx[85];
acadoVariables.mu[38] = 0.0000000000000000e+00;
acadoVariables.mu[38] -= + acadoWorkspace.y[95]*acadoWorkspace.evHx[82] + acadoWorkspace.y[96]*acadoWorkspace.evHx[86];
acadoVariables.mu[39] = 0.0000000000000000e+00;
acadoVariables.mu[39] -= + acadoWorkspace.y[95]*acadoWorkspace.evHx[83] + acadoWorkspace.y[96]*acadoWorkspace.evHx[87];
acado_expansionStep2( &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoVariables.mu[ 36 ]), &(acadoVariables.mu[ 40 ]) );
acadoVariables.mu[32] = 0.0000000000000000e+00;
acadoVariables.mu[32] -= + acadoWorkspace.y[93]*acadoWorkspace.evHx[72] + acadoWorkspace.y[94]*acadoWorkspace.evHx[76];
acadoVariables.mu[33] = 0.0000000000000000e+00;
acadoVariables.mu[33] -= + acadoWorkspace.y[93]*acadoWorkspace.evHx[73] + acadoWorkspace.y[94]*acadoWorkspace.evHx[77];
acadoVariables.mu[34] = 0.0000000000000000e+00;
acadoVariables.mu[34] -= + acadoWorkspace.y[93]*acadoWorkspace.evHx[74] + acadoWorkspace.y[94]*acadoWorkspace.evHx[78];
acadoVariables.mu[35] = 0.0000000000000000e+00;
acadoVariables.mu[35] -= + acadoWorkspace.y[93]*acadoWorkspace.evHx[75] + acadoWorkspace.y[94]*acadoWorkspace.evHx[79];
acado_expansionStep2( &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoVariables.mu[ 32 ]), &(acadoVariables.mu[ 36 ]) );
acadoVariables.mu[28] = 0.0000000000000000e+00;
acadoVariables.mu[28] -= + acadoWorkspace.y[91]*acadoWorkspace.evHx[64] + acadoWorkspace.y[92]*acadoWorkspace.evHx[68];
acadoVariables.mu[29] = 0.0000000000000000e+00;
acadoVariables.mu[29] -= + acadoWorkspace.y[91]*acadoWorkspace.evHx[65] + acadoWorkspace.y[92]*acadoWorkspace.evHx[69];
acadoVariables.mu[30] = 0.0000000000000000e+00;
acadoVariables.mu[30] -= + acadoWorkspace.y[91]*acadoWorkspace.evHx[66] + acadoWorkspace.y[92]*acadoWorkspace.evHx[70];
acadoVariables.mu[31] = 0.0000000000000000e+00;
acadoVariables.mu[31] -= + acadoWorkspace.y[91]*acadoWorkspace.evHx[67] + acadoWorkspace.y[92]*acadoWorkspace.evHx[71];
acado_expansionStep2( &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoVariables.mu[ 28 ]), &(acadoVariables.mu[ 32 ]) );
acadoVariables.mu[24] = 0.0000000000000000e+00;
acadoVariables.mu[24] -= + acadoWorkspace.y[89]*acadoWorkspace.evHx[56] + acadoWorkspace.y[90]*acadoWorkspace.evHx[60];
acadoVariables.mu[25] = 0.0000000000000000e+00;
acadoVariables.mu[25] -= + acadoWorkspace.y[89]*acadoWorkspace.evHx[57] + acadoWorkspace.y[90]*acadoWorkspace.evHx[61];
acadoVariables.mu[26] = 0.0000000000000000e+00;
acadoVariables.mu[26] -= + acadoWorkspace.y[89]*acadoWorkspace.evHx[58] + acadoWorkspace.y[90]*acadoWorkspace.evHx[62];
acadoVariables.mu[27] = 0.0000000000000000e+00;
acadoVariables.mu[27] -= + acadoWorkspace.y[89]*acadoWorkspace.evHx[59] + acadoWorkspace.y[90]*acadoWorkspace.evHx[63];
acado_expansionStep2( &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoVariables.mu[ 24 ]), &(acadoVariables.mu[ 28 ]) );
acadoVariables.mu[20] = 0.0000000000000000e+00;
acadoVariables.mu[20] -= + acadoWorkspace.y[87]*acadoWorkspace.evHx[48] + acadoWorkspace.y[88]*acadoWorkspace.evHx[52];
acadoVariables.mu[21] = 0.0000000000000000e+00;
acadoVariables.mu[21] -= + acadoWorkspace.y[87]*acadoWorkspace.evHx[49] + acadoWorkspace.y[88]*acadoWorkspace.evHx[53];
acadoVariables.mu[22] = 0.0000000000000000e+00;
acadoVariables.mu[22] -= + acadoWorkspace.y[87]*acadoWorkspace.evHx[50] + acadoWorkspace.y[88]*acadoWorkspace.evHx[54];
acadoVariables.mu[23] = 0.0000000000000000e+00;
acadoVariables.mu[23] -= + acadoWorkspace.y[87]*acadoWorkspace.evHx[51] + acadoWorkspace.y[88]*acadoWorkspace.evHx[55];
acado_expansionStep2( &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoVariables.mu[ 20 ]), &(acadoVariables.mu[ 24 ]) );
acadoVariables.mu[16] = 0.0000000000000000e+00;
acadoVariables.mu[16] -= + acadoWorkspace.y[85]*acadoWorkspace.evHx[40] + acadoWorkspace.y[86]*acadoWorkspace.evHx[44];
acadoVariables.mu[17] = 0.0000000000000000e+00;
acadoVariables.mu[17] -= + acadoWorkspace.y[85]*acadoWorkspace.evHx[41] + acadoWorkspace.y[86]*acadoWorkspace.evHx[45];
acadoVariables.mu[18] = 0.0000000000000000e+00;
acadoVariables.mu[18] -= + acadoWorkspace.y[85]*acadoWorkspace.evHx[42] + acadoWorkspace.y[86]*acadoWorkspace.evHx[46];
acadoVariables.mu[19] = 0.0000000000000000e+00;
acadoVariables.mu[19] -= + acadoWorkspace.y[85]*acadoWorkspace.evHx[43] + acadoWorkspace.y[86]*acadoWorkspace.evHx[47];
acado_expansionStep2( &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoVariables.mu[ 16 ]), &(acadoVariables.mu[ 20 ]) );
acadoVariables.mu[12] = 0.0000000000000000e+00;
acadoVariables.mu[12] -= + acadoWorkspace.y[83]*acadoWorkspace.evHx[32] + acadoWorkspace.y[84]*acadoWorkspace.evHx[36];
acadoVariables.mu[13] = 0.0000000000000000e+00;
acadoVariables.mu[13] -= + acadoWorkspace.y[83]*acadoWorkspace.evHx[33] + acadoWorkspace.y[84]*acadoWorkspace.evHx[37];
acadoVariables.mu[14] = 0.0000000000000000e+00;
acadoVariables.mu[14] -= + acadoWorkspace.y[83]*acadoWorkspace.evHx[34] + acadoWorkspace.y[84]*acadoWorkspace.evHx[38];
acadoVariables.mu[15] = 0.0000000000000000e+00;
acadoVariables.mu[15] -= + acadoWorkspace.y[83]*acadoWorkspace.evHx[35] + acadoWorkspace.y[84]*acadoWorkspace.evHx[39];
acado_expansionStep2( &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoVariables.mu[ 12 ]), &(acadoVariables.mu[ 16 ]) );
acadoVariables.mu[8] = 0.0000000000000000e+00;
acadoVariables.mu[8] -= + acadoWorkspace.y[81]*acadoWorkspace.evHx[24] + acadoWorkspace.y[82]*acadoWorkspace.evHx[28];
acadoVariables.mu[9] = 0.0000000000000000e+00;
acadoVariables.mu[9] -= + acadoWorkspace.y[81]*acadoWorkspace.evHx[25] + acadoWorkspace.y[82]*acadoWorkspace.evHx[29];
acadoVariables.mu[10] = 0.0000000000000000e+00;
acadoVariables.mu[10] -= + acadoWorkspace.y[81]*acadoWorkspace.evHx[26] + acadoWorkspace.y[82]*acadoWorkspace.evHx[30];
acadoVariables.mu[11] = 0.0000000000000000e+00;
acadoVariables.mu[11] -= + acadoWorkspace.y[81]*acadoWorkspace.evHx[27] + acadoWorkspace.y[82]*acadoWorkspace.evHx[31];
acado_expansionStep2( &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoVariables.mu[ 8 ]), &(acadoVariables.mu[ 12 ]) );
acadoVariables.mu[4] = 0.0000000000000000e+00;
acadoVariables.mu[4] -= + acadoWorkspace.y[79]*acadoWorkspace.evHx[16] + acadoWorkspace.y[80]*acadoWorkspace.evHx[20];
acadoVariables.mu[5] = 0.0000000000000000e+00;
acadoVariables.mu[5] -= + acadoWorkspace.y[79]*acadoWorkspace.evHx[17] + acadoWorkspace.y[80]*acadoWorkspace.evHx[21];
acadoVariables.mu[6] = 0.0000000000000000e+00;
acadoVariables.mu[6] -= + acadoWorkspace.y[79]*acadoWorkspace.evHx[18] + acadoWorkspace.y[80]*acadoWorkspace.evHx[22];
acadoVariables.mu[7] = 0.0000000000000000e+00;
acadoVariables.mu[7] -= + acadoWorkspace.y[79]*acadoWorkspace.evHx[19] + acadoWorkspace.y[80]*acadoWorkspace.evHx[23];
acado_expansionStep2( &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoVariables.mu[ 4 ]), &(acadoVariables.mu[ 8 ]) );
acadoVariables.mu[0] = 0.0000000000000000e+00;
acadoVariables.mu[0] -= + acadoWorkspace.y[77]*acadoWorkspace.evHx[8] + acadoWorkspace.y[78]*acadoWorkspace.evHx[12];
acadoVariables.mu[1] = 0.0000000000000000e+00;
acadoVariables.mu[1] -= + acadoWorkspace.y[77]*acadoWorkspace.evHx[9] + acadoWorkspace.y[78]*acadoWorkspace.evHx[13];
acadoVariables.mu[2] = 0.0000000000000000e+00;
acadoVariables.mu[2] -= + acadoWorkspace.y[77]*acadoWorkspace.evHx[10] + acadoWorkspace.y[78]*acadoWorkspace.evHx[14];
acadoVariables.mu[3] = 0.0000000000000000e+00;
acadoVariables.mu[3] -= + acadoWorkspace.y[77]*acadoWorkspace.evHx[11] + acadoWorkspace.y[78]*acadoWorkspace.evHx[15];
acado_expansionStep2( &(acadoWorkspace.QDy[ 4 ]), &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.S1[ 12 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.evGx[ 16 ]), acadoVariables.mu, &(acadoVariables.mu[ 4 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_regularizeHessian(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -5.0000000000000000e-01;
acadoVariables.lbValues[1] = -5.0000000000000000e-01;
acadoVariables.lbValues[2] = -1.0000000000000000e+12;
acadoVariables.lbValues[3] = -5.0000000000000000e-01;
acadoVariables.lbValues[4] = -5.0000000000000000e-01;
acadoVariables.lbValues[5] = -1.0000000000000000e+12;
acadoVariables.lbValues[6] = -5.0000000000000000e-01;
acadoVariables.lbValues[7] = -5.0000000000000000e-01;
acadoVariables.lbValues[8] = -1.0000000000000000e+12;
acadoVariables.lbValues[9] = -5.0000000000000000e-01;
acadoVariables.lbValues[10] = -5.0000000000000000e-01;
acadoVariables.lbValues[11] = -1.0000000000000000e+12;
acadoVariables.lbValues[12] = -5.0000000000000000e-01;
acadoVariables.lbValues[13] = -5.0000000000000000e-01;
acadoVariables.lbValues[14] = -1.0000000000000000e+12;
acadoVariables.lbValues[15] = -5.0000000000000000e-01;
acadoVariables.lbValues[16] = -5.0000000000000000e-01;
acadoVariables.lbValues[17] = -1.0000000000000000e+12;
acadoVariables.lbValues[18] = -5.0000000000000000e-01;
acadoVariables.lbValues[19] = -5.0000000000000000e-01;
acadoVariables.lbValues[20] = -1.0000000000000000e+12;
acadoVariables.lbValues[21] = -5.0000000000000000e-01;
acadoVariables.lbValues[22] = -5.0000000000000000e-01;
acadoVariables.lbValues[23] = -1.0000000000000000e+12;
acadoVariables.lbValues[24] = -5.0000000000000000e-01;
acadoVariables.lbValues[25] = -5.0000000000000000e-01;
acadoVariables.lbValues[26] = -1.0000000000000000e+12;
acadoVariables.lbValues[27] = -5.0000000000000000e-01;
acadoVariables.lbValues[28] = -5.0000000000000000e-01;
acadoVariables.lbValues[29] = -1.0000000000000000e+12;
acadoVariables.lbValues[30] = -5.0000000000000000e-01;
acadoVariables.lbValues[31] = -5.0000000000000000e-01;
acadoVariables.lbValues[32] = -1.0000000000000000e+12;
acadoVariables.lbValues[33] = -5.0000000000000000e-01;
acadoVariables.lbValues[34] = -5.0000000000000000e-01;
acadoVariables.lbValues[35] = -1.0000000000000000e+12;
acadoVariables.lbValues[36] = -5.0000000000000000e-01;
acadoVariables.lbValues[37] = -5.0000000000000000e-01;
acadoVariables.lbValues[38] = -1.0000000000000000e+12;
acadoVariables.lbValues[39] = -5.0000000000000000e-01;
acadoVariables.lbValues[40] = -5.0000000000000000e-01;
acadoVariables.lbValues[41] = -1.0000000000000000e+12;
acadoVariables.lbValues[42] = -5.0000000000000000e-01;
acadoVariables.lbValues[43] = -5.0000000000000000e-01;
acadoVariables.lbValues[44] = -1.0000000000000000e+12;
acadoVariables.lbValues[45] = -5.0000000000000000e-01;
acadoVariables.lbValues[46] = -5.0000000000000000e-01;
acadoVariables.lbValues[47] = -1.0000000000000000e+12;
acadoVariables.lbValues[48] = -5.0000000000000000e-01;
acadoVariables.lbValues[49] = -5.0000000000000000e-01;
acadoVariables.lbValues[50] = -1.0000000000000000e+12;
acadoVariables.lbValues[51] = -5.0000000000000000e-01;
acadoVariables.lbValues[52] = -5.0000000000000000e-01;
acadoVariables.lbValues[53] = -1.0000000000000000e+12;
acadoVariables.lbValues[54] = -5.0000000000000000e-01;
acadoVariables.lbValues[55] = -5.0000000000000000e-01;
acadoVariables.lbValues[56] = -1.0000000000000000e+12;
acadoVariables.lbValues[57] = -5.0000000000000000e-01;
acadoVariables.lbValues[58] = -5.0000000000000000e-01;
acadoVariables.lbValues[59] = -1.0000000000000000e+12;
acadoVariables.lbValues[60] = -5.0000000000000000e-01;
acadoVariables.lbValues[61] = -5.0000000000000000e-01;
acadoVariables.lbValues[62] = -1.0000000000000000e+12;
acadoVariables.lbValues[63] = -5.0000000000000000e-01;
acadoVariables.lbValues[64] = -5.0000000000000000e-01;
acadoVariables.lbValues[65] = -1.0000000000000000e+12;
acadoVariables.lbValues[66] = -5.0000000000000000e-01;
acadoVariables.lbValues[67] = -5.0000000000000000e-01;
acadoVariables.lbValues[68] = -1.0000000000000000e+12;
acadoVariables.lbValues[69] = -5.0000000000000000e-01;
acadoVariables.lbValues[70] = -5.0000000000000000e-01;
acadoVariables.lbValues[71] = -1.0000000000000000e+12;
acadoVariables.lbValues[72] = -5.0000000000000000e-01;
acadoVariables.lbValues[73] = -5.0000000000000000e-01;
acadoVariables.lbValues[74] = -1.0000000000000000e+12;
acadoVariables.ubValues[0] = 5.0000000000000000e-01;
acadoVariables.ubValues[1] = 5.0000000000000000e-01;
acadoVariables.ubValues[2] = 1.0000000000000000e+12;
acadoVariables.ubValues[3] = 5.0000000000000000e-01;
acadoVariables.ubValues[4] = 5.0000000000000000e-01;
acadoVariables.ubValues[5] = 1.0000000000000000e+12;
acadoVariables.ubValues[6] = 5.0000000000000000e-01;
acadoVariables.ubValues[7] = 5.0000000000000000e-01;
acadoVariables.ubValues[8] = 1.0000000000000000e+12;
acadoVariables.ubValues[9] = 5.0000000000000000e-01;
acadoVariables.ubValues[10] = 5.0000000000000000e-01;
acadoVariables.ubValues[11] = 1.0000000000000000e+12;
acadoVariables.ubValues[12] = 5.0000000000000000e-01;
acadoVariables.ubValues[13] = 5.0000000000000000e-01;
acadoVariables.ubValues[14] = 1.0000000000000000e+12;
acadoVariables.ubValues[15] = 5.0000000000000000e-01;
acadoVariables.ubValues[16] = 5.0000000000000000e-01;
acadoVariables.ubValues[17] = 1.0000000000000000e+12;
acadoVariables.ubValues[18] = 5.0000000000000000e-01;
acadoVariables.ubValues[19] = 5.0000000000000000e-01;
acadoVariables.ubValues[20] = 1.0000000000000000e+12;
acadoVariables.ubValues[21] = 5.0000000000000000e-01;
acadoVariables.ubValues[22] = 5.0000000000000000e-01;
acadoVariables.ubValues[23] = 1.0000000000000000e+12;
acadoVariables.ubValues[24] = 5.0000000000000000e-01;
acadoVariables.ubValues[25] = 5.0000000000000000e-01;
acadoVariables.ubValues[26] = 1.0000000000000000e+12;
acadoVariables.ubValues[27] = 5.0000000000000000e-01;
acadoVariables.ubValues[28] = 5.0000000000000000e-01;
acadoVariables.ubValues[29] = 1.0000000000000000e+12;
acadoVariables.ubValues[30] = 5.0000000000000000e-01;
acadoVariables.ubValues[31] = 5.0000000000000000e-01;
acadoVariables.ubValues[32] = 1.0000000000000000e+12;
acadoVariables.ubValues[33] = 5.0000000000000000e-01;
acadoVariables.ubValues[34] = 5.0000000000000000e-01;
acadoVariables.ubValues[35] = 1.0000000000000000e+12;
acadoVariables.ubValues[36] = 5.0000000000000000e-01;
acadoVariables.ubValues[37] = 5.0000000000000000e-01;
acadoVariables.ubValues[38] = 1.0000000000000000e+12;
acadoVariables.ubValues[39] = 5.0000000000000000e-01;
acadoVariables.ubValues[40] = 5.0000000000000000e-01;
acadoVariables.ubValues[41] = 1.0000000000000000e+12;
acadoVariables.ubValues[42] = 5.0000000000000000e-01;
acadoVariables.ubValues[43] = 5.0000000000000000e-01;
acadoVariables.ubValues[44] = 1.0000000000000000e+12;
acadoVariables.ubValues[45] = 5.0000000000000000e-01;
acadoVariables.ubValues[46] = 5.0000000000000000e-01;
acadoVariables.ubValues[47] = 1.0000000000000000e+12;
acadoVariables.ubValues[48] = 5.0000000000000000e-01;
acadoVariables.ubValues[49] = 5.0000000000000000e-01;
acadoVariables.ubValues[50] = 1.0000000000000000e+12;
acadoVariables.ubValues[51] = 5.0000000000000000e-01;
acadoVariables.ubValues[52] = 5.0000000000000000e-01;
acadoVariables.ubValues[53] = 1.0000000000000000e+12;
acadoVariables.ubValues[54] = 5.0000000000000000e-01;
acadoVariables.ubValues[55] = 5.0000000000000000e-01;
acadoVariables.ubValues[56] = 1.0000000000000000e+12;
acadoVariables.ubValues[57] = 5.0000000000000000e-01;
acadoVariables.ubValues[58] = 5.0000000000000000e-01;
acadoVariables.ubValues[59] = 1.0000000000000000e+12;
acadoVariables.ubValues[60] = 5.0000000000000000e-01;
acadoVariables.ubValues[61] = 5.0000000000000000e-01;
acadoVariables.ubValues[62] = 1.0000000000000000e+12;
acadoVariables.ubValues[63] = 5.0000000000000000e-01;
acadoVariables.ubValues[64] = 5.0000000000000000e-01;
acadoVariables.ubValues[65] = 1.0000000000000000e+12;
acadoVariables.ubValues[66] = 5.0000000000000000e-01;
acadoVariables.ubValues[67] = 5.0000000000000000e-01;
acadoVariables.ubValues[68] = 1.0000000000000000e+12;
acadoVariables.ubValues[69] = 5.0000000000000000e-01;
acadoVariables.ubValues[70] = 5.0000000000000000e-01;
acadoVariables.ubValues[71] = 1.0000000000000000e+12;
acadoVariables.ubValues[72] = 5.0000000000000000e-01;
acadoVariables.ubValues[73] = 5.0000000000000000e-01;
acadoVariables.ubValues[74] = 1.0000000000000000e+12;
acadoVariables.lbAValues[0] = 1.0000000000000000e+00;
acadoVariables.lbAValues[1] = 1.0000000000000000e+00;
acadoVariables.lbAValues[2] = 1.0000000000000000e+00;
acadoVariables.lbAValues[3] = 1.0000000000000000e+00;
acadoVariables.lbAValues[4] = 1.0000000000000000e+00;
acadoVariables.lbAValues[5] = 1.0000000000000000e+00;
acadoVariables.lbAValues[6] = 1.0000000000000000e+00;
acadoVariables.lbAValues[7] = 1.0000000000000000e+00;
acadoVariables.lbAValues[8] = 1.0000000000000000e+00;
acadoVariables.lbAValues[9] = 1.0000000000000000e+00;
acadoVariables.lbAValues[10] = 1.0000000000000000e+00;
acadoVariables.lbAValues[11] = 1.0000000000000000e+00;
acadoVariables.lbAValues[12] = 1.0000000000000000e+00;
acadoVariables.lbAValues[13] = 1.0000000000000000e+00;
acadoVariables.lbAValues[14] = 1.0000000000000000e+00;
acadoVariables.lbAValues[15] = 1.0000000000000000e+00;
acadoVariables.lbAValues[16] = 1.0000000000000000e+00;
acadoVariables.lbAValues[17] = 1.0000000000000000e+00;
acadoVariables.lbAValues[18] = 1.0000000000000000e+00;
acadoVariables.lbAValues[19] = 1.0000000000000000e+00;
acadoVariables.lbAValues[20] = 1.0000000000000000e+00;
acadoVariables.lbAValues[21] = 1.0000000000000000e+00;
acadoVariables.lbAValues[22] = 1.0000000000000000e+00;
acadoVariables.lbAValues[23] = 1.0000000000000000e+00;
acadoVariables.lbAValues[24] = 1.0000000000000000e+00;
acadoVariables.lbAValues[25] = 1.0000000000000000e+00;
acadoVariables.lbAValues[26] = 1.0000000000000000e+00;
acadoVariables.lbAValues[27] = 1.0000000000000000e+00;
acadoVariables.lbAValues[28] = 1.0000000000000000e+00;
acadoVariables.lbAValues[29] = 1.0000000000000000e+00;
acadoVariables.lbAValues[30] = 1.0000000000000000e+00;
acadoVariables.lbAValues[31] = 1.0000000000000000e+00;
acadoVariables.lbAValues[32] = 1.0000000000000000e+00;
acadoVariables.lbAValues[33] = 1.0000000000000000e+00;
acadoVariables.lbAValues[34] = 1.0000000000000000e+00;
acadoVariables.lbAValues[35] = 1.0000000000000000e+00;
acadoVariables.lbAValues[36] = 1.0000000000000000e+00;
acadoVariables.lbAValues[37] = 1.0000000000000000e+00;
acadoVariables.lbAValues[38] = 1.0000000000000000e+00;
acadoVariables.lbAValues[39] = 1.0000000000000000e+00;
acadoVariables.lbAValues[40] = 1.0000000000000000e+00;
acadoVariables.lbAValues[41] = 1.0000000000000000e+00;
acadoVariables.lbAValues[42] = 1.0000000000000000e+00;
acadoVariables.lbAValues[43] = 1.0000000000000000e+00;
acadoVariables.lbAValues[44] = 1.0000000000000000e+00;
acadoVariables.lbAValues[45] = 1.0000000000000000e+00;
acadoVariables.lbAValues[46] = 1.0000000000000000e+00;
acadoVariables.lbAValues[47] = 1.0000000000000000e+00;
acadoVariables.lbAValues[48] = 1.0000000000000000e+00;
acadoVariables.lbAValues[49] = 1.0000000000000000e+00;
acadoVariables.ubAValues[0] = 1.0000000000000000e+12;
acadoVariables.ubAValues[1] = 1.0000000000000000e+12;
acadoVariables.ubAValues[2] = 1.0000000000000000e+12;
acadoVariables.ubAValues[3] = 1.0000000000000000e+12;
acadoVariables.ubAValues[4] = 1.0000000000000000e+12;
acadoVariables.ubAValues[5] = 1.0000000000000000e+12;
acadoVariables.ubAValues[6] = 1.0000000000000000e+12;
acadoVariables.ubAValues[7] = 1.0000000000000000e+12;
acadoVariables.ubAValues[8] = 1.0000000000000000e+12;
acadoVariables.ubAValues[9] = 1.0000000000000000e+12;
acadoVariables.ubAValues[10] = 1.0000000000000000e+12;
acadoVariables.ubAValues[11] = 1.0000000000000000e+12;
acadoVariables.ubAValues[12] = 1.0000000000000000e+12;
acadoVariables.ubAValues[13] = 1.0000000000000000e+12;
acadoVariables.ubAValues[14] = 1.0000000000000000e+12;
acadoVariables.ubAValues[15] = 1.0000000000000000e+12;
acadoVariables.ubAValues[16] = 1.0000000000000000e+12;
acadoVariables.ubAValues[17] = 1.0000000000000000e+12;
acadoVariables.ubAValues[18] = 1.0000000000000000e+12;
acadoVariables.ubAValues[19] = 1.0000000000000000e+12;
acadoVariables.ubAValues[20] = 1.0000000000000000e+12;
acadoVariables.ubAValues[21] = 1.0000000000000000e+12;
acadoVariables.ubAValues[22] = 1.0000000000000000e+12;
acadoVariables.ubAValues[23] = 1.0000000000000000e+12;
acadoVariables.ubAValues[24] = 1.0000000000000000e+12;
acadoVariables.ubAValues[25] = 1.0000000000000000e+12;
acadoVariables.ubAValues[26] = 1.0000000000000000e+12;
acadoVariables.ubAValues[27] = 1.0000000000000000e+12;
acadoVariables.ubAValues[28] = 1.0000000000000000e+12;
acadoVariables.ubAValues[29] = 1.0000000000000000e+12;
acadoVariables.ubAValues[30] = 1.0000000000000000e+12;
acadoVariables.ubAValues[31] = 1.0000000000000000e+12;
acadoVariables.ubAValues[32] = 1.0000000000000000e+12;
acadoVariables.ubAValues[33] = 1.0000000000000000e+12;
acadoVariables.ubAValues[34] = 1.0000000000000000e+12;
acadoVariables.ubAValues[35] = 1.0000000000000000e+12;
acadoVariables.ubAValues[36] = 1.0000000000000000e+12;
acadoVariables.ubAValues[37] = 1.0000000000000000e+12;
acadoVariables.ubAValues[38] = 1.0000000000000000e+12;
acadoVariables.ubAValues[39] = 1.0000000000000000e+12;
acadoVariables.ubAValues[40] = 1.0000000000000000e+12;
acadoVariables.ubAValues[41] = 1.0000000000000000e+12;
acadoVariables.ubAValues[42] = 1.0000000000000000e+12;
acadoVariables.ubAValues[43] = 1.0000000000000000e+12;
acadoVariables.ubAValues[44] = 1.0000000000000000e+12;
acadoVariables.ubAValues[45] = 1.0000000000000000e+12;
acadoVariables.ubAValues[46] = 1.0000000000000000e+12;
acadoVariables.ubAValues[47] = 1.0000000000000000e+12;
acadoVariables.ubAValues[48] = 1.0000000000000000e+12;
acadoVariables.ubAValues[49] = 1.0000000000000000e+12;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[64] = acadoVariables.u[index * 3];
acadoWorkspace.state[65] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[66] = acadoVariables.u[index * 3 + 2];
acadoWorkspace.state[67] = acadoVariables.od[index * 25];
acadoWorkspace.state[68] = acadoVariables.od[index * 25 + 1];
acadoWorkspace.state[69] = acadoVariables.od[index * 25 + 2];
acadoWorkspace.state[70] = acadoVariables.od[index * 25 + 3];
acadoWorkspace.state[71] = acadoVariables.od[index * 25 + 4];
acadoWorkspace.state[72] = acadoVariables.od[index * 25 + 5];
acadoWorkspace.state[73] = acadoVariables.od[index * 25 + 6];
acadoWorkspace.state[74] = acadoVariables.od[index * 25 + 7];
acadoWorkspace.state[75] = acadoVariables.od[index * 25 + 8];
acadoWorkspace.state[76] = acadoVariables.od[index * 25 + 9];
acadoWorkspace.state[77] = acadoVariables.od[index * 25 + 10];
acadoWorkspace.state[78] = acadoVariables.od[index * 25 + 11];
acadoWorkspace.state[79] = acadoVariables.od[index * 25 + 12];
acadoWorkspace.state[80] = acadoVariables.od[index * 25 + 13];
acadoWorkspace.state[81] = acadoVariables.od[index * 25 + 14];
acadoWorkspace.state[82] = acadoVariables.od[index * 25 + 15];
acadoWorkspace.state[83] = acadoVariables.od[index * 25 + 16];
acadoWorkspace.state[84] = acadoVariables.od[index * 25 + 17];
acadoWorkspace.state[85] = acadoVariables.od[index * 25 + 18];
acadoWorkspace.state[86] = acadoVariables.od[index * 25 + 19];
acadoWorkspace.state[87] = acadoVariables.od[index * 25 + 20];
acadoWorkspace.state[88] = acadoVariables.od[index * 25 + 21];
acadoWorkspace.state[89] = acadoVariables.od[index * 25 + 22];
acadoWorkspace.state[90] = acadoVariables.od[index * 25 + 23];
acadoWorkspace.state[91] = acadoVariables.od[index * 25 + 24];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[100] = xEnd[0];
acadoVariables.x[101] = xEnd[1];
acadoVariables.x[102] = xEnd[2];
acadoVariables.x[103] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[100];
acadoWorkspace.state[1] = acadoVariables.x[101];
acadoWorkspace.state[2] = acadoVariables.x[102];
acadoWorkspace.state[3] = acadoVariables.x[103];
if (uEnd != 0)
{
acadoWorkspace.state[64] = uEnd[0];
acadoWorkspace.state[65] = uEnd[1];
acadoWorkspace.state[66] = uEnd[2];
}
else
{
acadoWorkspace.state[64] = acadoVariables.u[72];
acadoWorkspace.state[65] = acadoVariables.u[73];
acadoWorkspace.state[66] = acadoVariables.u[74];
}
acadoWorkspace.state[67] = acadoVariables.od[625];
acadoWorkspace.state[68] = acadoVariables.od[626];
acadoWorkspace.state[69] = acadoVariables.od[627];
acadoWorkspace.state[70] = acadoVariables.od[628];
acadoWorkspace.state[71] = acadoVariables.od[629];
acadoWorkspace.state[72] = acadoVariables.od[630];
acadoWorkspace.state[73] = acadoVariables.od[631];
acadoWorkspace.state[74] = acadoVariables.od[632];
acadoWorkspace.state[75] = acadoVariables.od[633];
acadoWorkspace.state[76] = acadoVariables.od[634];
acadoWorkspace.state[77] = acadoVariables.od[635];
acadoWorkspace.state[78] = acadoVariables.od[636];
acadoWorkspace.state[79] = acadoVariables.od[637];
acadoWorkspace.state[80] = acadoVariables.od[638];
acadoWorkspace.state[81] = acadoVariables.od[639];
acadoWorkspace.state[82] = acadoVariables.od[640];
acadoWorkspace.state[83] = acadoVariables.od[641];
acadoWorkspace.state[84] = acadoVariables.od[642];
acadoWorkspace.state[85] = acadoVariables.od[643];
acadoWorkspace.state[86] = acadoVariables.od[644];
acadoWorkspace.state[87] = acadoVariables.od[645];
acadoWorkspace.state[88] = acadoVariables.od[646];
acadoWorkspace.state[89] = acadoVariables.od[647];
acadoWorkspace.state[90] = acadoVariables.od[648];
acadoWorkspace.state[91] = acadoVariables.od[649];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[100] = acadoWorkspace.state[0];
acadoVariables.x[101] = acadoWorkspace.state[1];
acadoVariables.x[102] = acadoWorkspace.state[2];
acadoVariables.x[103] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 24; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[72] = uEnd[0];
acadoVariables.u[73] = uEnd[1];
acadoVariables.u[74] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74];
kkt = fabs( kkt );
for (index = 0; index < 75; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 50; ++index)
{
prd = acadoWorkspace.y[index + 75];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 25];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 25 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 25 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 25 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 25 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 25 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 25 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 25 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 25 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 25 + 9];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 25 + 10];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 25 + 11];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 25 + 12];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 25 + 13];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 25 + 14];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 25 + 15];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 25 + 16];
acadoWorkspace.objValueIn[24] = acadoVariables.od[lRun1 * 25 + 17];
acadoWorkspace.objValueIn[25] = acadoVariables.od[lRun1 * 25 + 18];
acadoWorkspace.objValueIn[26] = acadoVariables.od[lRun1 * 25 + 19];
acadoWorkspace.objValueIn[27] = acadoVariables.od[lRun1 * 25 + 20];
acadoWorkspace.objValueIn[28] = acadoVariables.od[lRun1 * 25 + 21];
acadoWorkspace.objValueIn[29] = acadoVariables.od[lRun1 * 25 + 22];
acadoWorkspace.objValueIn[30] = acadoVariables.od[lRun1 * 25 + 23];
acadoWorkspace.objValueIn[31] = acadoVariables.od[lRun1 * 25 + 24];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[100];
acadoWorkspace.objValueIn[1] = acadoVariables.x[101];
acadoWorkspace.objValueIn[2] = acadoVariables.x[102];
acadoWorkspace.objValueIn[3] = acadoVariables.x[103];
acadoWorkspace.objValueIn[4] = acadoVariables.od[625];
acadoWorkspace.objValueIn[5] = acadoVariables.od[626];
acadoWorkspace.objValueIn[6] = acadoVariables.od[627];
acadoWorkspace.objValueIn[7] = acadoVariables.od[628];
acadoWorkspace.objValueIn[8] = acadoVariables.od[629];
acadoWorkspace.objValueIn[9] = acadoVariables.od[630];
acadoWorkspace.objValueIn[10] = acadoVariables.od[631];
acadoWorkspace.objValueIn[11] = acadoVariables.od[632];
acadoWorkspace.objValueIn[12] = acadoVariables.od[633];
acadoWorkspace.objValueIn[13] = acadoVariables.od[634];
acadoWorkspace.objValueIn[14] = acadoVariables.od[635];
acadoWorkspace.objValueIn[15] = acadoVariables.od[636];
acadoWorkspace.objValueIn[16] = acadoVariables.od[637];
acadoWorkspace.objValueIn[17] = acadoVariables.od[638];
acadoWorkspace.objValueIn[18] = acadoVariables.od[639];
acadoWorkspace.objValueIn[19] = acadoVariables.od[640];
acadoWorkspace.objValueIn[20] = acadoVariables.od[641];
acadoWorkspace.objValueIn[21] = acadoVariables.od[642];
acadoWorkspace.objValueIn[22] = acadoVariables.od[643];
acadoWorkspace.objValueIn[23] = acadoVariables.od[644];
acadoWorkspace.objValueIn[24] = acadoVariables.od[645];
acadoWorkspace.objValueIn[25] = acadoVariables.od[646];
acadoWorkspace.objValueIn[26] = acadoVariables.od[647];
acadoWorkspace.objValueIn[27] = acadoVariables.od[648];
acadoWorkspace.objValueIn[28] = acadoVariables.od[649];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
return objVal;
}

