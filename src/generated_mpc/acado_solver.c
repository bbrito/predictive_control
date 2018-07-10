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
for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
acadoWorkspace.state[67] = acadoVariables.od[lRun1 * 19];
acadoWorkspace.state[68] = acadoVariables.od[lRun1 * 19 + 1];
acadoWorkspace.state[69] = acadoVariables.od[lRun1 * 19 + 2];
acadoWorkspace.state[70] = acadoVariables.od[lRun1 * 19 + 3];
acadoWorkspace.state[71] = acadoVariables.od[lRun1 * 19 + 4];
acadoWorkspace.state[72] = acadoVariables.od[lRun1 * 19 + 5];
acadoWorkspace.state[73] = acadoVariables.od[lRun1 * 19 + 6];
acadoWorkspace.state[74] = acadoVariables.od[lRun1 * 19 + 7];
acadoWorkspace.state[75] = acadoVariables.od[lRun1 * 19 + 8];
acadoWorkspace.state[76] = acadoVariables.od[lRun1 * 19 + 9];
acadoWorkspace.state[77] = acadoVariables.od[lRun1 * 19 + 10];
acadoWorkspace.state[78] = acadoVariables.od[lRun1 * 19 + 11];
acadoWorkspace.state[79] = acadoVariables.od[lRun1 * 19 + 12];
acadoWorkspace.state[80] = acadoVariables.od[lRun1 * 19 + 13];
acadoWorkspace.state[81] = acadoVariables.od[lRun1 * 19 + 14];
acadoWorkspace.state[82] = acadoVariables.od[lRun1 * 19 + 15];
acadoWorkspace.state[83] = acadoVariables.od[lRun1 * 19 + 16];
acadoWorkspace.state[84] = acadoVariables.od[lRun1 * 19 + 17];
acadoWorkspace.state[85] = acadoVariables.od[lRun1 * 19 + 18];

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
/* Vector of auxiliary variables; number of elements: 50. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (xd[0]-od[0]);
a[1] = (od[3]*a[0]);
a[2] = (od[3]*(xd[0]-od[0]));
a[3] = (a[1]+a[2]);
a[4] = (xd[1]-od[1]);
a[5] = (od[4]*a[4]);
a[6] = (od[4]*(xd[1]-od[1]));
a[7] = (a[5]+a[6]);
a[8] = (xd[2]-od[2]);
a[9] = (od[5]*a[8]);
a[10] = (od[5]*(xd[2]-od[2]));
a[11] = (a[9]+a[10]);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (od[6]*u[0]);
a[14] = (od[6]*u[0]);
a[15] = (a[13]+a[14]);
a[16] = (od[7]*u[1]);
a[17] = (od[7]*u[1]);
a[18] = (a[16]+a[17]);
a[19] = (od[11]*u[2]);
a[20] = (od[11]*u[2]);
a[21] = (a[19]+a[20]);
a[22] = (od[3]*(real_t)(2.0000000000000000e+00));
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (od[4]*(real_t)(2.0000000000000000e+00));
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (od[5]*(real_t)(2.0000000000000000e+00));
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = (real_t)(0.0000000000000000e+00);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = (real_t)(0.0000000000000000e+00);
a[37] = (real_t)(0.0000000000000000e+00);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = (od[6]*(real_t)(2.0000000000000000e+00));
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (od[7]*(real_t)(2.0000000000000000e+00));
a[48] = (real_t)(0.0000000000000000e+00);
a[49] = (od[11]*(real_t)(2.0000000000000000e+00));

/* Compute outputs: */
out[0] = (((((((od[3]*(xd[0]-od[0]))*(xd[0]-od[0]))+((od[4]*(xd[1]-od[1]))*(xd[1]-od[1])))+((od[5]*(xd[2]-od[2]))*(xd[2]-od[2])))+((od[6]*u[0])*u[0]))+((od[7]*u[1])*u[1]))+((od[11]*u[2])*u[2]));
out[1] = a[3];
out[2] = a[7];
out[3] = a[11];
out[4] = a[12];
out[5] = a[15];
out[6] = a[18];
out[7] = a[21];
out[8] = a[22];
out[9] = a[23];
out[10] = a[24];
out[11] = a[25];
out[12] = a[23];
out[13] = a[26];
out[14] = a[27];
out[15] = a[28];
out[16] = a[24];
out[17] = a[27];
out[18] = a[29];
out[19] = a[30];
out[20] = a[25];
out[21] = a[28];
out[22] = a[30];
out[23] = a[31];
out[24] = a[32];
out[25] = a[33];
out[26] = a[34];
out[27] = a[35];
out[28] = a[36];
out[29] = a[37];
out[30] = a[38];
out[31] = a[39];
out[32] = a[40];
out[33] = a[41];
out[34] = a[42];
out[35] = a[43];
out[36] = a[44];
out[37] = a[45];
out[38] = a[46];
out[39] = a[45];
out[40] = a[47];
out[41] = a[48];
out[42] = a[46];
out[43] = a[48];
out[44] = a[49];
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
const real_t* u = in + 5;
const real_t* od = in + 8;
/* Vector of auxiliary variables; number of elements: 81. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (xd[0]-od[14]);
a[1] = (cos(od[16]));
a[2] = (a[0]*a[1]);
a[3] = (xd[1]-od[15]);
a[4] = (sin(od[16]));
a[5] = ((real_t)(0.0000000000000000e+00)-a[4]);
a[6] = (a[2]+(a[3]*a[5]));
a[7] = ((real_t)(1.0000000000000000e+00)/((od[17]+od[12])*(od[17]+od[12])));
a[8] = (a[6]*a[7]);
a[9] = (a[8]*a[1]);
a[10] = (sin(od[16]));
a[11] = (a[0]*a[10]);
a[12] = (cos(od[16]));
a[13] = (a[11]+(a[3]*a[12]));
a[14] = ((real_t)(1.0000000000000000e+00)/((od[18]+od[12])*(od[18]+od[12])));
a[15] = (a[13]*a[14]);
a[16] = (a[9]+(a[15]*a[10]));
a[17] = (a[16]*a[0]);
a[18] = (a[8]*a[5]);
a[19] = (a[18]+(a[15]*a[12]));
a[20] = (a[17]+(a[19]*a[3]));
a[21] = (a[1]*a[7]);
a[22] = (a[21]*a[1]);
a[23] = (a[10]*a[14]);
a[24] = (a[23]*a[10]);
a[25] = (a[22]+a[24]);
a[26] = (a[25]*a[0]);
a[27] = (a[26]+a[16]);
a[28] = (a[21]*a[5]);
a[29] = (a[23]*a[12]);
a[30] = (a[28]+a[29]);
a[31] = (a[30]*a[3]);
a[32] = (a[27]+a[31]);
a[33] = (a[5]*a[7]);
a[34] = (a[33]*a[1]);
a[35] = (a[12]*a[14]);
a[36] = (a[35]*a[10]);
a[37] = (a[34]+a[36]);
a[38] = (a[37]*a[0]);
a[39] = (a[33]*a[5]);
a[40] = (a[35]*a[12]);
a[41] = (a[39]+a[40]);
a[42] = (a[41]*a[3]);
a[43] = (a[42]+a[19]);
a[44] = (a[38]+a[43]);
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);
a[48] = (real_t)(0.0000000000000000e+00);
a[49] = (real_t)(-1.0000000000000000e+00);
a[50] = (a[25]*(real_t)(2.0000000000000000e+00));
a[51] = (a[50]*xd[4]);
a[52] = (a[37]+a[30]);
a[53] = (a[52]*xd[4]);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = (real_t)(0.0000000000000000e+00);
a[56] = (real_t)(0.0000000000000000e+00);
a[57] = (real_t)(0.0000000000000000e+00);
a[58] = (real_t)(0.0000000000000000e+00);
a[59] = (a[41]*(real_t)(2.0000000000000000e+00));
a[60] = (a[59]*xd[4]);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);
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
a[80] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (a[20]-u[2]);
out[1] = a[32];
out[2] = a[44];
out[3] = a[45];
out[4] = a[46];
out[5] = a[47];
out[6] = a[48];
out[7] = a[49];
out[8] = a[51];
out[9] = a[53];
out[10] = a[54];
out[11] = a[55];
out[12] = a[56];
out[13] = a[57];
out[14] = a[58];
out[15] = a[53];
out[16] = a[60];
out[17] = a[61];
out[18] = a[62];
out[19] = a[63];
out[20] = a[64];
out[21] = a[65];
out[22] = a[54];
out[23] = a[61];
out[24] = a[66];
out[25] = a[67];
out[26] = a[68];
out[27] = a[69];
out[28] = a[70];
out[29] = a[55];
out[30] = a[62];
out[31] = a[67];
out[32] = a[71];
out[33] = a[72];
out[34] = a[73];
out[35] = a[74];
out[36] = a[56];
out[37] = a[63];
out[38] = a[68];
out[39] = a[72];
out[40] = a[75];
out[41] = a[76];
out[42] = a[77];
out[43] = a[57];
out[44] = a[64];
out[45] = a[69];
out[46] = a[73];
out[47] = a[76];
out[48] = a[78];
out[49] = a[79];
out[50] = a[58];
out[51] = a[65];
out[52] = a[70];
out[53] = a[74];
out[54] = a[77];
out[55] = a[79];
out[56] = a[80];
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
for (runObj = 0; runObj < 50; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 19];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 19 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 19 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 19 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 19 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 19 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 19 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 19 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 19 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 19 + 9];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 19 + 10];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 19 + 11];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 19 + 12];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 19 + 13];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 19 + 14];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 19 + 15];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 19 + 16];
acadoWorkspace.objValueIn[24] = acadoVariables.od[runObj * 19 + 17];
acadoWorkspace.objValueIn[25] = acadoVariables.od[runObj * 19 + 18];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjTerm( &(acadoWorkspace.objValueOut[ 8 ]), &(acadoWorkspace.objValueOut[ 24 ]), &(acadoWorkspace.objValueOut[ 36 ]), &(acadoWorkspace.EH[ runObj * 49 ]) );
acado_addObjLinearTerm( &(acadoWorkspace.QDy[ runObj * 4 ]), &(acadoWorkspace.g[ runObj * 3 ]), &(acadoWorkspace.objValueOut[ 1 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[200];
acadoWorkspace.objValueIn[1] = acadoVariables.x[201];
acadoWorkspace.objValueIn[2] = acadoVariables.x[202];
acadoWorkspace.objValueIn[3] = acadoVariables.x[203];
acadoWorkspace.objValueIn[4] = acadoVariables.od[950];
acadoWorkspace.objValueIn[5] = acadoVariables.od[951];
acadoWorkspace.objValueIn[6] = acadoVariables.od[952];
acadoWorkspace.objValueIn[7] = acadoVariables.od[953];
acadoWorkspace.objValueIn[8] = acadoVariables.od[954];
acadoWorkspace.objValueIn[9] = acadoVariables.od[955];
acadoWorkspace.objValueIn[10] = acadoVariables.od[956];
acadoWorkspace.objValueIn[11] = acadoVariables.od[957];
acadoWorkspace.objValueIn[12] = acadoVariables.od[958];
acadoWorkspace.objValueIn[13] = acadoVariables.od[959];
acadoWorkspace.objValueIn[14] = acadoVariables.od[960];
acadoWorkspace.objValueIn[15] = acadoVariables.od[961];
acadoWorkspace.objValueIn[16] = acadoVariables.od[962];
acadoWorkspace.objValueIn[17] = acadoVariables.od[963];
acadoWorkspace.objValueIn[18] = acadoVariables.od[964];
acadoWorkspace.objValueIn[19] = acadoVariables.od[965];
acadoWorkspace.objValueIn[20] = acadoVariables.od[966];
acadoWorkspace.objValueIn[21] = acadoVariables.od[967];
acadoWorkspace.objValueIn[22] = acadoVariables.od[968];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjEndTerm( &(acadoWorkspace.objValueOut[ 5 ]), acadoWorkspace.EH_N );
acadoWorkspace.QDy[200] = acadoWorkspace.objValueOut[1];
acadoWorkspace.QDy[201] = acadoWorkspace.objValueOut[2];
acadoWorkspace.QDy[202] = acadoWorkspace.objValueOut[3];
acadoWorkspace.QDy[203] = acadoWorkspace.objValueOut[4];

for (lRun2 = 0; lRun2 < 50; ++lRun2)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun2 * 4];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun2 * 4 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun2 * 4 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun2 * 4 + 3];
acadoWorkspace.conValueIn[4] = acadoWorkspace.y[lRun2 + 150];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun2 * 3];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun2 * 3 + 1];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun2 * 3 + 2];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun2 * 19];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun2 * 19 + 1];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun2 * 19 + 2];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun2 * 19 + 3];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun2 * 19 + 4];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun2 * 19 + 5];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun2 * 19 + 6];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun2 * 19 + 7];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun2 * 19 + 8];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun2 * 19 + 9];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun2 * 19 + 10];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun2 * 19 + 11];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun2 * 19 + 12];
acadoWorkspace.conValueIn[21] = acadoVariables.od[lRun2 * 19 + 13];
acadoWorkspace.conValueIn[22] = acadoVariables.od[lRun2 * 19 + 14];
acadoWorkspace.conValueIn[23] = acadoVariables.od[lRun2 * 19 + 15];
acadoWorkspace.conValueIn[24] = acadoVariables.od[lRun2 * 19 + 16];
acadoWorkspace.conValueIn[25] = acadoVariables.od[lRun2 * 19 + 17];
acadoWorkspace.conValueIn[26] = acadoVariables.od[lRun2 * 19 + 18];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun2] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun2 * 4] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun2 * 4 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun2 * 4 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun2 * 4 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHu[lRun2 * 3] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHu[lRun2 * 3 + 1] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHu[lRun2 * 3 + 2] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evDDH[0] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evDDH[1] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evDDH[2] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evDDH[3] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evDDH[4] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evDDH[5] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evDDH[6] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evDDH[7] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evDDH[8] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evDDH[9] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evDDH[10] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evDDH[11] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evDDH[12] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evDDH[13] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evDDH[14] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evDDH[15] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evDDH[16] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evDDH[17] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evDDH[18] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evDDH[19] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evDDH[20] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evDDH[21] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evDDH[22] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evDDH[23] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evDDH[24] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evDDH[25] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evDDH[26] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evDDH[27] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evDDH[28] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evDDH[29] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evDDH[30] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evDDH[31] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evDDH[32] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evDDH[33] = acadoWorkspace.conValueOut[41];
acadoWorkspace.evDDH[34] = acadoWorkspace.conValueOut[42];
acadoWorkspace.evDDH[35] = acadoWorkspace.conValueOut[43];
acadoWorkspace.evDDH[36] = acadoWorkspace.conValueOut[44];
acadoWorkspace.evDDH[37] = acadoWorkspace.conValueOut[45];
acadoWorkspace.evDDH[38] = acadoWorkspace.conValueOut[46];
acadoWorkspace.evDDH[39] = acadoWorkspace.conValueOut[47];
acadoWorkspace.evDDH[40] = acadoWorkspace.conValueOut[48];
acadoWorkspace.evDDH[41] = acadoWorkspace.conValueOut[49];
acadoWorkspace.evDDH[42] = acadoWorkspace.conValueOut[50];
acadoWorkspace.evDDH[43] = acadoWorkspace.conValueOut[51];
acadoWorkspace.evDDH[44] = acadoWorkspace.conValueOut[52];
acadoWorkspace.evDDH[45] = acadoWorkspace.conValueOut[53];
acadoWorkspace.evDDH[46] = acadoWorkspace.conValueOut[54];
acadoWorkspace.evDDH[47] = acadoWorkspace.conValueOut[55];
acadoWorkspace.evDDH[48] = acadoWorkspace.conValueOut[56];
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
for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 453] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + R11[0];
acadoWorkspace.H[iRow * 453 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + R11[1];
acadoWorkspace.H[iRow * 453 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + R11[2];
acadoWorkspace.H[iRow * 453 + 150] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + R11[3];
acadoWorkspace.H[iRow * 453 + 151] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + R11[4];
acadoWorkspace.H[iRow * 453 + 152] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + R11[5];
acadoWorkspace.H[iRow * 453 + 300] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + R11[6];
acadoWorkspace.H[iRow * 453 + 301] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + R11[7];
acadoWorkspace.H[iRow * 453 + 302] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + R11[8];
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
acadoWorkspace.H[(iRow * 450) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 150) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3)] = acadoWorkspace.H[(iCol * 450) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 450 + 150) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 450 + 300) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 450 + 300) + (iRow * 3 + 2)];
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
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
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
acado_multGxGx( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.C[ 384 ]), &(acadoWorkspace.C[ 400 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.C[ 400 ]), &(acadoWorkspace.C[ 416 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.C[ 416 ]), &(acadoWorkspace.C[ 432 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.C[ 432 ]), &(acadoWorkspace.C[ 448 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 464 ]), &(acadoWorkspace.C[ 448 ]), &(acadoWorkspace.C[ 464 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 480 ]), &(acadoWorkspace.C[ 464 ]), &(acadoWorkspace.C[ 480 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 496 ]), &(acadoWorkspace.C[ 480 ]), &(acadoWorkspace.C[ 496 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.C[ 496 ]), &(acadoWorkspace.C[ 512 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 528 ]), &(acadoWorkspace.C[ 512 ]), &(acadoWorkspace.C[ 528 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 544 ]), &(acadoWorkspace.C[ 528 ]), &(acadoWorkspace.C[ 544 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 560 ]), &(acadoWorkspace.C[ 544 ]), &(acadoWorkspace.C[ 560 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.C[ 560 ]), &(acadoWorkspace.C[ 576 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 592 ]), &(acadoWorkspace.C[ 576 ]), &(acadoWorkspace.C[ 592 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 608 ]), &(acadoWorkspace.C[ 592 ]), &(acadoWorkspace.C[ 608 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 624 ]), &(acadoWorkspace.C[ 608 ]), &(acadoWorkspace.C[ 624 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 640 ]), &(acadoWorkspace.C[ 624 ]), &(acadoWorkspace.C[ 640 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 656 ]), &(acadoWorkspace.C[ 640 ]), &(acadoWorkspace.C[ 656 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 672 ]), &(acadoWorkspace.C[ 656 ]), &(acadoWorkspace.C[ 672 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 688 ]), &(acadoWorkspace.C[ 672 ]), &(acadoWorkspace.C[ 688 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 704 ]), &(acadoWorkspace.C[ 688 ]), &(acadoWorkspace.C[ 704 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.C[ 704 ]), &(acadoWorkspace.C[ 720 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 736 ]), &(acadoWorkspace.C[ 720 ]), &(acadoWorkspace.C[ 736 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 752 ]), &(acadoWorkspace.C[ 736 ]), &(acadoWorkspace.C[ 752 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 768 ]), &(acadoWorkspace.C[ 752 ]), &(acadoWorkspace.C[ 768 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.C[ 768 ]), &(acadoWorkspace.C[ 784 ]) );
for (lRun2 = 0; lRun2 < 50; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 101)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 12 ]), &(acadoWorkspace.E[ lRun3 * 12 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 50; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (4)) * (4)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (4)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (4)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (50)) - (1)) * (4)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 49; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 12 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 12 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (3)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 16 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 12 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.sbar[lRun1 + 4] = acadoWorkspace.d[lRun1];




for (lRun1 = 0; lRun1 < 49; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun2) * (lRun2 * -1 + 99)) / (2)) + (lRun1);
lRun4 = lRun1 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun1 * 4 + 4 ]), &(acadoWorkspace.E[ lRun3 * 12 ]), lRun4, lRun2 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[153] = acadoWorkspace.evHu[3];
acadoWorkspace.A[154] = acadoWorkspace.evHu[4];
acadoWorkspace.A[155] = acadoWorkspace.evHu[5];
acadoWorkspace.A[306] = acadoWorkspace.evHu[6];
acadoWorkspace.A[307] = acadoWorkspace.evHu[7];
acadoWorkspace.A[308] = acadoWorkspace.evHu[8];
acadoWorkspace.A[459] = acadoWorkspace.evHu[9];
acadoWorkspace.A[460] = acadoWorkspace.evHu[10];
acadoWorkspace.A[461] = acadoWorkspace.evHu[11];
acadoWorkspace.A[612] = acadoWorkspace.evHu[12];
acadoWorkspace.A[613] = acadoWorkspace.evHu[13];
acadoWorkspace.A[614] = acadoWorkspace.evHu[14];
acadoWorkspace.A[765] = acadoWorkspace.evHu[15];
acadoWorkspace.A[766] = acadoWorkspace.evHu[16];
acadoWorkspace.A[767] = acadoWorkspace.evHu[17];
acadoWorkspace.A[918] = acadoWorkspace.evHu[18];
acadoWorkspace.A[919] = acadoWorkspace.evHu[19];
acadoWorkspace.A[920] = acadoWorkspace.evHu[20];
acadoWorkspace.A[1071] = acadoWorkspace.evHu[21];
acadoWorkspace.A[1072] = acadoWorkspace.evHu[22];
acadoWorkspace.A[1073] = acadoWorkspace.evHu[23];
acadoWorkspace.A[1224] = acadoWorkspace.evHu[24];
acadoWorkspace.A[1225] = acadoWorkspace.evHu[25];
acadoWorkspace.A[1226] = acadoWorkspace.evHu[26];
acadoWorkspace.A[1377] = acadoWorkspace.evHu[27];
acadoWorkspace.A[1378] = acadoWorkspace.evHu[28];
acadoWorkspace.A[1379] = acadoWorkspace.evHu[29];
acadoWorkspace.A[1530] = acadoWorkspace.evHu[30];
acadoWorkspace.A[1531] = acadoWorkspace.evHu[31];
acadoWorkspace.A[1532] = acadoWorkspace.evHu[32];
acadoWorkspace.A[1683] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1684] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1685] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1836] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1837] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1838] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1989] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1990] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1991] = acadoWorkspace.evHu[41];
acadoWorkspace.A[2142] = acadoWorkspace.evHu[42];
acadoWorkspace.A[2143] = acadoWorkspace.evHu[43];
acadoWorkspace.A[2144] = acadoWorkspace.evHu[44];
acadoWorkspace.A[2295] = acadoWorkspace.evHu[45];
acadoWorkspace.A[2296] = acadoWorkspace.evHu[46];
acadoWorkspace.A[2297] = acadoWorkspace.evHu[47];
acadoWorkspace.A[2448] = acadoWorkspace.evHu[48];
acadoWorkspace.A[2449] = acadoWorkspace.evHu[49];
acadoWorkspace.A[2450] = acadoWorkspace.evHu[50];
acadoWorkspace.A[2601] = acadoWorkspace.evHu[51];
acadoWorkspace.A[2602] = acadoWorkspace.evHu[52];
acadoWorkspace.A[2603] = acadoWorkspace.evHu[53];
acadoWorkspace.A[2754] = acadoWorkspace.evHu[54];
acadoWorkspace.A[2755] = acadoWorkspace.evHu[55];
acadoWorkspace.A[2756] = acadoWorkspace.evHu[56];
acadoWorkspace.A[2907] = acadoWorkspace.evHu[57];
acadoWorkspace.A[2908] = acadoWorkspace.evHu[58];
acadoWorkspace.A[2909] = acadoWorkspace.evHu[59];
acadoWorkspace.A[3060] = acadoWorkspace.evHu[60];
acadoWorkspace.A[3061] = acadoWorkspace.evHu[61];
acadoWorkspace.A[3062] = acadoWorkspace.evHu[62];
acadoWorkspace.A[3213] = acadoWorkspace.evHu[63];
acadoWorkspace.A[3214] = acadoWorkspace.evHu[64];
acadoWorkspace.A[3215] = acadoWorkspace.evHu[65];
acadoWorkspace.A[3366] = acadoWorkspace.evHu[66];
acadoWorkspace.A[3367] = acadoWorkspace.evHu[67];
acadoWorkspace.A[3368] = acadoWorkspace.evHu[68];
acadoWorkspace.A[3519] = acadoWorkspace.evHu[69];
acadoWorkspace.A[3520] = acadoWorkspace.evHu[70];
acadoWorkspace.A[3521] = acadoWorkspace.evHu[71];
acadoWorkspace.A[3672] = acadoWorkspace.evHu[72];
acadoWorkspace.A[3673] = acadoWorkspace.evHu[73];
acadoWorkspace.A[3674] = acadoWorkspace.evHu[74];
acadoWorkspace.A[3825] = acadoWorkspace.evHu[75];
acadoWorkspace.A[3826] = acadoWorkspace.evHu[76];
acadoWorkspace.A[3827] = acadoWorkspace.evHu[77];
acadoWorkspace.A[3978] = acadoWorkspace.evHu[78];
acadoWorkspace.A[3979] = acadoWorkspace.evHu[79];
acadoWorkspace.A[3980] = acadoWorkspace.evHu[80];
acadoWorkspace.A[4131] = acadoWorkspace.evHu[81];
acadoWorkspace.A[4132] = acadoWorkspace.evHu[82];
acadoWorkspace.A[4133] = acadoWorkspace.evHu[83];
acadoWorkspace.A[4284] = acadoWorkspace.evHu[84];
acadoWorkspace.A[4285] = acadoWorkspace.evHu[85];
acadoWorkspace.A[4286] = acadoWorkspace.evHu[86];
acadoWorkspace.A[4437] = acadoWorkspace.evHu[87];
acadoWorkspace.A[4438] = acadoWorkspace.evHu[88];
acadoWorkspace.A[4439] = acadoWorkspace.evHu[89];
acadoWorkspace.A[4590] = acadoWorkspace.evHu[90];
acadoWorkspace.A[4591] = acadoWorkspace.evHu[91];
acadoWorkspace.A[4592] = acadoWorkspace.evHu[92];
acadoWorkspace.A[4743] = acadoWorkspace.evHu[93];
acadoWorkspace.A[4744] = acadoWorkspace.evHu[94];
acadoWorkspace.A[4745] = acadoWorkspace.evHu[95];
acadoWorkspace.A[4896] = acadoWorkspace.evHu[96];
acadoWorkspace.A[4897] = acadoWorkspace.evHu[97];
acadoWorkspace.A[4898] = acadoWorkspace.evHu[98];
acadoWorkspace.A[5049] = acadoWorkspace.evHu[99];
acadoWorkspace.A[5050] = acadoWorkspace.evHu[100];
acadoWorkspace.A[5051] = acadoWorkspace.evHu[101];
acadoWorkspace.A[5202] = acadoWorkspace.evHu[102];
acadoWorkspace.A[5203] = acadoWorkspace.evHu[103];
acadoWorkspace.A[5204] = acadoWorkspace.evHu[104];
acadoWorkspace.A[5355] = acadoWorkspace.evHu[105];
acadoWorkspace.A[5356] = acadoWorkspace.evHu[106];
acadoWorkspace.A[5357] = acadoWorkspace.evHu[107];
acadoWorkspace.A[5508] = acadoWorkspace.evHu[108];
acadoWorkspace.A[5509] = acadoWorkspace.evHu[109];
acadoWorkspace.A[5510] = acadoWorkspace.evHu[110];
acadoWorkspace.A[5661] = acadoWorkspace.evHu[111];
acadoWorkspace.A[5662] = acadoWorkspace.evHu[112];
acadoWorkspace.A[5663] = acadoWorkspace.evHu[113];
acadoWorkspace.A[5814] = acadoWorkspace.evHu[114];
acadoWorkspace.A[5815] = acadoWorkspace.evHu[115];
acadoWorkspace.A[5816] = acadoWorkspace.evHu[116];
acadoWorkspace.A[5967] = acadoWorkspace.evHu[117];
acadoWorkspace.A[5968] = acadoWorkspace.evHu[118];
acadoWorkspace.A[5969] = acadoWorkspace.evHu[119];
acadoWorkspace.A[6120] = acadoWorkspace.evHu[120];
acadoWorkspace.A[6121] = acadoWorkspace.evHu[121];
acadoWorkspace.A[6122] = acadoWorkspace.evHu[122];
acadoWorkspace.A[6273] = acadoWorkspace.evHu[123];
acadoWorkspace.A[6274] = acadoWorkspace.evHu[124];
acadoWorkspace.A[6275] = acadoWorkspace.evHu[125];
acadoWorkspace.A[6426] = acadoWorkspace.evHu[126];
acadoWorkspace.A[6427] = acadoWorkspace.evHu[127];
acadoWorkspace.A[6428] = acadoWorkspace.evHu[128];
acadoWorkspace.A[6579] = acadoWorkspace.evHu[129];
acadoWorkspace.A[6580] = acadoWorkspace.evHu[130];
acadoWorkspace.A[6581] = acadoWorkspace.evHu[131];
acadoWorkspace.A[6732] = acadoWorkspace.evHu[132];
acadoWorkspace.A[6733] = acadoWorkspace.evHu[133];
acadoWorkspace.A[6734] = acadoWorkspace.evHu[134];
acadoWorkspace.A[6885] = acadoWorkspace.evHu[135];
acadoWorkspace.A[6886] = acadoWorkspace.evHu[136];
acadoWorkspace.A[6887] = acadoWorkspace.evHu[137];
acadoWorkspace.A[7038] = acadoWorkspace.evHu[138];
acadoWorkspace.A[7039] = acadoWorkspace.evHu[139];
acadoWorkspace.A[7040] = acadoWorkspace.evHu[140];
acadoWorkspace.A[7191] = acadoWorkspace.evHu[141];
acadoWorkspace.A[7192] = acadoWorkspace.evHu[142];
acadoWorkspace.A[7193] = acadoWorkspace.evHu[143];
acadoWorkspace.A[7344] = acadoWorkspace.evHu[144];
acadoWorkspace.A[7345] = acadoWorkspace.evHu[145];
acadoWorkspace.A[7346] = acadoWorkspace.evHu[146];
acadoWorkspace.A[7497] = acadoWorkspace.evHu[147];
acadoWorkspace.A[7498] = acadoWorkspace.evHu[148];
acadoWorkspace.A[7499] = acadoWorkspace.evHu[149];
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
acado_macASbar( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 116 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 124 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 148 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 640 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 164 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 656 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 672 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 172 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 688 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.sbar[ 176 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 704 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 184 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 736 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.sbar[ 188 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 752 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 768 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 196 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.sbar[ 200 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[200];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[201];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[202];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[200] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[201] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[202] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[203] + acadoWorkspace.QDy[203];
acado_macBTw1( &(acadoWorkspace.evGu[ 588 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 147 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 588 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.g[ 147 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 784 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 196 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 784 ]), &(acadoWorkspace.sbar[ 196 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 144 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 576 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.g[ 144 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 768 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 192 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 768 ]), &(acadoWorkspace.sbar[ 192 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 564 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 141 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 564 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.g[ 141 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 752 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 188 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 752 ]), &(acadoWorkspace.sbar[ 188 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 552 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 138 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 552 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.g[ 138 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 736 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 184 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 736 ]), &(acadoWorkspace.sbar[ 184 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 135 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 540 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.g[ 135 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 528 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 132 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 528 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.g[ 132 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 704 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 176 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 704 ]), &(acadoWorkspace.sbar[ 176 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 516 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 129 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 516 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.g[ 129 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 688 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 172 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 688 ]), &(acadoWorkspace.sbar[ 172 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 504 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 126 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 504 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.g[ 126 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 672 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 168 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 672 ]), &(acadoWorkspace.sbar[ 168 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 492 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 123 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 492 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.g[ 123 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 656 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 164 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 656 ]), &(acadoWorkspace.sbar[ 164 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 120 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 480 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.g[ 120 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 640 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 160 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 640 ]), &(acadoWorkspace.sbar[ 160 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 117 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 468 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.g[ 117 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 624 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 456 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 114 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 456 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.g[ 114 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 608 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 152 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 444 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 111 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 444 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.g[ 111 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 592 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 148 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 108 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.g[ 108 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 420 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 105 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 420 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.g[ 105 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 560 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 408 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 102 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 408 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.g[ 102 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 544 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 136 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 99 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.g[ 99 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 528 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 384 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 128 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 372 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 93 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 372 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.g[ 93 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 496 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 124 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 348 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 87 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 348 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.g[ 87 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 464 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 116 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 336 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 112 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 81 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.g[ 81 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 312 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 312 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 416 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 104 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 300 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 75 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 300 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.g[ 75 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
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
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.lb[80] = acadoVariables.lbValues[80] - acadoVariables.u[80];
acadoWorkspace.lb[81] = acadoVariables.lbValues[81] - acadoVariables.u[81];
acadoWorkspace.lb[82] = acadoVariables.lbValues[82] - acadoVariables.u[82];
acadoWorkspace.lb[83] = acadoVariables.lbValues[83] - acadoVariables.u[83];
acadoWorkspace.lb[84] = acadoVariables.lbValues[84] - acadoVariables.u[84];
acadoWorkspace.lb[85] = acadoVariables.lbValues[85] - acadoVariables.u[85];
acadoWorkspace.lb[86] = acadoVariables.lbValues[86] - acadoVariables.u[86];
acadoWorkspace.lb[87] = acadoVariables.lbValues[87] - acadoVariables.u[87];
acadoWorkspace.lb[88] = acadoVariables.lbValues[88] - acadoVariables.u[88];
acadoWorkspace.lb[89] = acadoVariables.lbValues[89] - acadoVariables.u[89];
acadoWorkspace.lb[90] = acadoVariables.lbValues[90] - acadoVariables.u[90];
acadoWorkspace.lb[91] = acadoVariables.lbValues[91] - acadoVariables.u[91];
acadoWorkspace.lb[92] = acadoVariables.lbValues[92] - acadoVariables.u[92];
acadoWorkspace.lb[93] = acadoVariables.lbValues[93] - acadoVariables.u[93];
acadoWorkspace.lb[94] = acadoVariables.lbValues[94] - acadoVariables.u[94];
acadoWorkspace.lb[95] = acadoVariables.lbValues[95] - acadoVariables.u[95];
acadoWorkspace.lb[96] = acadoVariables.lbValues[96] - acadoVariables.u[96];
acadoWorkspace.lb[97] = acadoVariables.lbValues[97] - acadoVariables.u[97];
acadoWorkspace.lb[98] = acadoVariables.lbValues[98] - acadoVariables.u[98];
acadoWorkspace.lb[99] = acadoVariables.lbValues[99] - acadoVariables.u[99];
acadoWorkspace.lb[100] = acadoVariables.lbValues[100] - acadoVariables.u[100];
acadoWorkspace.lb[101] = acadoVariables.lbValues[101] - acadoVariables.u[101];
acadoWorkspace.lb[102] = acadoVariables.lbValues[102] - acadoVariables.u[102];
acadoWorkspace.lb[103] = acadoVariables.lbValues[103] - acadoVariables.u[103];
acadoWorkspace.lb[104] = acadoVariables.lbValues[104] - acadoVariables.u[104];
acadoWorkspace.lb[105] = acadoVariables.lbValues[105] - acadoVariables.u[105];
acadoWorkspace.lb[106] = acadoVariables.lbValues[106] - acadoVariables.u[106];
acadoWorkspace.lb[107] = acadoVariables.lbValues[107] - acadoVariables.u[107];
acadoWorkspace.lb[108] = acadoVariables.lbValues[108] - acadoVariables.u[108];
acadoWorkspace.lb[109] = acadoVariables.lbValues[109] - acadoVariables.u[109];
acadoWorkspace.lb[110] = acadoVariables.lbValues[110] - acadoVariables.u[110];
acadoWorkspace.lb[111] = acadoVariables.lbValues[111] - acadoVariables.u[111];
acadoWorkspace.lb[112] = acadoVariables.lbValues[112] - acadoVariables.u[112];
acadoWorkspace.lb[113] = acadoVariables.lbValues[113] - acadoVariables.u[113];
acadoWorkspace.lb[114] = acadoVariables.lbValues[114] - acadoVariables.u[114];
acadoWorkspace.lb[115] = acadoVariables.lbValues[115] - acadoVariables.u[115];
acadoWorkspace.lb[116] = acadoVariables.lbValues[116] - acadoVariables.u[116];
acadoWorkspace.lb[117] = acadoVariables.lbValues[117] - acadoVariables.u[117];
acadoWorkspace.lb[118] = acadoVariables.lbValues[118] - acadoVariables.u[118];
acadoWorkspace.lb[119] = acadoVariables.lbValues[119] - acadoVariables.u[119];
acadoWorkspace.lb[120] = acadoVariables.lbValues[120] - acadoVariables.u[120];
acadoWorkspace.lb[121] = acadoVariables.lbValues[121] - acadoVariables.u[121];
acadoWorkspace.lb[122] = acadoVariables.lbValues[122] - acadoVariables.u[122];
acadoWorkspace.lb[123] = acadoVariables.lbValues[123] - acadoVariables.u[123];
acadoWorkspace.lb[124] = acadoVariables.lbValues[124] - acadoVariables.u[124];
acadoWorkspace.lb[125] = acadoVariables.lbValues[125] - acadoVariables.u[125];
acadoWorkspace.lb[126] = acadoVariables.lbValues[126] - acadoVariables.u[126];
acadoWorkspace.lb[127] = acadoVariables.lbValues[127] - acadoVariables.u[127];
acadoWorkspace.lb[128] = acadoVariables.lbValues[128] - acadoVariables.u[128];
acadoWorkspace.lb[129] = acadoVariables.lbValues[129] - acadoVariables.u[129];
acadoWorkspace.lb[130] = acadoVariables.lbValues[130] - acadoVariables.u[130];
acadoWorkspace.lb[131] = acadoVariables.lbValues[131] - acadoVariables.u[131];
acadoWorkspace.lb[132] = acadoVariables.lbValues[132] - acadoVariables.u[132];
acadoWorkspace.lb[133] = acadoVariables.lbValues[133] - acadoVariables.u[133];
acadoWorkspace.lb[134] = acadoVariables.lbValues[134] - acadoVariables.u[134];
acadoWorkspace.lb[135] = acadoVariables.lbValues[135] - acadoVariables.u[135];
acadoWorkspace.lb[136] = acadoVariables.lbValues[136] - acadoVariables.u[136];
acadoWorkspace.lb[137] = acadoVariables.lbValues[137] - acadoVariables.u[137];
acadoWorkspace.lb[138] = acadoVariables.lbValues[138] - acadoVariables.u[138];
acadoWorkspace.lb[139] = acadoVariables.lbValues[139] - acadoVariables.u[139];
acadoWorkspace.lb[140] = acadoVariables.lbValues[140] - acadoVariables.u[140];
acadoWorkspace.lb[141] = acadoVariables.lbValues[141] - acadoVariables.u[141];
acadoWorkspace.lb[142] = acadoVariables.lbValues[142] - acadoVariables.u[142];
acadoWorkspace.lb[143] = acadoVariables.lbValues[143] - acadoVariables.u[143];
acadoWorkspace.lb[144] = acadoVariables.lbValues[144] - acadoVariables.u[144];
acadoWorkspace.lb[145] = acadoVariables.lbValues[145] - acadoVariables.u[145];
acadoWorkspace.lb[146] = acadoVariables.lbValues[146] - acadoVariables.u[146];
acadoWorkspace.lb[147] = acadoVariables.lbValues[147] - acadoVariables.u[147];
acadoWorkspace.lb[148] = acadoVariables.lbValues[148] - acadoVariables.u[148];
acadoWorkspace.lb[149] = acadoVariables.lbValues[149] - acadoVariables.u[149];
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
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[80] = acadoVariables.ubValues[80] - acadoVariables.u[80];
acadoWorkspace.ub[81] = acadoVariables.ubValues[81] - acadoVariables.u[81];
acadoWorkspace.ub[82] = acadoVariables.ubValues[82] - acadoVariables.u[82];
acadoWorkspace.ub[83] = acadoVariables.ubValues[83] - acadoVariables.u[83];
acadoWorkspace.ub[84] = acadoVariables.ubValues[84] - acadoVariables.u[84];
acadoWorkspace.ub[85] = acadoVariables.ubValues[85] - acadoVariables.u[85];
acadoWorkspace.ub[86] = acadoVariables.ubValues[86] - acadoVariables.u[86];
acadoWorkspace.ub[87] = acadoVariables.ubValues[87] - acadoVariables.u[87];
acadoWorkspace.ub[88] = acadoVariables.ubValues[88] - acadoVariables.u[88];
acadoWorkspace.ub[89] = acadoVariables.ubValues[89] - acadoVariables.u[89];
acadoWorkspace.ub[90] = acadoVariables.ubValues[90] - acadoVariables.u[90];
acadoWorkspace.ub[91] = acadoVariables.ubValues[91] - acadoVariables.u[91];
acadoWorkspace.ub[92] = acadoVariables.ubValues[92] - acadoVariables.u[92];
acadoWorkspace.ub[93] = acadoVariables.ubValues[93] - acadoVariables.u[93];
acadoWorkspace.ub[94] = acadoVariables.ubValues[94] - acadoVariables.u[94];
acadoWorkspace.ub[95] = acadoVariables.ubValues[95] - acadoVariables.u[95];
acadoWorkspace.ub[96] = acadoVariables.ubValues[96] - acadoVariables.u[96];
acadoWorkspace.ub[97] = acadoVariables.ubValues[97] - acadoVariables.u[97];
acadoWorkspace.ub[98] = acadoVariables.ubValues[98] - acadoVariables.u[98];
acadoWorkspace.ub[99] = acadoVariables.ubValues[99] - acadoVariables.u[99];
acadoWorkspace.ub[100] = acadoVariables.ubValues[100] - acadoVariables.u[100];
acadoWorkspace.ub[101] = acadoVariables.ubValues[101] - acadoVariables.u[101];
acadoWorkspace.ub[102] = acadoVariables.ubValues[102] - acadoVariables.u[102];
acadoWorkspace.ub[103] = acadoVariables.ubValues[103] - acadoVariables.u[103];
acadoWorkspace.ub[104] = acadoVariables.ubValues[104] - acadoVariables.u[104];
acadoWorkspace.ub[105] = acadoVariables.ubValues[105] - acadoVariables.u[105];
acadoWorkspace.ub[106] = acadoVariables.ubValues[106] - acadoVariables.u[106];
acadoWorkspace.ub[107] = acadoVariables.ubValues[107] - acadoVariables.u[107];
acadoWorkspace.ub[108] = acadoVariables.ubValues[108] - acadoVariables.u[108];
acadoWorkspace.ub[109] = acadoVariables.ubValues[109] - acadoVariables.u[109];
acadoWorkspace.ub[110] = acadoVariables.ubValues[110] - acadoVariables.u[110];
acadoWorkspace.ub[111] = acadoVariables.ubValues[111] - acadoVariables.u[111];
acadoWorkspace.ub[112] = acadoVariables.ubValues[112] - acadoVariables.u[112];
acadoWorkspace.ub[113] = acadoVariables.ubValues[113] - acadoVariables.u[113];
acadoWorkspace.ub[114] = acadoVariables.ubValues[114] - acadoVariables.u[114];
acadoWorkspace.ub[115] = acadoVariables.ubValues[115] - acadoVariables.u[115];
acadoWorkspace.ub[116] = acadoVariables.ubValues[116] - acadoVariables.u[116];
acadoWorkspace.ub[117] = acadoVariables.ubValues[117] - acadoVariables.u[117];
acadoWorkspace.ub[118] = acadoVariables.ubValues[118] - acadoVariables.u[118];
acadoWorkspace.ub[119] = acadoVariables.ubValues[119] - acadoVariables.u[119];
acadoWorkspace.ub[120] = acadoVariables.ubValues[120] - acadoVariables.u[120];
acadoWorkspace.ub[121] = acadoVariables.ubValues[121] - acadoVariables.u[121];
acadoWorkspace.ub[122] = acadoVariables.ubValues[122] - acadoVariables.u[122];
acadoWorkspace.ub[123] = acadoVariables.ubValues[123] - acadoVariables.u[123];
acadoWorkspace.ub[124] = acadoVariables.ubValues[124] - acadoVariables.u[124];
acadoWorkspace.ub[125] = acadoVariables.ubValues[125] - acadoVariables.u[125];
acadoWorkspace.ub[126] = acadoVariables.ubValues[126] - acadoVariables.u[126];
acadoWorkspace.ub[127] = acadoVariables.ubValues[127] - acadoVariables.u[127];
acadoWorkspace.ub[128] = acadoVariables.ubValues[128] - acadoVariables.u[128];
acadoWorkspace.ub[129] = acadoVariables.ubValues[129] - acadoVariables.u[129];
acadoWorkspace.ub[130] = acadoVariables.ubValues[130] - acadoVariables.u[130];
acadoWorkspace.ub[131] = acadoVariables.ubValues[131] - acadoVariables.u[131];
acadoWorkspace.ub[132] = acadoVariables.ubValues[132] - acadoVariables.u[132];
acadoWorkspace.ub[133] = acadoVariables.ubValues[133] - acadoVariables.u[133];
acadoWorkspace.ub[134] = acadoVariables.ubValues[134] - acadoVariables.u[134];
acadoWorkspace.ub[135] = acadoVariables.ubValues[135] - acadoVariables.u[135];
acadoWorkspace.ub[136] = acadoVariables.ubValues[136] - acadoVariables.u[136];
acadoWorkspace.ub[137] = acadoVariables.ubValues[137] - acadoVariables.u[137];
acadoWorkspace.ub[138] = acadoVariables.ubValues[138] - acadoVariables.u[138];
acadoWorkspace.ub[139] = acadoVariables.ubValues[139] - acadoVariables.u[139];
acadoWorkspace.ub[140] = acadoVariables.ubValues[140] - acadoVariables.u[140];
acadoWorkspace.ub[141] = acadoVariables.ubValues[141] - acadoVariables.u[141];
acadoWorkspace.ub[142] = acadoVariables.ubValues[142] - acadoVariables.u[142];
acadoWorkspace.ub[143] = acadoVariables.ubValues[143] - acadoVariables.u[143];
acadoWorkspace.ub[144] = acadoVariables.ubValues[144] - acadoVariables.u[144];
acadoWorkspace.ub[145] = acadoVariables.ubValues[145] - acadoVariables.u[145];
acadoWorkspace.ub[146] = acadoVariables.ubValues[146] - acadoVariables.u[146];
acadoWorkspace.ub[147] = acadoVariables.ubValues[147] - acadoVariables.u[147];
acadoWorkspace.ub[148] = acadoVariables.ubValues[148] - acadoVariables.u[148];
acadoWorkspace.ub[149] = acadoVariables.ubValues[149] - acadoVariables.u[149];


acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 4 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 16 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 28 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 44 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 52 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 56 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 64 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 68 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 76 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 88 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 92 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.lbA[ 23 ]), &(acadoWorkspace.ubA[ 23 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.lbA[ 25 ]), &(acadoWorkspace.ubA[ 25 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 104 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 112 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 116 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.lbA[ 29 ]), &(acadoWorkspace.ubA[ 29 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 124 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.lbA[ 31 ]), &(acadoWorkspace.ubA[ 31 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 128 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.lbA[ 33 ]), &(acadoWorkspace.ubA[ 33 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 136 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.lbA[ 35 ]), &(acadoWorkspace.ubA[ 35 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 148 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.lbA[ 37 ]), &(acadoWorkspace.ubA[ 37 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 152 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.lbA[ 38 ]), &(acadoWorkspace.ubA[ 38 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.lbA[ 39 ]), &(acadoWorkspace.ubA[ 39 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 164 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.lbA[ 41 ]), &(acadoWorkspace.ubA[ 41 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 172 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.lbA[ 43 ]), &(acadoWorkspace.ubA[ 43 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 176 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.lbA[ 45 ]), &(acadoWorkspace.ubA[ 45 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 184 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 188 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.lbA[ 47 ]), &(acadoWorkspace.ubA[ 47 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 196 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.lbA[ 49 ]), &(acadoWorkspace.ubA[ 49 ]) );

}

void acado_expand(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoVariables.u[lRun1] += acadoWorkspace.x[lRun1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.sbar[lRun1 + 4] = acadoWorkspace.d[lRun1];

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
acado_expansionStep( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGu[ 300 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.evGu[ 312 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 81 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.evGu[ 336 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 116 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 464 ]), &(acadoWorkspace.evGu[ 348 ]), &(acadoWorkspace.x[ 87 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 480 ]), &(acadoWorkspace.evGu[ 360 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 124 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 496 ]), &(acadoWorkspace.evGu[ 372 ]), &(acadoWorkspace.x[ 93 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.evGu[ 384 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 528 ]), &(acadoWorkspace.evGu[ 396 ]), &(acadoWorkspace.x[ 99 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 544 ]), &(acadoWorkspace.evGu[ 408 ]), &(acadoWorkspace.x[ 102 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 560 ]), &(acadoWorkspace.evGu[ 420 ]), &(acadoWorkspace.x[ 105 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 148 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 592 ]), &(acadoWorkspace.evGu[ 444 ]), &(acadoWorkspace.x[ 111 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 608 ]), &(acadoWorkspace.evGu[ 456 ]), &(acadoWorkspace.x[ 114 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 624 ]), &(acadoWorkspace.evGu[ 468 ]), &(acadoWorkspace.x[ 117 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 160 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 640 ]), &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.x[ 120 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.sbar[ 164 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 656 ]), &(acadoWorkspace.evGu[ 492 ]), &(acadoWorkspace.x[ 123 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 672 ]), &(acadoWorkspace.evGu[ 504 ]), &(acadoWorkspace.x[ 126 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 172 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 688 ]), &(acadoWorkspace.evGu[ 516 ]), &(acadoWorkspace.x[ 129 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.sbar[ 176 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 704 ]), &(acadoWorkspace.evGu[ 528 ]), &(acadoWorkspace.x[ 132 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.evGu[ 540 ]), &(acadoWorkspace.x[ 135 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 184 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 736 ]), &(acadoWorkspace.evGu[ 552 ]), &(acadoWorkspace.x[ 138 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.sbar[ 188 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 752 ]), &(acadoWorkspace.evGu[ 564 ]), &(acadoWorkspace.x[ 141 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 768 ]), &(acadoWorkspace.evGu[ 576 ]), &(acadoWorkspace.x[ 144 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 196 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 784 ]), &(acadoWorkspace.evGu[ 588 ]), &(acadoWorkspace.x[ 147 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.sbar[ 200 ]) );
for (lRun1 = 0; lRun1 < 204; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

acadoVariables.mu[196] = 0.0000000000000000e+00;
acadoVariables.mu[197] = 0.0000000000000000e+00;
acadoVariables.mu[198] = 0.0000000000000000e+00;
acadoVariables.mu[199] = 0.0000000000000000e+00;
acadoVariables.mu[196] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[8] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[12];
acadoVariables.mu[197] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[9] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[13];
acadoVariables.mu[198] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[6] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[10] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[14];
acadoVariables.mu[199] += + acadoWorkspace.sbar[200]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[201]*acadoWorkspace.QN1[7] + acadoWorkspace.sbar[202]*acadoWorkspace.QN1[11] + acadoWorkspace.sbar[203]*acadoWorkspace.QN1[15];
acadoVariables.mu[196] += acadoWorkspace.QDy[200];
acadoVariables.mu[197] += acadoWorkspace.QDy[201];
acadoVariables.mu[198] += acadoWorkspace.QDy[202];
acadoVariables.mu[199] += acadoWorkspace.QDy[203];
acadoVariables.mu[192] = 0.0000000000000000e+00;
acadoVariables.mu[192] -= + acadoWorkspace.y[199]*acadoWorkspace.evHx[196];
acadoVariables.mu[193] = 0.0000000000000000e+00;
acadoVariables.mu[193] -= + acadoWorkspace.y[199]*acadoWorkspace.evHx[197];
acadoVariables.mu[194] = 0.0000000000000000e+00;
acadoVariables.mu[194] -= + acadoWorkspace.y[199]*acadoWorkspace.evHx[198];
acadoVariables.mu[195] = 0.0000000000000000e+00;
acadoVariables.mu[195] -= + acadoWorkspace.y[199]*acadoWorkspace.evHx[199];
acado_expansionStep2( &(acadoWorkspace.QDy[ 196 ]), &(acadoWorkspace.Q1[ 784 ]), &(acadoWorkspace.sbar[ 196 ]), &(acadoWorkspace.S1[ 588 ]), &(acadoWorkspace.x[ 147 ]), &(acadoWorkspace.evGx[ 784 ]), &(acadoVariables.mu[ 192 ]), &(acadoVariables.mu[ 196 ]) );
acadoVariables.mu[188] = 0.0000000000000000e+00;
acadoVariables.mu[188] -= + acadoWorkspace.y[198]*acadoWorkspace.evHx[192];
acadoVariables.mu[189] = 0.0000000000000000e+00;
acadoVariables.mu[189] -= + acadoWorkspace.y[198]*acadoWorkspace.evHx[193];
acadoVariables.mu[190] = 0.0000000000000000e+00;
acadoVariables.mu[190] -= + acadoWorkspace.y[198]*acadoWorkspace.evHx[194];
acadoVariables.mu[191] = 0.0000000000000000e+00;
acadoVariables.mu[191] -= + acadoWorkspace.y[198]*acadoWorkspace.evHx[195];
acado_expansionStep2( &(acadoWorkspace.QDy[ 192 ]), &(acadoWorkspace.Q1[ 768 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.S1[ 576 ]), &(acadoWorkspace.x[ 144 ]), &(acadoWorkspace.evGx[ 768 ]), &(acadoVariables.mu[ 188 ]), &(acadoVariables.mu[ 192 ]) );
acadoVariables.mu[184] = 0.0000000000000000e+00;
acadoVariables.mu[184] -= + acadoWorkspace.y[197]*acadoWorkspace.evHx[188];
acadoVariables.mu[185] = 0.0000000000000000e+00;
acadoVariables.mu[185] -= + acadoWorkspace.y[197]*acadoWorkspace.evHx[189];
acadoVariables.mu[186] = 0.0000000000000000e+00;
acadoVariables.mu[186] -= + acadoWorkspace.y[197]*acadoWorkspace.evHx[190];
acadoVariables.mu[187] = 0.0000000000000000e+00;
acadoVariables.mu[187] -= + acadoWorkspace.y[197]*acadoWorkspace.evHx[191];
acado_expansionStep2( &(acadoWorkspace.QDy[ 188 ]), &(acadoWorkspace.Q1[ 752 ]), &(acadoWorkspace.sbar[ 188 ]), &(acadoWorkspace.S1[ 564 ]), &(acadoWorkspace.x[ 141 ]), &(acadoWorkspace.evGx[ 752 ]), &(acadoVariables.mu[ 184 ]), &(acadoVariables.mu[ 188 ]) );
acadoVariables.mu[180] = 0.0000000000000000e+00;
acadoVariables.mu[180] -= + acadoWorkspace.y[196]*acadoWorkspace.evHx[184];
acadoVariables.mu[181] = 0.0000000000000000e+00;
acadoVariables.mu[181] -= + acadoWorkspace.y[196]*acadoWorkspace.evHx[185];
acadoVariables.mu[182] = 0.0000000000000000e+00;
acadoVariables.mu[182] -= + acadoWorkspace.y[196]*acadoWorkspace.evHx[186];
acadoVariables.mu[183] = 0.0000000000000000e+00;
acadoVariables.mu[183] -= + acadoWorkspace.y[196]*acadoWorkspace.evHx[187];
acado_expansionStep2( &(acadoWorkspace.QDy[ 184 ]), &(acadoWorkspace.Q1[ 736 ]), &(acadoWorkspace.sbar[ 184 ]), &(acadoWorkspace.S1[ 552 ]), &(acadoWorkspace.x[ 138 ]), &(acadoWorkspace.evGx[ 736 ]), &(acadoVariables.mu[ 180 ]), &(acadoVariables.mu[ 184 ]) );
acadoVariables.mu[176] = 0.0000000000000000e+00;
acadoVariables.mu[176] -= + acadoWorkspace.y[195]*acadoWorkspace.evHx[180];
acadoVariables.mu[177] = 0.0000000000000000e+00;
acadoVariables.mu[177] -= + acadoWorkspace.y[195]*acadoWorkspace.evHx[181];
acadoVariables.mu[178] = 0.0000000000000000e+00;
acadoVariables.mu[178] -= + acadoWorkspace.y[195]*acadoWorkspace.evHx[182];
acadoVariables.mu[179] = 0.0000000000000000e+00;
acadoVariables.mu[179] -= + acadoWorkspace.y[195]*acadoWorkspace.evHx[183];
acado_expansionStep2( &(acadoWorkspace.QDy[ 180 ]), &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.S1[ 540 ]), &(acadoWorkspace.x[ 135 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoVariables.mu[ 176 ]), &(acadoVariables.mu[ 180 ]) );
acadoVariables.mu[172] = 0.0000000000000000e+00;
acadoVariables.mu[172] -= + acadoWorkspace.y[194]*acadoWorkspace.evHx[176];
acadoVariables.mu[173] = 0.0000000000000000e+00;
acadoVariables.mu[173] -= + acadoWorkspace.y[194]*acadoWorkspace.evHx[177];
acadoVariables.mu[174] = 0.0000000000000000e+00;
acadoVariables.mu[174] -= + acadoWorkspace.y[194]*acadoWorkspace.evHx[178];
acadoVariables.mu[175] = 0.0000000000000000e+00;
acadoVariables.mu[175] -= + acadoWorkspace.y[194]*acadoWorkspace.evHx[179];
acado_expansionStep2( &(acadoWorkspace.QDy[ 176 ]), &(acadoWorkspace.Q1[ 704 ]), &(acadoWorkspace.sbar[ 176 ]), &(acadoWorkspace.S1[ 528 ]), &(acadoWorkspace.x[ 132 ]), &(acadoWorkspace.evGx[ 704 ]), &(acadoVariables.mu[ 172 ]), &(acadoVariables.mu[ 176 ]) );
acadoVariables.mu[168] = 0.0000000000000000e+00;
acadoVariables.mu[168] -= + acadoWorkspace.y[193]*acadoWorkspace.evHx[172];
acadoVariables.mu[169] = 0.0000000000000000e+00;
acadoVariables.mu[169] -= + acadoWorkspace.y[193]*acadoWorkspace.evHx[173];
acadoVariables.mu[170] = 0.0000000000000000e+00;
acadoVariables.mu[170] -= + acadoWorkspace.y[193]*acadoWorkspace.evHx[174];
acadoVariables.mu[171] = 0.0000000000000000e+00;
acadoVariables.mu[171] -= + acadoWorkspace.y[193]*acadoWorkspace.evHx[175];
acado_expansionStep2( &(acadoWorkspace.QDy[ 172 ]), &(acadoWorkspace.Q1[ 688 ]), &(acadoWorkspace.sbar[ 172 ]), &(acadoWorkspace.S1[ 516 ]), &(acadoWorkspace.x[ 129 ]), &(acadoWorkspace.evGx[ 688 ]), &(acadoVariables.mu[ 168 ]), &(acadoVariables.mu[ 172 ]) );
acadoVariables.mu[164] = 0.0000000000000000e+00;
acadoVariables.mu[164] -= + acadoWorkspace.y[192]*acadoWorkspace.evHx[168];
acadoVariables.mu[165] = 0.0000000000000000e+00;
acadoVariables.mu[165] -= + acadoWorkspace.y[192]*acadoWorkspace.evHx[169];
acadoVariables.mu[166] = 0.0000000000000000e+00;
acadoVariables.mu[166] -= + acadoWorkspace.y[192]*acadoWorkspace.evHx[170];
acadoVariables.mu[167] = 0.0000000000000000e+00;
acadoVariables.mu[167] -= + acadoWorkspace.y[192]*acadoWorkspace.evHx[171];
acado_expansionStep2( &(acadoWorkspace.QDy[ 168 ]), &(acadoWorkspace.Q1[ 672 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.S1[ 504 ]), &(acadoWorkspace.x[ 126 ]), &(acadoWorkspace.evGx[ 672 ]), &(acadoVariables.mu[ 164 ]), &(acadoVariables.mu[ 168 ]) );
acadoVariables.mu[160] = 0.0000000000000000e+00;
acadoVariables.mu[160] -= + acadoWorkspace.y[191]*acadoWorkspace.evHx[164];
acadoVariables.mu[161] = 0.0000000000000000e+00;
acadoVariables.mu[161] -= + acadoWorkspace.y[191]*acadoWorkspace.evHx[165];
acadoVariables.mu[162] = 0.0000000000000000e+00;
acadoVariables.mu[162] -= + acadoWorkspace.y[191]*acadoWorkspace.evHx[166];
acadoVariables.mu[163] = 0.0000000000000000e+00;
acadoVariables.mu[163] -= + acadoWorkspace.y[191]*acadoWorkspace.evHx[167];
acado_expansionStep2( &(acadoWorkspace.QDy[ 164 ]), &(acadoWorkspace.Q1[ 656 ]), &(acadoWorkspace.sbar[ 164 ]), &(acadoWorkspace.S1[ 492 ]), &(acadoWorkspace.x[ 123 ]), &(acadoWorkspace.evGx[ 656 ]), &(acadoVariables.mu[ 160 ]), &(acadoVariables.mu[ 164 ]) );
acadoVariables.mu[156] = 0.0000000000000000e+00;
acadoVariables.mu[156] -= + acadoWorkspace.y[190]*acadoWorkspace.evHx[160];
acadoVariables.mu[157] = 0.0000000000000000e+00;
acadoVariables.mu[157] -= + acadoWorkspace.y[190]*acadoWorkspace.evHx[161];
acadoVariables.mu[158] = 0.0000000000000000e+00;
acadoVariables.mu[158] -= + acadoWorkspace.y[190]*acadoWorkspace.evHx[162];
acadoVariables.mu[159] = 0.0000000000000000e+00;
acadoVariables.mu[159] -= + acadoWorkspace.y[190]*acadoWorkspace.evHx[163];
acado_expansionStep2( &(acadoWorkspace.QDy[ 160 ]), &(acadoWorkspace.Q1[ 640 ]), &(acadoWorkspace.sbar[ 160 ]), &(acadoWorkspace.S1[ 480 ]), &(acadoWorkspace.x[ 120 ]), &(acadoWorkspace.evGx[ 640 ]), &(acadoVariables.mu[ 156 ]), &(acadoVariables.mu[ 160 ]) );
acadoVariables.mu[152] = 0.0000000000000000e+00;
acadoVariables.mu[152] -= + acadoWorkspace.y[189]*acadoWorkspace.evHx[156];
acadoVariables.mu[153] = 0.0000000000000000e+00;
acadoVariables.mu[153] -= + acadoWorkspace.y[189]*acadoWorkspace.evHx[157];
acadoVariables.mu[154] = 0.0000000000000000e+00;
acadoVariables.mu[154] -= + acadoWorkspace.y[189]*acadoWorkspace.evHx[158];
acadoVariables.mu[155] = 0.0000000000000000e+00;
acadoVariables.mu[155] -= + acadoWorkspace.y[189]*acadoWorkspace.evHx[159];
acado_expansionStep2( &(acadoWorkspace.QDy[ 156 ]), &(acadoWorkspace.Q1[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.S1[ 468 ]), &(acadoWorkspace.x[ 117 ]), &(acadoWorkspace.evGx[ 624 ]), &(acadoVariables.mu[ 152 ]), &(acadoVariables.mu[ 156 ]) );
acadoVariables.mu[148] = 0.0000000000000000e+00;
acadoVariables.mu[148] -= + acadoWorkspace.y[188]*acadoWorkspace.evHx[152];
acadoVariables.mu[149] = 0.0000000000000000e+00;
acadoVariables.mu[149] -= + acadoWorkspace.y[188]*acadoWorkspace.evHx[153];
acadoVariables.mu[150] = 0.0000000000000000e+00;
acadoVariables.mu[150] -= + acadoWorkspace.y[188]*acadoWorkspace.evHx[154];
acadoVariables.mu[151] = 0.0000000000000000e+00;
acadoVariables.mu[151] -= + acadoWorkspace.y[188]*acadoWorkspace.evHx[155];
acado_expansionStep2( &(acadoWorkspace.QDy[ 152 ]), &(acadoWorkspace.Q1[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.S1[ 456 ]), &(acadoWorkspace.x[ 114 ]), &(acadoWorkspace.evGx[ 608 ]), &(acadoVariables.mu[ 148 ]), &(acadoVariables.mu[ 152 ]) );
acadoVariables.mu[144] = 0.0000000000000000e+00;
acadoVariables.mu[144] -= + acadoWorkspace.y[187]*acadoWorkspace.evHx[148];
acadoVariables.mu[145] = 0.0000000000000000e+00;
acadoVariables.mu[145] -= + acadoWorkspace.y[187]*acadoWorkspace.evHx[149];
acadoVariables.mu[146] = 0.0000000000000000e+00;
acadoVariables.mu[146] -= + acadoWorkspace.y[187]*acadoWorkspace.evHx[150];
acadoVariables.mu[147] = 0.0000000000000000e+00;
acadoVariables.mu[147] -= + acadoWorkspace.y[187]*acadoWorkspace.evHx[151];
acado_expansionStep2( &(acadoWorkspace.QDy[ 148 ]), &(acadoWorkspace.Q1[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.S1[ 444 ]), &(acadoWorkspace.x[ 111 ]), &(acadoWorkspace.evGx[ 592 ]), &(acadoVariables.mu[ 144 ]), &(acadoVariables.mu[ 148 ]) );
acadoVariables.mu[140] = 0.0000000000000000e+00;
acadoVariables.mu[140] -= + acadoWorkspace.y[186]*acadoWorkspace.evHx[144];
acadoVariables.mu[141] = 0.0000000000000000e+00;
acadoVariables.mu[141] -= + acadoWorkspace.y[186]*acadoWorkspace.evHx[145];
acadoVariables.mu[142] = 0.0000000000000000e+00;
acadoVariables.mu[142] -= + acadoWorkspace.y[186]*acadoWorkspace.evHx[146];
acadoVariables.mu[143] = 0.0000000000000000e+00;
acadoVariables.mu[143] -= + acadoWorkspace.y[186]*acadoWorkspace.evHx[147];
acado_expansionStep2( &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.S1[ 432 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoVariables.mu[ 140 ]), &(acadoVariables.mu[ 144 ]) );
acadoVariables.mu[136] = 0.0000000000000000e+00;
acadoVariables.mu[136] -= + acadoWorkspace.y[185]*acadoWorkspace.evHx[140];
acadoVariables.mu[137] = 0.0000000000000000e+00;
acadoVariables.mu[137] -= + acadoWorkspace.y[185]*acadoWorkspace.evHx[141];
acadoVariables.mu[138] = 0.0000000000000000e+00;
acadoVariables.mu[138] -= + acadoWorkspace.y[185]*acadoWorkspace.evHx[142];
acadoVariables.mu[139] = 0.0000000000000000e+00;
acadoVariables.mu[139] -= + acadoWorkspace.y[185]*acadoWorkspace.evHx[143];
acado_expansionStep2( &(acadoWorkspace.QDy[ 140 ]), &(acadoWorkspace.Q1[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.S1[ 420 ]), &(acadoWorkspace.x[ 105 ]), &(acadoWorkspace.evGx[ 560 ]), &(acadoVariables.mu[ 136 ]), &(acadoVariables.mu[ 140 ]) );
acadoVariables.mu[132] = 0.0000000000000000e+00;
acadoVariables.mu[132] -= + acadoWorkspace.y[184]*acadoWorkspace.evHx[136];
acadoVariables.mu[133] = 0.0000000000000000e+00;
acadoVariables.mu[133] -= + acadoWorkspace.y[184]*acadoWorkspace.evHx[137];
acadoVariables.mu[134] = 0.0000000000000000e+00;
acadoVariables.mu[134] -= + acadoWorkspace.y[184]*acadoWorkspace.evHx[138];
acadoVariables.mu[135] = 0.0000000000000000e+00;
acadoVariables.mu[135] -= + acadoWorkspace.y[184]*acadoWorkspace.evHx[139];
acado_expansionStep2( &(acadoWorkspace.QDy[ 136 ]), &(acadoWorkspace.Q1[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.S1[ 408 ]), &(acadoWorkspace.x[ 102 ]), &(acadoWorkspace.evGx[ 544 ]), &(acadoVariables.mu[ 132 ]), &(acadoVariables.mu[ 136 ]) );
acadoVariables.mu[128] = 0.0000000000000000e+00;
acadoVariables.mu[128] -= + acadoWorkspace.y[183]*acadoWorkspace.evHx[132];
acadoVariables.mu[129] = 0.0000000000000000e+00;
acadoVariables.mu[129] -= + acadoWorkspace.y[183]*acadoWorkspace.evHx[133];
acadoVariables.mu[130] = 0.0000000000000000e+00;
acadoVariables.mu[130] -= + acadoWorkspace.y[183]*acadoWorkspace.evHx[134];
acadoVariables.mu[131] = 0.0000000000000000e+00;
acadoVariables.mu[131] -= + acadoWorkspace.y[183]*acadoWorkspace.evHx[135];
acado_expansionStep2( &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.Q1[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.S1[ 396 ]), &(acadoWorkspace.x[ 99 ]), &(acadoWorkspace.evGx[ 528 ]), &(acadoVariables.mu[ 128 ]), &(acadoVariables.mu[ 132 ]) );
acadoVariables.mu[124] = 0.0000000000000000e+00;
acadoVariables.mu[124] -= + acadoWorkspace.y[182]*acadoWorkspace.evHx[128];
acadoVariables.mu[125] = 0.0000000000000000e+00;
acadoVariables.mu[125] -= + acadoWorkspace.y[182]*acadoWorkspace.evHx[129];
acadoVariables.mu[126] = 0.0000000000000000e+00;
acadoVariables.mu[126] -= + acadoWorkspace.y[182]*acadoWorkspace.evHx[130];
acadoVariables.mu[127] = 0.0000000000000000e+00;
acadoVariables.mu[127] -= + acadoWorkspace.y[182]*acadoWorkspace.evHx[131];
acado_expansionStep2( &(acadoWorkspace.QDy[ 128 ]), &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.S1[ 384 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoVariables.mu[ 124 ]), &(acadoVariables.mu[ 128 ]) );
acadoVariables.mu[120] = 0.0000000000000000e+00;
acadoVariables.mu[120] -= + acadoWorkspace.y[181]*acadoWorkspace.evHx[124];
acadoVariables.mu[121] = 0.0000000000000000e+00;
acadoVariables.mu[121] -= + acadoWorkspace.y[181]*acadoWorkspace.evHx[125];
acadoVariables.mu[122] = 0.0000000000000000e+00;
acadoVariables.mu[122] -= + acadoWorkspace.y[181]*acadoWorkspace.evHx[126];
acadoVariables.mu[123] = 0.0000000000000000e+00;
acadoVariables.mu[123] -= + acadoWorkspace.y[181]*acadoWorkspace.evHx[127];
acado_expansionStep2( &(acadoWorkspace.QDy[ 124 ]), &(acadoWorkspace.Q1[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.S1[ 372 ]), &(acadoWorkspace.x[ 93 ]), &(acadoWorkspace.evGx[ 496 ]), &(acadoVariables.mu[ 120 ]), &(acadoVariables.mu[ 124 ]) );
acadoVariables.mu[116] = 0.0000000000000000e+00;
acadoVariables.mu[116] -= + acadoWorkspace.y[180]*acadoWorkspace.evHx[120];
acadoVariables.mu[117] = 0.0000000000000000e+00;
acadoVariables.mu[117] -= + acadoWorkspace.y[180]*acadoWorkspace.evHx[121];
acadoVariables.mu[118] = 0.0000000000000000e+00;
acadoVariables.mu[118] -= + acadoWorkspace.y[180]*acadoWorkspace.evHx[122];
acadoVariables.mu[119] = 0.0000000000000000e+00;
acadoVariables.mu[119] -= + acadoWorkspace.y[180]*acadoWorkspace.evHx[123];
acado_expansionStep2( &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.Q1[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.S1[ 360 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.evGx[ 480 ]), &(acadoVariables.mu[ 116 ]), &(acadoVariables.mu[ 120 ]) );
acadoVariables.mu[112] = 0.0000000000000000e+00;
acadoVariables.mu[112] -= + acadoWorkspace.y[179]*acadoWorkspace.evHx[116];
acadoVariables.mu[113] = 0.0000000000000000e+00;
acadoVariables.mu[113] -= + acadoWorkspace.y[179]*acadoWorkspace.evHx[117];
acadoVariables.mu[114] = 0.0000000000000000e+00;
acadoVariables.mu[114] -= + acadoWorkspace.y[179]*acadoWorkspace.evHx[118];
acadoVariables.mu[115] = 0.0000000000000000e+00;
acadoVariables.mu[115] -= + acadoWorkspace.y[179]*acadoWorkspace.evHx[119];
acado_expansionStep2( &(acadoWorkspace.QDy[ 116 ]), &(acadoWorkspace.Q1[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.S1[ 348 ]), &(acadoWorkspace.x[ 87 ]), &(acadoWorkspace.evGx[ 464 ]), &(acadoVariables.mu[ 112 ]), &(acadoVariables.mu[ 116 ]) );
acadoVariables.mu[108] = 0.0000000000000000e+00;
acadoVariables.mu[108] -= + acadoWorkspace.y[178]*acadoWorkspace.evHx[112];
acadoVariables.mu[109] = 0.0000000000000000e+00;
acadoVariables.mu[109] -= + acadoWorkspace.y[178]*acadoWorkspace.evHx[113];
acadoVariables.mu[110] = 0.0000000000000000e+00;
acadoVariables.mu[110] -= + acadoWorkspace.y[178]*acadoWorkspace.evHx[114];
acadoVariables.mu[111] = 0.0000000000000000e+00;
acadoVariables.mu[111] -= + acadoWorkspace.y[178]*acadoWorkspace.evHx[115];
acado_expansionStep2( &(acadoWorkspace.QDy[ 112 ]), &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.S1[ 336 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoVariables.mu[ 108 ]), &(acadoVariables.mu[ 112 ]) );
acadoVariables.mu[104] = 0.0000000000000000e+00;
acadoVariables.mu[104] -= + acadoWorkspace.y[177]*acadoWorkspace.evHx[108];
acadoVariables.mu[105] = 0.0000000000000000e+00;
acadoVariables.mu[105] -= + acadoWorkspace.y[177]*acadoWorkspace.evHx[109];
acadoVariables.mu[106] = 0.0000000000000000e+00;
acadoVariables.mu[106] -= + acadoWorkspace.y[177]*acadoWorkspace.evHx[110];
acadoVariables.mu[107] = 0.0000000000000000e+00;
acadoVariables.mu[107] -= + acadoWorkspace.y[177]*acadoWorkspace.evHx[111];
acado_expansionStep2( &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.S1[ 324 ]), &(acadoWorkspace.x[ 81 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoVariables.mu[ 104 ]), &(acadoVariables.mu[ 108 ]) );
acadoVariables.mu[100] = 0.0000000000000000e+00;
acadoVariables.mu[100] -= + acadoWorkspace.y[176]*acadoWorkspace.evHx[104];
acadoVariables.mu[101] = 0.0000000000000000e+00;
acadoVariables.mu[101] -= + acadoWorkspace.y[176]*acadoWorkspace.evHx[105];
acadoVariables.mu[102] = 0.0000000000000000e+00;
acadoVariables.mu[102] -= + acadoWorkspace.y[176]*acadoWorkspace.evHx[106];
acadoVariables.mu[103] = 0.0000000000000000e+00;
acadoVariables.mu[103] -= + acadoWorkspace.y[176]*acadoWorkspace.evHx[107];
acado_expansionStep2( &(acadoWorkspace.QDy[ 104 ]), &(acadoWorkspace.Q1[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.S1[ 312 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.evGx[ 416 ]), &(acadoVariables.mu[ 100 ]), &(acadoVariables.mu[ 104 ]) );
acadoVariables.mu[96] = 0.0000000000000000e+00;
acadoVariables.mu[96] -= + acadoWorkspace.y[175]*acadoWorkspace.evHx[100];
acadoVariables.mu[97] = 0.0000000000000000e+00;
acadoVariables.mu[97] -= + acadoWorkspace.y[175]*acadoWorkspace.evHx[101];
acadoVariables.mu[98] = 0.0000000000000000e+00;
acadoVariables.mu[98] -= + acadoWorkspace.y[175]*acadoWorkspace.evHx[102];
acadoVariables.mu[99] = 0.0000000000000000e+00;
acadoVariables.mu[99] -= + acadoWorkspace.y[175]*acadoWorkspace.evHx[103];
acado_expansionStep2( &(acadoWorkspace.QDy[ 100 ]), &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.S1[ 300 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoVariables.mu[ 96 ]), &(acadoVariables.mu[ 100 ]) );
acadoVariables.mu[92] = 0.0000000000000000e+00;
acadoVariables.mu[92] -= + acadoWorkspace.y[174]*acadoWorkspace.evHx[96];
acadoVariables.mu[93] = 0.0000000000000000e+00;
acadoVariables.mu[93] -= + acadoWorkspace.y[174]*acadoWorkspace.evHx[97];
acadoVariables.mu[94] = 0.0000000000000000e+00;
acadoVariables.mu[94] -= + acadoWorkspace.y[174]*acadoWorkspace.evHx[98];
acadoVariables.mu[95] = 0.0000000000000000e+00;
acadoVariables.mu[95] -= + acadoWorkspace.y[174]*acadoWorkspace.evHx[99];
acado_expansionStep2( &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.S1[ 288 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoVariables.mu[ 92 ]), &(acadoVariables.mu[ 96 ]) );
acadoVariables.mu[88] = 0.0000000000000000e+00;
acadoVariables.mu[88] -= + acadoWorkspace.y[173]*acadoWorkspace.evHx[92];
acadoVariables.mu[89] = 0.0000000000000000e+00;
acadoVariables.mu[89] -= + acadoWorkspace.y[173]*acadoWorkspace.evHx[93];
acadoVariables.mu[90] = 0.0000000000000000e+00;
acadoVariables.mu[90] -= + acadoWorkspace.y[173]*acadoWorkspace.evHx[94];
acadoVariables.mu[91] = 0.0000000000000000e+00;
acadoVariables.mu[91] -= + acadoWorkspace.y[173]*acadoWorkspace.evHx[95];
acado_expansionStep2( &(acadoWorkspace.QDy[ 92 ]), &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.S1[ 276 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.evGx[ 368 ]), &(acadoVariables.mu[ 88 ]), &(acadoVariables.mu[ 92 ]) );
acadoVariables.mu[84] = 0.0000000000000000e+00;
acadoVariables.mu[84] -= + acadoWorkspace.y[172]*acadoWorkspace.evHx[88];
acadoVariables.mu[85] = 0.0000000000000000e+00;
acadoVariables.mu[85] -= + acadoWorkspace.y[172]*acadoWorkspace.evHx[89];
acadoVariables.mu[86] = 0.0000000000000000e+00;
acadoVariables.mu[86] -= + acadoWorkspace.y[172]*acadoWorkspace.evHx[90];
acadoVariables.mu[87] = 0.0000000000000000e+00;
acadoVariables.mu[87] -= + acadoWorkspace.y[172]*acadoWorkspace.evHx[91];
acado_expansionStep2( &(acadoWorkspace.QDy[ 88 ]), &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.S1[ 264 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.evGx[ 352 ]), &(acadoVariables.mu[ 84 ]), &(acadoVariables.mu[ 88 ]) );
acadoVariables.mu[80] = 0.0000000000000000e+00;
acadoVariables.mu[80] -= + acadoWorkspace.y[171]*acadoWorkspace.evHx[84];
acadoVariables.mu[81] = 0.0000000000000000e+00;
acadoVariables.mu[81] -= + acadoWorkspace.y[171]*acadoWorkspace.evHx[85];
acadoVariables.mu[82] = 0.0000000000000000e+00;
acadoVariables.mu[82] -= + acadoWorkspace.y[171]*acadoWorkspace.evHx[86];
acadoVariables.mu[83] = 0.0000000000000000e+00;
acadoVariables.mu[83] -= + acadoWorkspace.y[171]*acadoWorkspace.evHx[87];
acado_expansionStep2( &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.S1[ 252 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.evGx[ 336 ]), &(acadoVariables.mu[ 80 ]), &(acadoVariables.mu[ 84 ]) );
acadoVariables.mu[76] = 0.0000000000000000e+00;
acadoVariables.mu[76] -= + acadoWorkspace.y[170]*acadoWorkspace.evHx[80];
acadoVariables.mu[77] = 0.0000000000000000e+00;
acadoVariables.mu[77] -= + acadoWorkspace.y[170]*acadoWorkspace.evHx[81];
acadoVariables.mu[78] = 0.0000000000000000e+00;
acadoVariables.mu[78] -= + acadoWorkspace.y[170]*acadoWorkspace.evHx[82];
acadoVariables.mu[79] = 0.0000000000000000e+00;
acadoVariables.mu[79] -= + acadoWorkspace.y[170]*acadoWorkspace.evHx[83];
acado_expansionStep2( &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoVariables.mu[ 76 ]), &(acadoVariables.mu[ 80 ]) );
acadoVariables.mu[72] = 0.0000000000000000e+00;
acadoVariables.mu[72] -= + acadoWorkspace.y[169]*acadoWorkspace.evHx[76];
acadoVariables.mu[73] = 0.0000000000000000e+00;
acadoVariables.mu[73] -= + acadoWorkspace.y[169]*acadoWorkspace.evHx[77];
acadoVariables.mu[74] = 0.0000000000000000e+00;
acadoVariables.mu[74] -= + acadoWorkspace.y[169]*acadoWorkspace.evHx[78];
acadoVariables.mu[75] = 0.0000000000000000e+00;
acadoVariables.mu[75] -= + acadoWorkspace.y[169]*acadoWorkspace.evHx[79];
acado_expansionStep2( &(acadoWorkspace.QDy[ 76 ]), &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.S1[ 228 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.evGx[ 304 ]), &(acadoVariables.mu[ 72 ]), &(acadoVariables.mu[ 76 ]) );
acadoVariables.mu[68] = 0.0000000000000000e+00;
acadoVariables.mu[68] -= + acadoWorkspace.y[168]*acadoWorkspace.evHx[72];
acadoVariables.mu[69] = 0.0000000000000000e+00;
acadoVariables.mu[69] -= + acadoWorkspace.y[168]*acadoWorkspace.evHx[73];
acadoVariables.mu[70] = 0.0000000000000000e+00;
acadoVariables.mu[70] -= + acadoWorkspace.y[168]*acadoWorkspace.evHx[74];
acadoVariables.mu[71] = 0.0000000000000000e+00;
acadoVariables.mu[71] -= + acadoWorkspace.y[168]*acadoWorkspace.evHx[75];
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoVariables.mu[ 68 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[64] = 0.0000000000000000e+00;
acadoVariables.mu[64] -= + acadoWorkspace.y[167]*acadoWorkspace.evHx[68];
acadoVariables.mu[65] = 0.0000000000000000e+00;
acadoVariables.mu[65] -= + acadoWorkspace.y[167]*acadoWorkspace.evHx[69];
acadoVariables.mu[66] = 0.0000000000000000e+00;
acadoVariables.mu[66] -= + acadoWorkspace.y[167]*acadoWorkspace.evHx[70];
acadoVariables.mu[67] = 0.0000000000000000e+00;
acadoVariables.mu[67] -= + acadoWorkspace.y[167]*acadoWorkspace.evHx[71];
acado_expansionStep2( &(acadoWorkspace.QDy[ 68 ]), &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.S1[ 204 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.evGx[ 272 ]), &(acadoVariables.mu[ 64 ]), &(acadoVariables.mu[ 68 ]) );
acadoVariables.mu[60] = 0.0000000000000000e+00;
acadoVariables.mu[60] -= + acadoWorkspace.y[166]*acadoWorkspace.evHx[64];
acadoVariables.mu[61] = 0.0000000000000000e+00;
acadoVariables.mu[61] -= + acadoWorkspace.y[166]*acadoWorkspace.evHx[65];
acadoVariables.mu[62] = 0.0000000000000000e+00;
acadoVariables.mu[62] -= + acadoWorkspace.y[166]*acadoWorkspace.evHx[66];
acadoVariables.mu[63] = 0.0000000000000000e+00;
acadoVariables.mu[63] -= + acadoWorkspace.y[166]*acadoWorkspace.evHx[67];
acado_expansionStep2( &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.S1[ 192 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoVariables.mu[ 60 ]), &(acadoVariables.mu[ 64 ]) );
acadoVariables.mu[56] = 0.0000000000000000e+00;
acadoVariables.mu[56] -= + acadoWorkspace.y[165]*acadoWorkspace.evHx[60];
acadoVariables.mu[57] = 0.0000000000000000e+00;
acadoVariables.mu[57] -= + acadoWorkspace.y[165]*acadoWorkspace.evHx[61];
acadoVariables.mu[58] = 0.0000000000000000e+00;
acadoVariables.mu[58] -= + acadoWorkspace.y[165]*acadoWorkspace.evHx[62];
acadoVariables.mu[59] = 0.0000000000000000e+00;
acadoVariables.mu[59] -= + acadoWorkspace.y[165]*acadoWorkspace.evHx[63];
acado_expansionStep2( &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.S1[ 180 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.evGx[ 240 ]), &(acadoVariables.mu[ 56 ]), &(acadoVariables.mu[ 60 ]) );
acadoVariables.mu[52] = 0.0000000000000000e+00;
acadoVariables.mu[52] -= + acadoWorkspace.y[164]*acadoWorkspace.evHx[56];
acadoVariables.mu[53] = 0.0000000000000000e+00;
acadoVariables.mu[53] -= + acadoWorkspace.y[164]*acadoWorkspace.evHx[57];
acadoVariables.mu[54] = 0.0000000000000000e+00;
acadoVariables.mu[54] -= + acadoWorkspace.y[164]*acadoWorkspace.evHx[58];
acadoVariables.mu[55] = 0.0000000000000000e+00;
acadoVariables.mu[55] -= + acadoWorkspace.y[164]*acadoWorkspace.evHx[59];
acado_expansionStep2( &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.S1[ 168 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.evGx[ 224 ]), &(acadoVariables.mu[ 52 ]), &(acadoVariables.mu[ 56 ]) );
acadoVariables.mu[48] = 0.0000000000000000e+00;
acadoVariables.mu[48] -= + acadoWorkspace.y[163]*acadoWorkspace.evHx[52];
acadoVariables.mu[49] = 0.0000000000000000e+00;
acadoVariables.mu[49] -= + acadoWorkspace.y[163]*acadoWorkspace.evHx[53];
acadoVariables.mu[50] = 0.0000000000000000e+00;
acadoVariables.mu[50] -= + acadoWorkspace.y[163]*acadoWorkspace.evHx[54];
acadoVariables.mu[51] = 0.0000000000000000e+00;
acadoVariables.mu[51] -= + acadoWorkspace.y[163]*acadoWorkspace.evHx[55];
acado_expansionStep2( &(acadoWorkspace.QDy[ 52 ]), &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.S1[ 156 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.evGx[ 208 ]), &(acadoVariables.mu[ 48 ]), &(acadoVariables.mu[ 52 ]) );
acadoVariables.mu[44] = 0.0000000000000000e+00;
acadoVariables.mu[44] -= + acadoWorkspace.y[162]*acadoWorkspace.evHx[48];
acadoVariables.mu[45] = 0.0000000000000000e+00;
acadoVariables.mu[45] -= + acadoWorkspace.y[162]*acadoWorkspace.evHx[49];
acadoVariables.mu[46] = 0.0000000000000000e+00;
acadoVariables.mu[46] -= + acadoWorkspace.y[162]*acadoWorkspace.evHx[50];
acadoVariables.mu[47] = 0.0000000000000000e+00;
acadoVariables.mu[47] -= + acadoWorkspace.y[162]*acadoWorkspace.evHx[51];
acado_expansionStep2( &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoVariables.mu[ 44 ]), &(acadoVariables.mu[ 48 ]) );
acadoVariables.mu[40] = 0.0000000000000000e+00;
acadoVariables.mu[40] -= + acadoWorkspace.y[161]*acadoWorkspace.evHx[44];
acadoVariables.mu[41] = 0.0000000000000000e+00;
acadoVariables.mu[41] -= + acadoWorkspace.y[161]*acadoWorkspace.evHx[45];
acadoVariables.mu[42] = 0.0000000000000000e+00;
acadoVariables.mu[42] -= + acadoWorkspace.y[161]*acadoWorkspace.evHx[46];
acadoVariables.mu[43] = 0.0000000000000000e+00;
acadoVariables.mu[43] -= + acadoWorkspace.y[161]*acadoWorkspace.evHx[47];
acado_expansionStep2( &(acadoWorkspace.QDy[ 44 ]), &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.S1[ 132 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.evGx[ 176 ]), &(acadoVariables.mu[ 40 ]), &(acadoVariables.mu[ 44 ]) );
acadoVariables.mu[36] = 0.0000000000000000e+00;
acadoVariables.mu[36] -= + acadoWorkspace.y[160]*acadoWorkspace.evHx[40];
acadoVariables.mu[37] = 0.0000000000000000e+00;
acadoVariables.mu[37] -= + acadoWorkspace.y[160]*acadoWorkspace.evHx[41];
acadoVariables.mu[38] = 0.0000000000000000e+00;
acadoVariables.mu[38] -= + acadoWorkspace.y[160]*acadoWorkspace.evHx[42];
acadoVariables.mu[39] = 0.0000000000000000e+00;
acadoVariables.mu[39] -= + acadoWorkspace.y[160]*acadoWorkspace.evHx[43];
acado_expansionStep2( &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.evGx[ 160 ]), &(acadoVariables.mu[ 36 ]), &(acadoVariables.mu[ 40 ]) );
acadoVariables.mu[32] = 0.0000000000000000e+00;
acadoVariables.mu[32] -= + acadoWorkspace.y[159]*acadoWorkspace.evHx[36];
acadoVariables.mu[33] = 0.0000000000000000e+00;
acadoVariables.mu[33] -= + acadoWorkspace.y[159]*acadoWorkspace.evHx[37];
acadoVariables.mu[34] = 0.0000000000000000e+00;
acadoVariables.mu[34] -= + acadoWorkspace.y[159]*acadoWorkspace.evHx[38];
acadoVariables.mu[35] = 0.0000000000000000e+00;
acadoVariables.mu[35] -= + acadoWorkspace.y[159]*acadoWorkspace.evHx[39];
acado_expansionStep2( &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoVariables.mu[ 32 ]), &(acadoVariables.mu[ 36 ]) );
acadoVariables.mu[28] = 0.0000000000000000e+00;
acadoVariables.mu[28] -= + acadoWorkspace.y[158]*acadoWorkspace.evHx[32];
acadoVariables.mu[29] = 0.0000000000000000e+00;
acadoVariables.mu[29] -= + acadoWorkspace.y[158]*acadoWorkspace.evHx[33];
acadoVariables.mu[30] = 0.0000000000000000e+00;
acadoVariables.mu[30] -= + acadoWorkspace.y[158]*acadoWorkspace.evHx[34];
acadoVariables.mu[31] = 0.0000000000000000e+00;
acadoVariables.mu[31] -= + acadoWorkspace.y[158]*acadoWorkspace.evHx[35];
acado_expansionStep2( &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoVariables.mu[ 28 ]), &(acadoVariables.mu[ 32 ]) );
acadoVariables.mu[24] = 0.0000000000000000e+00;
acadoVariables.mu[24] -= + acadoWorkspace.y[157]*acadoWorkspace.evHx[28];
acadoVariables.mu[25] = 0.0000000000000000e+00;
acadoVariables.mu[25] -= + acadoWorkspace.y[157]*acadoWorkspace.evHx[29];
acadoVariables.mu[26] = 0.0000000000000000e+00;
acadoVariables.mu[26] -= + acadoWorkspace.y[157]*acadoWorkspace.evHx[30];
acadoVariables.mu[27] = 0.0000000000000000e+00;
acadoVariables.mu[27] -= + acadoWorkspace.y[157]*acadoWorkspace.evHx[31];
acado_expansionStep2( &(acadoWorkspace.QDy[ 28 ]), &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.evGx[ 112 ]), &(acadoVariables.mu[ 24 ]), &(acadoVariables.mu[ 28 ]) );
acadoVariables.mu[20] = 0.0000000000000000e+00;
acadoVariables.mu[20] -= + acadoWorkspace.y[156]*acadoWorkspace.evHx[24];
acadoVariables.mu[21] = 0.0000000000000000e+00;
acadoVariables.mu[21] -= + acadoWorkspace.y[156]*acadoWorkspace.evHx[25];
acadoVariables.mu[22] = 0.0000000000000000e+00;
acadoVariables.mu[22] -= + acadoWorkspace.y[156]*acadoWorkspace.evHx[26];
acadoVariables.mu[23] = 0.0000000000000000e+00;
acadoVariables.mu[23] -= + acadoWorkspace.y[156]*acadoWorkspace.evHx[27];
acado_expansionStep2( &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 96 ]), &(acadoVariables.mu[ 20 ]), &(acadoVariables.mu[ 24 ]) );
acadoVariables.mu[16] = 0.0000000000000000e+00;
acadoVariables.mu[16] -= + acadoWorkspace.y[155]*acadoWorkspace.evHx[20];
acadoVariables.mu[17] = 0.0000000000000000e+00;
acadoVariables.mu[17] -= + acadoWorkspace.y[155]*acadoWorkspace.evHx[21];
acadoVariables.mu[18] = 0.0000000000000000e+00;
acadoVariables.mu[18] -= + acadoWorkspace.y[155]*acadoWorkspace.evHx[22];
acadoVariables.mu[19] = 0.0000000000000000e+00;
acadoVariables.mu[19] -= + acadoWorkspace.y[155]*acadoWorkspace.evHx[23];
acado_expansionStep2( &(acadoWorkspace.QDy[ 20 ]), &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.evGx[ 80 ]), &(acadoVariables.mu[ 16 ]), &(acadoVariables.mu[ 20 ]) );
acadoVariables.mu[12] = 0.0000000000000000e+00;
acadoVariables.mu[12] -= + acadoWorkspace.y[154]*acadoWorkspace.evHx[16];
acadoVariables.mu[13] = 0.0000000000000000e+00;
acadoVariables.mu[13] -= + acadoWorkspace.y[154]*acadoWorkspace.evHx[17];
acadoVariables.mu[14] = 0.0000000000000000e+00;
acadoVariables.mu[14] -= + acadoWorkspace.y[154]*acadoWorkspace.evHx[18];
acadoVariables.mu[15] = 0.0000000000000000e+00;
acadoVariables.mu[15] -= + acadoWorkspace.y[154]*acadoWorkspace.evHx[19];
acado_expansionStep2( &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoVariables.mu[ 12 ]), &(acadoVariables.mu[ 16 ]) );
acadoVariables.mu[8] = 0.0000000000000000e+00;
acadoVariables.mu[8] -= + acadoWorkspace.y[153]*acadoWorkspace.evHx[12];
acadoVariables.mu[9] = 0.0000000000000000e+00;
acadoVariables.mu[9] -= + acadoWorkspace.y[153]*acadoWorkspace.evHx[13];
acadoVariables.mu[10] = 0.0000000000000000e+00;
acadoVariables.mu[10] -= + acadoWorkspace.y[153]*acadoWorkspace.evHx[14];
acadoVariables.mu[11] = 0.0000000000000000e+00;
acadoVariables.mu[11] -= + acadoWorkspace.y[153]*acadoWorkspace.evHx[15];
acado_expansionStep2( &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.evGx[ 48 ]), &(acadoVariables.mu[ 8 ]), &(acadoVariables.mu[ 12 ]) );
acadoVariables.mu[4] = 0.0000000000000000e+00;
acadoVariables.mu[4] -= + acadoWorkspace.y[152]*acadoWorkspace.evHx[8];
acadoVariables.mu[5] = 0.0000000000000000e+00;
acadoVariables.mu[5] -= + acadoWorkspace.y[152]*acadoWorkspace.evHx[9];
acadoVariables.mu[6] = 0.0000000000000000e+00;
acadoVariables.mu[6] -= + acadoWorkspace.y[152]*acadoWorkspace.evHx[10];
acadoVariables.mu[7] = 0.0000000000000000e+00;
acadoVariables.mu[7] -= + acadoWorkspace.y[152]*acadoWorkspace.evHx[11];
acado_expansionStep2( &(acadoWorkspace.QDy[ 8 ]), &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 32 ]), &(acadoVariables.mu[ 4 ]), &(acadoVariables.mu[ 8 ]) );
acadoVariables.mu[0] = 0.0000000000000000e+00;
acadoVariables.mu[0] -= + acadoWorkspace.y[151]*acadoWorkspace.evHx[4];
acadoVariables.mu[1] = 0.0000000000000000e+00;
acadoVariables.mu[1] -= + acadoWorkspace.y[151]*acadoWorkspace.evHx[5];
acadoVariables.mu[2] = 0.0000000000000000e+00;
acadoVariables.mu[2] -= + acadoWorkspace.y[151]*acadoWorkspace.evHx[6];
acadoVariables.mu[3] = 0.0000000000000000e+00;
acadoVariables.mu[3] -= + acadoWorkspace.y[151]*acadoWorkspace.evHx[7];
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
acadoVariables.lbValues[0] = -1.0000000000000000e+00;
acadoVariables.lbValues[1] = -1.0000000000000000e+00;
acadoVariables.lbValues[2] = -1.0000000000000000e+12;
acadoVariables.lbValues[3] = -1.0000000000000000e+00;
acadoVariables.lbValues[4] = -1.0000000000000000e+00;
acadoVariables.lbValues[5] = -1.0000000000000000e+12;
acadoVariables.lbValues[6] = -1.0000000000000000e+00;
acadoVariables.lbValues[7] = -1.0000000000000000e+00;
acadoVariables.lbValues[8] = -1.0000000000000000e+12;
acadoVariables.lbValues[9] = -1.0000000000000000e+00;
acadoVariables.lbValues[10] = -1.0000000000000000e+00;
acadoVariables.lbValues[11] = -1.0000000000000000e+12;
acadoVariables.lbValues[12] = -1.0000000000000000e+00;
acadoVariables.lbValues[13] = -1.0000000000000000e+00;
acadoVariables.lbValues[14] = -1.0000000000000000e+12;
acadoVariables.lbValues[15] = -1.0000000000000000e+00;
acadoVariables.lbValues[16] = -1.0000000000000000e+00;
acadoVariables.lbValues[17] = -1.0000000000000000e+12;
acadoVariables.lbValues[18] = -1.0000000000000000e+00;
acadoVariables.lbValues[19] = -1.0000000000000000e+00;
acadoVariables.lbValues[20] = -1.0000000000000000e+12;
acadoVariables.lbValues[21] = -1.0000000000000000e+00;
acadoVariables.lbValues[22] = -1.0000000000000000e+00;
acadoVariables.lbValues[23] = -1.0000000000000000e+12;
acadoVariables.lbValues[24] = -1.0000000000000000e+00;
acadoVariables.lbValues[25] = -1.0000000000000000e+00;
acadoVariables.lbValues[26] = -1.0000000000000000e+12;
acadoVariables.lbValues[27] = -1.0000000000000000e+00;
acadoVariables.lbValues[28] = -1.0000000000000000e+00;
acadoVariables.lbValues[29] = -1.0000000000000000e+12;
acadoVariables.lbValues[30] = -1.0000000000000000e+00;
acadoVariables.lbValues[31] = -1.0000000000000000e+00;
acadoVariables.lbValues[32] = -1.0000000000000000e+12;
acadoVariables.lbValues[33] = -1.0000000000000000e+00;
acadoVariables.lbValues[34] = -1.0000000000000000e+00;
acadoVariables.lbValues[35] = -1.0000000000000000e+12;
acadoVariables.lbValues[36] = -1.0000000000000000e+00;
acadoVariables.lbValues[37] = -1.0000000000000000e+00;
acadoVariables.lbValues[38] = -1.0000000000000000e+12;
acadoVariables.lbValues[39] = -1.0000000000000000e+00;
acadoVariables.lbValues[40] = -1.0000000000000000e+00;
acadoVariables.lbValues[41] = -1.0000000000000000e+12;
acadoVariables.lbValues[42] = -1.0000000000000000e+00;
acadoVariables.lbValues[43] = -1.0000000000000000e+00;
acadoVariables.lbValues[44] = -1.0000000000000000e+12;
acadoVariables.lbValues[45] = -1.0000000000000000e+00;
acadoVariables.lbValues[46] = -1.0000000000000000e+00;
acadoVariables.lbValues[47] = -1.0000000000000000e+12;
acadoVariables.lbValues[48] = -1.0000000000000000e+00;
acadoVariables.lbValues[49] = -1.0000000000000000e+00;
acadoVariables.lbValues[50] = -1.0000000000000000e+12;
acadoVariables.lbValues[51] = -1.0000000000000000e+00;
acadoVariables.lbValues[52] = -1.0000000000000000e+00;
acadoVariables.lbValues[53] = -1.0000000000000000e+12;
acadoVariables.lbValues[54] = -1.0000000000000000e+00;
acadoVariables.lbValues[55] = -1.0000000000000000e+00;
acadoVariables.lbValues[56] = -1.0000000000000000e+12;
acadoVariables.lbValues[57] = -1.0000000000000000e+00;
acadoVariables.lbValues[58] = -1.0000000000000000e+00;
acadoVariables.lbValues[59] = -1.0000000000000000e+12;
acadoVariables.lbValues[60] = -1.0000000000000000e+00;
acadoVariables.lbValues[61] = -1.0000000000000000e+00;
acadoVariables.lbValues[62] = -1.0000000000000000e+12;
acadoVariables.lbValues[63] = -1.0000000000000000e+00;
acadoVariables.lbValues[64] = -1.0000000000000000e+00;
acadoVariables.lbValues[65] = -1.0000000000000000e+12;
acadoVariables.lbValues[66] = -1.0000000000000000e+00;
acadoVariables.lbValues[67] = -1.0000000000000000e+00;
acadoVariables.lbValues[68] = -1.0000000000000000e+12;
acadoVariables.lbValues[69] = -1.0000000000000000e+00;
acadoVariables.lbValues[70] = -1.0000000000000000e+00;
acadoVariables.lbValues[71] = -1.0000000000000000e+12;
acadoVariables.lbValues[72] = -1.0000000000000000e+00;
acadoVariables.lbValues[73] = -1.0000000000000000e+00;
acadoVariables.lbValues[74] = -1.0000000000000000e+12;
acadoVariables.lbValues[75] = -1.0000000000000000e+00;
acadoVariables.lbValues[76] = -1.0000000000000000e+00;
acadoVariables.lbValues[77] = -1.0000000000000000e+12;
acadoVariables.lbValues[78] = -1.0000000000000000e+00;
acadoVariables.lbValues[79] = -1.0000000000000000e+00;
acadoVariables.lbValues[80] = -1.0000000000000000e+12;
acadoVariables.lbValues[81] = -1.0000000000000000e+00;
acadoVariables.lbValues[82] = -1.0000000000000000e+00;
acadoVariables.lbValues[83] = -1.0000000000000000e+12;
acadoVariables.lbValues[84] = -1.0000000000000000e+00;
acadoVariables.lbValues[85] = -1.0000000000000000e+00;
acadoVariables.lbValues[86] = -1.0000000000000000e+12;
acadoVariables.lbValues[87] = -1.0000000000000000e+00;
acadoVariables.lbValues[88] = -1.0000000000000000e+00;
acadoVariables.lbValues[89] = -1.0000000000000000e+12;
acadoVariables.lbValues[90] = -1.0000000000000000e+00;
acadoVariables.lbValues[91] = -1.0000000000000000e+00;
acadoVariables.lbValues[92] = -1.0000000000000000e+12;
acadoVariables.lbValues[93] = -1.0000000000000000e+00;
acadoVariables.lbValues[94] = -1.0000000000000000e+00;
acadoVariables.lbValues[95] = -1.0000000000000000e+12;
acadoVariables.lbValues[96] = -1.0000000000000000e+00;
acadoVariables.lbValues[97] = -1.0000000000000000e+00;
acadoVariables.lbValues[98] = -1.0000000000000000e+12;
acadoVariables.lbValues[99] = -1.0000000000000000e+00;
acadoVariables.lbValues[100] = -1.0000000000000000e+00;
acadoVariables.lbValues[101] = -1.0000000000000000e+12;
acadoVariables.lbValues[102] = -1.0000000000000000e+00;
acadoVariables.lbValues[103] = -1.0000000000000000e+00;
acadoVariables.lbValues[104] = -1.0000000000000000e+12;
acadoVariables.lbValues[105] = -1.0000000000000000e+00;
acadoVariables.lbValues[106] = -1.0000000000000000e+00;
acadoVariables.lbValues[107] = -1.0000000000000000e+12;
acadoVariables.lbValues[108] = -1.0000000000000000e+00;
acadoVariables.lbValues[109] = -1.0000000000000000e+00;
acadoVariables.lbValues[110] = -1.0000000000000000e+12;
acadoVariables.lbValues[111] = -1.0000000000000000e+00;
acadoVariables.lbValues[112] = -1.0000000000000000e+00;
acadoVariables.lbValues[113] = -1.0000000000000000e+12;
acadoVariables.lbValues[114] = -1.0000000000000000e+00;
acadoVariables.lbValues[115] = -1.0000000000000000e+00;
acadoVariables.lbValues[116] = -1.0000000000000000e+12;
acadoVariables.lbValues[117] = -1.0000000000000000e+00;
acadoVariables.lbValues[118] = -1.0000000000000000e+00;
acadoVariables.lbValues[119] = -1.0000000000000000e+12;
acadoVariables.lbValues[120] = -1.0000000000000000e+00;
acadoVariables.lbValues[121] = -1.0000000000000000e+00;
acadoVariables.lbValues[122] = -1.0000000000000000e+12;
acadoVariables.lbValues[123] = -1.0000000000000000e+00;
acadoVariables.lbValues[124] = -1.0000000000000000e+00;
acadoVariables.lbValues[125] = -1.0000000000000000e+12;
acadoVariables.lbValues[126] = -1.0000000000000000e+00;
acadoVariables.lbValues[127] = -1.0000000000000000e+00;
acadoVariables.lbValues[128] = -1.0000000000000000e+12;
acadoVariables.lbValues[129] = -1.0000000000000000e+00;
acadoVariables.lbValues[130] = -1.0000000000000000e+00;
acadoVariables.lbValues[131] = -1.0000000000000000e+12;
acadoVariables.lbValues[132] = -1.0000000000000000e+00;
acadoVariables.lbValues[133] = -1.0000000000000000e+00;
acadoVariables.lbValues[134] = -1.0000000000000000e+12;
acadoVariables.lbValues[135] = -1.0000000000000000e+00;
acadoVariables.lbValues[136] = -1.0000000000000000e+00;
acadoVariables.lbValues[137] = -1.0000000000000000e+12;
acadoVariables.lbValues[138] = -1.0000000000000000e+00;
acadoVariables.lbValues[139] = -1.0000000000000000e+00;
acadoVariables.lbValues[140] = -1.0000000000000000e+12;
acadoVariables.lbValues[141] = -1.0000000000000000e+00;
acadoVariables.lbValues[142] = -1.0000000000000000e+00;
acadoVariables.lbValues[143] = -1.0000000000000000e+12;
acadoVariables.lbValues[144] = -1.0000000000000000e+00;
acadoVariables.lbValues[145] = -1.0000000000000000e+00;
acadoVariables.lbValues[146] = -1.0000000000000000e+12;
acadoVariables.lbValues[147] = -1.0000000000000000e+00;
acadoVariables.lbValues[148] = -1.0000000000000000e+00;
acadoVariables.lbValues[149] = -1.0000000000000000e+12;
acadoVariables.ubValues[0] = 1.0000000000000000e+00;
acadoVariables.ubValues[1] = 1.0000000000000000e+00;
acadoVariables.ubValues[2] = 1.0000000000000000e+12;
acadoVariables.ubValues[3] = 1.0000000000000000e+00;
acadoVariables.ubValues[4] = 1.0000000000000000e+00;
acadoVariables.ubValues[5] = 1.0000000000000000e+12;
acadoVariables.ubValues[6] = 1.0000000000000000e+00;
acadoVariables.ubValues[7] = 1.0000000000000000e+00;
acadoVariables.ubValues[8] = 1.0000000000000000e+12;
acadoVariables.ubValues[9] = 1.0000000000000000e+00;
acadoVariables.ubValues[10] = 1.0000000000000000e+00;
acadoVariables.ubValues[11] = 1.0000000000000000e+12;
acadoVariables.ubValues[12] = 1.0000000000000000e+00;
acadoVariables.ubValues[13] = 1.0000000000000000e+00;
acadoVariables.ubValues[14] = 1.0000000000000000e+12;
acadoVariables.ubValues[15] = 1.0000000000000000e+00;
acadoVariables.ubValues[16] = 1.0000000000000000e+00;
acadoVariables.ubValues[17] = 1.0000000000000000e+12;
acadoVariables.ubValues[18] = 1.0000000000000000e+00;
acadoVariables.ubValues[19] = 1.0000000000000000e+00;
acadoVariables.ubValues[20] = 1.0000000000000000e+12;
acadoVariables.ubValues[21] = 1.0000000000000000e+00;
acadoVariables.ubValues[22] = 1.0000000000000000e+00;
acadoVariables.ubValues[23] = 1.0000000000000000e+12;
acadoVariables.ubValues[24] = 1.0000000000000000e+00;
acadoVariables.ubValues[25] = 1.0000000000000000e+00;
acadoVariables.ubValues[26] = 1.0000000000000000e+12;
acadoVariables.ubValues[27] = 1.0000000000000000e+00;
acadoVariables.ubValues[28] = 1.0000000000000000e+00;
acadoVariables.ubValues[29] = 1.0000000000000000e+12;
acadoVariables.ubValues[30] = 1.0000000000000000e+00;
acadoVariables.ubValues[31] = 1.0000000000000000e+00;
acadoVariables.ubValues[32] = 1.0000000000000000e+12;
acadoVariables.ubValues[33] = 1.0000000000000000e+00;
acadoVariables.ubValues[34] = 1.0000000000000000e+00;
acadoVariables.ubValues[35] = 1.0000000000000000e+12;
acadoVariables.ubValues[36] = 1.0000000000000000e+00;
acadoVariables.ubValues[37] = 1.0000000000000000e+00;
acadoVariables.ubValues[38] = 1.0000000000000000e+12;
acadoVariables.ubValues[39] = 1.0000000000000000e+00;
acadoVariables.ubValues[40] = 1.0000000000000000e+00;
acadoVariables.ubValues[41] = 1.0000000000000000e+12;
acadoVariables.ubValues[42] = 1.0000000000000000e+00;
acadoVariables.ubValues[43] = 1.0000000000000000e+00;
acadoVariables.ubValues[44] = 1.0000000000000000e+12;
acadoVariables.ubValues[45] = 1.0000000000000000e+00;
acadoVariables.ubValues[46] = 1.0000000000000000e+00;
acadoVariables.ubValues[47] = 1.0000000000000000e+12;
acadoVariables.ubValues[48] = 1.0000000000000000e+00;
acadoVariables.ubValues[49] = 1.0000000000000000e+00;
acadoVariables.ubValues[50] = 1.0000000000000000e+12;
acadoVariables.ubValues[51] = 1.0000000000000000e+00;
acadoVariables.ubValues[52] = 1.0000000000000000e+00;
acadoVariables.ubValues[53] = 1.0000000000000000e+12;
acadoVariables.ubValues[54] = 1.0000000000000000e+00;
acadoVariables.ubValues[55] = 1.0000000000000000e+00;
acadoVariables.ubValues[56] = 1.0000000000000000e+12;
acadoVariables.ubValues[57] = 1.0000000000000000e+00;
acadoVariables.ubValues[58] = 1.0000000000000000e+00;
acadoVariables.ubValues[59] = 1.0000000000000000e+12;
acadoVariables.ubValues[60] = 1.0000000000000000e+00;
acadoVariables.ubValues[61] = 1.0000000000000000e+00;
acadoVariables.ubValues[62] = 1.0000000000000000e+12;
acadoVariables.ubValues[63] = 1.0000000000000000e+00;
acadoVariables.ubValues[64] = 1.0000000000000000e+00;
acadoVariables.ubValues[65] = 1.0000000000000000e+12;
acadoVariables.ubValues[66] = 1.0000000000000000e+00;
acadoVariables.ubValues[67] = 1.0000000000000000e+00;
acadoVariables.ubValues[68] = 1.0000000000000000e+12;
acadoVariables.ubValues[69] = 1.0000000000000000e+00;
acadoVariables.ubValues[70] = 1.0000000000000000e+00;
acadoVariables.ubValues[71] = 1.0000000000000000e+12;
acadoVariables.ubValues[72] = 1.0000000000000000e+00;
acadoVariables.ubValues[73] = 1.0000000000000000e+00;
acadoVariables.ubValues[74] = 1.0000000000000000e+12;
acadoVariables.ubValues[75] = 1.0000000000000000e+00;
acadoVariables.ubValues[76] = 1.0000000000000000e+00;
acadoVariables.ubValues[77] = 1.0000000000000000e+12;
acadoVariables.ubValues[78] = 1.0000000000000000e+00;
acadoVariables.ubValues[79] = 1.0000000000000000e+00;
acadoVariables.ubValues[80] = 1.0000000000000000e+12;
acadoVariables.ubValues[81] = 1.0000000000000000e+00;
acadoVariables.ubValues[82] = 1.0000000000000000e+00;
acadoVariables.ubValues[83] = 1.0000000000000000e+12;
acadoVariables.ubValues[84] = 1.0000000000000000e+00;
acadoVariables.ubValues[85] = 1.0000000000000000e+00;
acadoVariables.ubValues[86] = 1.0000000000000000e+12;
acadoVariables.ubValues[87] = 1.0000000000000000e+00;
acadoVariables.ubValues[88] = 1.0000000000000000e+00;
acadoVariables.ubValues[89] = 1.0000000000000000e+12;
acadoVariables.ubValues[90] = 1.0000000000000000e+00;
acadoVariables.ubValues[91] = 1.0000000000000000e+00;
acadoVariables.ubValues[92] = 1.0000000000000000e+12;
acadoVariables.ubValues[93] = 1.0000000000000000e+00;
acadoVariables.ubValues[94] = 1.0000000000000000e+00;
acadoVariables.ubValues[95] = 1.0000000000000000e+12;
acadoVariables.ubValues[96] = 1.0000000000000000e+00;
acadoVariables.ubValues[97] = 1.0000000000000000e+00;
acadoVariables.ubValues[98] = 1.0000000000000000e+12;
acadoVariables.ubValues[99] = 1.0000000000000000e+00;
acadoVariables.ubValues[100] = 1.0000000000000000e+00;
acadoVariables.ubValues[101] = 1.0000000000000000e+12;
acadoVariables.ubValues[102] = 1.0000000000000000e+00;
acadoVariables.ubValues[103] = 1.0000000000000000e+00;
acadoVariables.ubValues[104] = 1.0000000000000000e+12;
acadoVariables.ubValues[105] = 1.0000000000000000e+00;
acadoVariables.ubValues[106] = 1.0000000000000000e+00;
acadoVariables.ubValues[107] = 1.0000000000000000e+12;
acadoVariables.ubValues[108] = 1.0000000000000000e+00;
acadoVariables.ubValues[109] = 1.0000000000000000e+00;
acadoVariables.ubValues[110] = 1.0000000000000000e+12;
acadoVariables.ubValues[111] = 1.0000000000000000e+00;
acadoVariables.ubValues[112] = 1.0000000000000000e+00;
acadoVariables.ubValues[113] = 1.0000000000000000e+12;
acadoVariables.ubValues[114] = 1.0000000000000000e+00;
acadoVariables.ubValues[115] = 1.0000000000000000e+00;
acadoVariables.ubValues[116] = 1.0000000000000000e+12;
acadoVariables.ubValues[117] = 1.0000000000000000e+00;
acadoVariables.ubValues[118] = 1.0000000000000000e+00;
acadoVariables.ubValues[119] = 1.0000000000000000e+12;
acadoVariables.ubValues[120] = 1.0000000000000000e+00;
acadoVariables.ubValues[121] = 1.0000000000000000e+00;
acadoVariables.ubValues[122] = 1.0000000000000000e+12;
acadoVariables.ubValues[123] = 1.0000000000000000e+00;
acadoVariables.ubValues[124] = 1.0000000000000000e+00;
acadoVariables.ubValues[125] = 1.0000000000000000e+12;
acadoVariables.ubValues[126] = 1.0000000000000000e+00;
acadoVariables.ubValues[127] = 1.0000000000000000e+00;
acadoVariables.ubValues[128] = 1.0000000000000000e+12;
acadoVariables.ubValues[129] = 1.0000000000000000e+00;
acadoVariables.ubValues[130] = 1.0000000000000000e+00;
acadoVariables.ubValues[131] = 1.0000000000000000e+12;
acadoVariables.ubValues[132] = 1.0000000000000000e+00;
acadoVariables.ubValues[133] = 1.0000000000000000e+00;
acadoVariables.ubValues[134] = 1.0000000000000000e+12;
acadoVariables.ubValues[135] = 1.0000000000000000e+00;
acadoVariables.ubValues[136] = 1.0000000000000000e+00;
acadoVariables.ubValues[137] = 1.0000000000000000e+12;
acadoVariables.ubValues[138] = 1.0000000000000000e+00;
acadoVariables.ubValues[139] = 1.0000000000000000e+00;
acadoVariables.ubValues[140] = 1.0000000000000000e+12;
acadoVariables.ubValues[141] = 1.0000000000000000e+00;
acadoVariables.ubValues[142] = 1.0000000000000000e+00;
acadoVariables.ubValues[143] = 1.0000000000000000e+12;
acadoVariables.ubValues[144] = 1.0000000000000000e+00;
acadoVariables.ubValues[145] = 1.0000000000000000e+00;
acadoVariables.ubValues[146] = 1.0000000000000000e+12;
acadoVariables.ubValues[147] = 1.0000000000000000e+00;
acadoVariables.ubValues[148] = 1.0000000000000000e+00;
acadoVariables.ubValues[149] = 1.0000000000000000e+12;
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
for (index = 0; index < 50; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[64] = acadoVariables.u[index * 3];
acadoWorkspace.state[65] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[66] = acadoVariables.u[index * 3 + 2];
acadoWorkspace.state[67] = acadoVariables.od[index * 19];
acadoWorkspace.state[68] = acadoVariables.od[index * 19 + 1];
acadoWorkspace.state[69] = acadoVariables.od[index * 19 + 2];
acadoWorkspace.state[70] = acadoVariables.od[index * 19 + 3];
acadoWorkspace.state[71] = acadoVariables.od[index * 19 + 4];
acadoWorkspace.state[72] = acadoVariables.od[index * 19 + 5];
acadoWorkspace.state[73] = acadoVariables.od[index * 19 + 6];
acadoWorkspace.state[74] = acadoVariables.od[index * 19 + 7];
acadoWorkspace.state[75] = acadoVariables.od[index * 19 + 8];
acadoWorkspace.state[76] = acadoVariables.od[index * 19 + 9];
acadoWorkspace.state[77] = acadoVariables.od[index * 19 + 10];
acadoWorkspace.state[78] = acadoVariables.od[index * 19 + 11];
acadoWorkspace.state[79] = acadoVariables.od[index * 19 + 12];
acadoWorkspace.state[80] = acadoVariables.od[index * 19 + 13];
acadoWorkspace.state[81] = acadoVariables.od[index * 19 + 14];
acadoWorkspace.state[82] = acadoVariables.od[index * 19 + 15];
acadoWorkspace.state[83] = acadoVariables.od[index * 19 + 16];
acadoWorkspace.state[84] = acadoVariables.od[index * 19 + 17];
acadoWorkspace.state[85] = acadoVariables.od[index * 19 + 18];

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
for (index = 0; index < 50; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[200] = xEnd[0];
acadoVariables.x[201] = xEnd[1];
acadoVariables.x[202] = xEnd[2];
acadoVariables.x[203] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[200];
acadoWorkspace.state[1] = acadoVariables.x[201];
acadoWorkspace.state[2] = acadoVariables.x[202];
acadoWorkspace.state[3] = acadoVariables.x[203];
if (uEnd != 0)
{
acadoWorkspace.state[64] = uEnd[0];
acadoWorkspace.state[65] = uEnd[1];
acadoWorkspace.state[66] = uEnd[2];
}
else
{
acadoWorkspace.state[64] = acadoVariables.u[147];
acadoWorkspace.state[65] = acadoVariables.u[148];
acadoWorkspace.state[66] = acadoVariables.u[149];
}
acadoWorkspace.state[67] = acadoVariables.od[950];
acadoWorkspace.state[68] = acadoVariables.od[951];
acadoWorkspace.state[69] = acadoVariables.od[952];
acadoWorkspace.state[70] = acadoVariables.od[953];
acadoWorkspace.state[71] = acadoVariables.od[954];
acadoWorkspace.state[72] = acadoVariables.od[955];
acadoWorkspace.state[73] = acadoVariables.od[956];
acadoWorkspace.state[74] = acadoVariables.od[957];
acadoWorkspace.state[75] = acadoVariables.od[958];
acadoWorkspace.state[76] = acadoVariables.od[959];
acadoWorkspace.state[77] = acadoVariables.od[960];
acadoWorkspace.state[78] = acadoVariables.od[961];
acadoWorkspace.state[79] = acadoVariables.od[962];
acadoWorkspace.state[80] = acadoVariables.od[963];
acadoWorkspace.state[81] = acadoVariables.od[964];
acadoWorkspace.state[82] = acadoVariables.od[965];
acadoWorkspace.state[83] = acadoVariables.od[966];
acadoWorkspace.state[84] = acadoVariables.od[967];
acadoWorkspace.state[85] = acadoVariables.od[968];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[200] = acadoWorkspace.state[0];
acadoVariables.x[201] = acadoWorkspace.state[1];
acadoVariables.x[202] = acadoWorkspace.state[2];
acadoVariables.x[203] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 49; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[147] = uEnd[0];
acadoVariables.u[148] = uEnd[1];
acadoVariables.u[149] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119] + acadoWorkspace.g[120]*acadoWorkspace.x[120] + acadoWorkspace.g[121]*acadoWorkspace.x[121] + acadoWorkspace.g[122]*acadoWorkspace.x[122] + acadoWorkspace.g[123]*acadoWorkspace.x[123] + acadoWorkspace.g[124]*acadoWorkspace.x[124] + acadoWorkspace.g[125]*acadoWorkspace.x[125] + acadoWorkspace.g[126]*acadoWorkspace.x[126] + acadoWorkspace.g[127]*acadoWorkspace.x[127] + acadoWorkspace.g[128]*acadoWorkspace.x[128] + acadoWorkspace.g[129]*acadoWorkspace.x[129] + acadoWorkspace.g[130]*acadoWorkspace.x[130] + acadoWorkspace.g[131]*acadoWorkspace.x[131] + acadoWorkspace.g[132]*acadoWorkspace.x[132] + acadoWorkspace.g[133]*acadoWorkspace.x[133] + acadoWorkspace.g[134]*acadoWorkspace.x[134] + acadoWorkspace.g[135]*acadoWorkspace.x[135] + acadoWorkspace.g[136]*acadoWorkspace.x[136] + acadoWorkspace.g[137]*acadoWorkspace.x[137] + acadoWorkspace.g[138]*acadoWorkspace.x[138] + acadoWorkspace.g[139]*acadoWorkspace.x[139] + acadoWorkspace.g[140]*acadoWorkspace.x[140] + acadoWorkspace.g[141]*acadoWorkspace.x[141] + acadoWorkspace.g[142]*acadoWorkspace.x[142] + acadoWorkspace.g[143]*acadoWorkspace.x[143] + acadoWorkspace.g[144]*acadoWorkspace.x[144] + acadoWorkspace.g[145]*acadoWorkspace.x[145] + acadoWorkspace.g[146]*acadoWorkspace.x[146] + acadoWorkspace.g[147]*acadoWorkspace.x[147] + acadoWorkspace.g[148]*acadoWorkspace.x[148] + acadoWorkspace.g[149]*acadoWorkspace.x[149];
kkt = fabs( kkt );
for (index = 0; index < 150; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 50; ++index)
{
prd = acadoWorkspace.y[index + 150];
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
for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 19];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 19 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 19 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 19 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 19 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 19 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 19 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 19 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 19 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 19 + 9];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 19 + 10];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 19 + 11];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 19 + 12];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 19 + 13];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 19 + 14];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 19 + 15];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 19 + 16];
acadoWorkspace.objValueIn[24] = acadoVariables.od[lRun1 * 19 + 17];
acadoWorkspace.objValueIn[25] = acadoVariables.od[lRun1 * 19 + 18];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[200];
acadoWorkspace.objValueIn[1] = acadoVariables.x[201];
acadoWorkspace.objValueIn[2] = acadoVariables.x[202];
acadoWorkspace.objValueIn[3] = acadoVariables.x[203];
acadoWorkspace.objValueIn[4] = acadoVariables.od[950];
acadoWorkspace.objValueIn[5] = acadoVariables.od[951];
acadoWorkspace.objValueIn[6] = acadoVariables.od[952];
acadoWorkspace.objValueIn[7] = acadoVariables.od[953];
acadoWorkspace.objValueIn[8] = acadoVariables.od[954];
acadoWorkspace.objValueIn[9] = acadoVariables.od[955];
acadoWorkspace.objValueIn[10] = acadoVariables.od[956];
acadoWorkspace.objValueIn[11] = acadoVariables.od[957];
acadoWorkspace.objValueIn[12] = acadoVariables.od[958];
acadoWorkspace.objValueIn[13] = acadoVariables.od[959];
acadoWorkspace.objValueIn[14] = acadoVariables.od[960];
acadoWorkspace.objValueIn[15] = acadoVariables.od[961];
acadoWorkspace.objValueIn[16] = acadoVariables.od[962];
acadoWorkspace.objValueIn[17] = acadoVariables.od[963];
acadoWorkspace.objValueIn[18] = acadoVariables.od[964];
acadoWorkspace.objValueIn[19] = acadoVariables.od[965];
acadoWorkspace.objValueIn[20] = acadoVariables.od[966];
acadoWorkspace.objValueIn[21] = acadoVariables.od[967];
acadoWorkspace.objValueIn[22] = acadoVariables.od[968];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
return objVal;
}

