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
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[3] = acadoVariables.mu[lRun1 * 3];
acadoWorkspace.state[4] = acadoVariables.mu[lRun1 * 3 + 1];
acadoWorkspace.state[5] = acadoVariables.mu[lRun1 * 3 + 2];
acadoWorkspace.state[36] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[37] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[38] = acadoVariables.od[lRun1 * 19];
acadoWorkspace.state[39] = acadoVariables.od[lRun1 * 19 + 1];
acadoWorkspace.state[40] = acadoVariables.od[lRun1 * 19 + 2];
acadoWorkspace.state[41] = acadoVariables.od[lRun1 * 19 + 3];
acadoWorkspace.state[42] = acadoVariables.od[lRun1 * 19 + 4];
acadoWorkspace.state[43] = acadoVariables.od[lRun1 * 19 + 5];
acadoWorkspace.state[44] = acadoVariables.od[lRun1 * 19 + 6];
acadoWorkspace.state[45] = acadoVariables.od[lRun1 * 19 + 7];
acadoWorkspace.state[46] = acadoVariables.od[lRun1 * 19 + 8];
acadoWorkspace.state[47] = acadoVariables.od[lRun1 * 19 + 9];
acadoWorkspace.state[48] = acadoVariables.od[lRun1 * 19 + 10];
acadoWorkspace.state[49] = acadoVariables.od[lRun1 * 19 + 11];
acadoWorkspace.state[50] = acadoVariables.od[lRun1 * 19 + 12];
acadoWorkspace.state[51] = acadoVariables.od[lRun1 * 19 + 13];
acadoWorkspace.state[52] = acadoVariables.od[lRun1 * 19 + 14];
acadoWorkspace.state[53] = acadoVariables.od[lRun1 * 19 + 15];
acadoWorkspace.state[54] = acadoVariables.od[lRun1 * 19 + 16];
acadoWorkspace.state[55] = acadoVariables.od[lRun1 * 19 + 17];
acadoWorkspace.state[56] = acadoVariables.od[lRun1 * 19 + 18];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 3] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 3 + 3];
acadoWorkspace.d[lRun1 * 3 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 3 + 4];
acadoWorkspace.d[lRun1 * 3 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 3 + 5];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[14];

acadoWorkspace.evGu[lRun1 * 6] = acadoWorkspace.state[15];
acadoWorkspace.evGu[lRun1 * 6 + 1] = acadoWorkspace.state[16];
acadoWorkspace.evGu[lRun1 * 6 + 2] = acadoWorkspace.state[17];
acadoWorkspace.evGu[lRun1 * 6 + 3] = acadoWorkspace.state[18];
acadoWorkspace.evGu[lRun1 * 6 + 4] = acadoWorkspace.state[19];
acadoWorkspace.evGu[lRun1 * 6 + 5] = acadoWorkspace.state[20];
acadoWorkspace.EH[lRun1 * 25] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[21];
acadoWorkspace.EH[lRun1 * 25 + 5] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[22];
acadoWorkspace.EH[lRun1 * 25 + 1] = acadoWorkspace.EH[lRun1 * 25 + 5];
acadoWorkspace.EH[lRun1 * 25 + 6] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[23];
acadoWorkspace.EH[lRun1 * 25 + 10] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[24];
acadoWorkspace.EH[lRun1 * 25 + 2] = acadoWorkspace.EH[lRun1 * 25 + 10];
acadoWorkspace.EH[lRun1 * 25 + 11] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[25];
acadoWorkspace.EH[lRun1 * 25 + 7] = acadoWorkspace.EH[lRun1 * 25 + 11];
acadoWorkspace.EH[lRun1 * 25 + 12] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[26];
acadoWorkspace.EH[lRun1 * 25 + 15] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[27];
acadoWorkspace.EH[lRun1 * 25 + 3] = acadoWorkspace.EH[lRun1 * 25 + 15];
acadoWorkspace.EH[lRun1 * 25 + 16] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[28];
acadoWorkspace.EH[lRun1 * 25 + 8] = acadoWorkspace.EH[lRun1 * 25 + 16];
acadoWorkspace.EH[lRun1 * 25 + 17] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[29];
acadoWorkspace.EH[lRun1 * 25 + 13] = acadoWorkspace.EH[lRun1 * 25 + 17];
acadoWorkspace.EH[lRun1 * 25 + 18] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[30];
acadoWorkspace.EH[lRun1 * 25 + 20] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[31];
acadoWorkspace.EH[lRun1 * 25 + 4] = acadoWorkspace.EH[lRun1 * 25 + 20];
acadoWorkspace.EH[lRun1 * 25 + 21] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[32];
acadoWorkspace.EH[lRun1 * 25 + 9] = acadoWorkspace.EH[lRun1 * 25 + 21];
acadoWorkspace.EH[lRun1 * 25 + 22] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[33];
acadoWorkspace.EH[lRun1 * 25 + 14] = acadoWorkspace.EH[lRun1 * 25 + 22];
acadoWorkspace.EH[lRun1 * 25 + 23] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[34];
acadoWorkspace.EH[lRun1 * 25 + 19] = acadoWorkspace.EH[lRun1 * 25 + 23];
acadoWorkspace.EH[lRun1 * 25 + 24] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[35];
}
return ret;
}

void acado_evaluateLagrange(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 5;
/* Vector of auxiliary variables; number of elements: 33. */
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
a[12] = (od[6]*u[0]);
a[13] = (od[6]*u[0]);
a[14] = (a[12]+a[13]);
a[15] = (od[7]*u[1]);
a[16] = (od[7]*u[1]);
a[17] = (a[15]+a[16]);
a[18] = (od[3]*(real_t)(2.0000000000000000e+00));
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (od[4]*(real_t)(2.0000000000000000e+00));
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (od[5]*(real_t)(2.0000000000000000e+00));
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(0.0000000000000000e+00);
a[30] = (od[6]*(real_t)(2.0000000000000000e+00));
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (od[7]*(real_t)(2.0000000000000000e+00));

/* Compute outputs: */
out[0] = ((((((od[3]*(xd[0]-od[0]))*(xd[0]-od[0]))+((od[4]*(xd[1]-od[1]))*(xd[1]-od[1])))+((od[5]*(xd[2]-od[2]))*(xd[2]-od[2])))+((od[6]*u[0])*u[0]))+((od[7]*u[1])*u[1]));
out[1] = a[3];
out[2] = a[7];
out[3] = a[11];
out[4] = a[14];
out[5] = a[17];
out[6] = a[18];
out[7] = a[19];
out[8] = a[20];
out[9] = a[19];
out[10] = a[21];
out[11] = a[22];
out[12] = a[20];
out[13] = a[22];
out[14] = a[23];
out[15] = a[24];
out[16] = a[25];
out[17] = a[26];
out[18] = a[27];
out[19] = a[28];
out[20] = a[29];
out[21] = a[30];
out[22] = a[31];
out[23] = a[31];
out[24] = a[32];
}

void acado_evaluateMayer(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 3;
/* Vector of auxiliary variables; number of elements: 18. */
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
a[12] = (od[8]*(real_t)(2.0000000000000000e+00));
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (od[9]*(real_t)(2.0000000000000000e+00));
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (od[10]*(real_t)(2.0000000000000000e+00));

/* Compute outputs: */
out[0] = ((((od[8]*(xd[0]-od[0]))*(xd[0]-od[0]))+((od[9]*(xd[1]-od[1]))*(xd[1]-od[1])))+((od[10]*(xd[2]-od[2]))*(xd[2]-od[2])));
out[1] = a[3];
out[2] = a[7];
out[3] = a[11];
out[4] = a[12];
out[5] = a[13];
out[6] = a[14];
out[7] = a[13];
out[8] = a[15];
out[9] = a[16];
out[10] = a[14];
out[11] = a[16];
out[12] = a[17];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 66. */
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
a[48] = (a[25]*(real_t)(2.0000000000000000e+00));
a[49] = (a[48]*xd[3]);
a[50] = (a[37]+a[30]);
a[51] = (a[50]*xd[3]);
a[52] = (real_t)(0.0000000000000000e+00);
a[53] = (real_t)(0.0000000000000000e+00);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = (a[41]*(real_t)(2.0000000000000000e+00));
a[56] = (a[55]*xd[3]);
a[57] = (real_t)(0.0000000000000000e+00);
a[58] = (real_t)(0.0000000000000000e+00);
a[59] = (real_t)(0.0000000000000000e+00);
a[60] = (real_t)(0.0000000000000000e+00);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[20];
out[1] = a[32];
out[2] = a[44];
out[3] = a[45];
out[4] = a[46];
out[5] = a[47];
out[6] = a[49];
out[7] = a[51];
out[8] = a[52];
out[9] = a[53];
out[10] = a[54];
out[11] = a[51];
out[12] = a[56];
out[13] = a[57];
out[14] = a[58];
out[15] = a[59];
out[16] = a[52];
out[17] = a[57];
out[18] = a[60];
out[19] = a[61];
out[20] = a[62];
out[21] = a[53];
out[22] = a[58];
out[23] = a[61];
out[24] = a[63];
out[25] = a[64];
out[26] = a[54];
out[27] = a[59];
out[28] = a[62];
out[29] = a[64];
out[30] = a[65];
}

void acado_addObjTerm( real_t* const tmpFxx, real_t* const tmpFxu, real_t* const tmpFuu, real_t* const tmpEH )
{
tmpEH[0] += tmpFxx[0];
tmpEH[1] += tmpFxx[1];
tmpEH[2] += tmpFxx[2];
tmpEH[5] += tmpFxx[3];
tmpEH[6] += tmpFxx[4];
tmpEH[7] += tmpFxx[5];
tmpEH[10] += tmpFxx[6];
tmpEH[11] += tmpFxx[7];
tmpEH[12] += tmpFxx[8];
tmpEH[3] += tmpFxu[0];
tmpEH[4] += tmpFxu[1];
tmpEH[8] += tmpFxu[2];
tmpEH[9] += tmpFxu[3];
tmpEH[13] += tmpFxu[4];
tmpEH[14] += tmpFxu[5];
tmpEH[15] += tmpFxu[0];
tmpEH[16] += tmpFxu[2];
tmpEH[17] += tmpFxu[4];
tmpEH[20] += tmpFxu[1];
tmpEH[21] += tmpFxu[3];
tmpEH[22] += tmpFxu[5];
tmpEH[18] += tmpFuu[0];
tmpEH[19] += tmpFuu[1];
tmpEH[23] += tmpFuu[2];
tmpEH[24] += tmpFuu[3];
}

void acado_addObjLinearTerm( real_t* const tmpDx, real_t* const tmpDu, real_t* const tmpDF )
{
tmpDx[0] = tmpDF[0];
tmpDx[1] = tmpDF[1];
tmpDx[2] = tmpDF[2];
tmpDu[0] = tmpDF[3];
tmpDu[1] = tmpDF[4];
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
}

void acado_evaluateObjective(  )
{
int lRun2;
int runObj;
for (runObj = 0; runObj < 25; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 19];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 19 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 19 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 19 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 19 + 4];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 19 + 5];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 19 + 6];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 19 + 7];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 19 + 8];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 19 + 9];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 19 + 10];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 19 + 11];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 19 + 12];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 19 + 13];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 19 + 14];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 19 + 15];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 19 + 16];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 19 + 17];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 19 + 18];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjTerm( &(acadoWorkspace.objValueOut[ 6 ]), &(acadoWorkspace.objValueOut[ 15 ]), &(acadoWorkspace.objValueOut[ 21 ]), &(acadoWorkspace.EH[ runObj * 25 ]) );
acado_addObjLinearTerm( &(acadoWorkspace.QDy[ runObj * 3 ]), &(acadoWorkspace.g[ runObj * 2 ]), &(acadoWorkspace.objValueOut[ 1 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[75];
acadoWorkspace.objValueIn[1] = acadoVariables.x[76];
acadoWorkspace.objValueIn[2] = acadoVariables.x[77];
acadoWorkspace.objValueIn[3] = acadoVariables.od[475];
acadoWorkspace.objValueIn[4] = acadoVariables.od[476];
acadoWorkspace.objValueIn[5] = acadoVariables.od[477];
acadoWorkspace.objValueIn[6] = acadoVariables.od[478];
acadoWorkspace.objValueIn[7] = acadoVariables.od[479];
acadoWorkspace.objValueIn[8] = acadoVariables.od[480];
acadoWorkspace.objValueIn[9] = acadoVariables.od[481];
acadoWorkspace.objValueIn[10] = acadoVariables.od[482];
acadoWorkspace.objValueIn[11] = acadoVariables.od[483];
acadoWorkspace.objValueIn[12] = acadoVariables.od[484];
acadoWorkspace.objValueIn[13] = acadoVariables.od[485];
acadoWorkspace.objValueIn[14] = acadoVariables.od[486];
acadoWorkspace.objValueIn[15] = acadoVariables.od[487];
acadoWorkspace.objValueIn[16] = acadoVariables.od[488];
acadoWorkspace.objValueIn[17] = acadoVariables.od[489];
acadoWorkspace.objValueIn[18] = acadoVariables.od[490];
acadoWorkspace.objValueIn[19] = acadoVariables.od[491];
acadoWorkspace.objValueIn[20] = acadoVariables.od[492];
acadoWorkspace.objValueIn[21] = acadoVariables.od[493];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjEndTerm( &(acadoWorkspace.objValueOut[ 4 ]), acadoWorkspace.EH_N );
acadoWorkspace.QDy[75] = acadoWorkspace.objValueOut[1];
acadoWorkspace.QDy[76] = acadoWorkspace.objValueOut[2];
acadoWorkspace.QDy[77] = acadoWorkspace.objValueOut[3];

for (lRun2 = 0; lRun2 < 25; ++lRun2)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun2 * 3];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun2 * 3 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun2 * 3 + 2];
acadoWorkspace.conValueIn[3] = acadoWorkspace.y[lRun2 + 50];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun2 * 2];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun2 * 2 + 1];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun2 * 19];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun2 * 19 + 1];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun2 * 19 + 2];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun2 * 19 + 3];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun2 * 19 + 4];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun2 * 19 + 5];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun2 * 19 + 6];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun2 * 19 + 7];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun2 * 19 + 8];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun2 * 19 + 9];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun2 * 19 + 10];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun2 * 19 + 11];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun2 * 19 + 12];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun2 * 19 + 13];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun2 * 19 + 14];
acadoWorkspace.conValueIn[21] = acadoVariables.od[lRun2 * 19 + 15];
acadoWorkspace.conValueIn[22] = acadoVariables.od[lRun2 * 19 + 16];
acadoWorkspace.conValueIn[23] = acadoVariables.od[lRun2 * 19 + 17];
acadoWorkspace.conValueIn[24] = acadoVariables.od[lRun2 * 19 + 18];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun2] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun2 * 3] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun2 * 3 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun2 * 3 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHu[lRun2 * 2] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHu[lRun2 * 2 + 1] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evDDH[0] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evDDH[1] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evDDH[2] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evDDH[3] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evDDH[4] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evDDH[5] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evDDH[6] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evDDH[7] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evDDH[8] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evDDH[9] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evDDH[10] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evDDH[11] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evDDH[12] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evDDH[13] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evDDH[14] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evDDH[15] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evDDH[16] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evDDH[17] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evDDH[18] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evDDH[19] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evDDH[20] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evDDH[21] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evDDH[22] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evDDH[23] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evDDH[24] = acadoWorkspace.conValueOut[30];
acadoWorkspace.EH[lRun2 * 25] += acadoWorkspace.evDDH[0];
acadoWorkspace.EH[lRun2 * 25 + 1] += acadoWorkspace.evDDH[1];
acadoWorkspace.EH[lRun2 * 25 + 2] += acadoWorkspace.evDDH[2];
acadoWorkspace.EH[lRun2 * 25 + 3] += acadoWorkspace.evDDH[3];
acadoWorkspace.EH[lRun2 * 25 + 4] += acadoWorkspace.evDDH[4];
acadoWorkspace.EH[lRun2 * 25 + 5] += acadoWorkspace.evDDH[5];
acadoWorkspace.EH[lRun2 * 25 + 6] += acadoWorkspace.evDDH[6];
acadoWorkspace.EH[lRun2 * 25 + 7] += acadoWorkspace.evDDH[7];
acadoWorkspace.EH[lRun2 * 25 + 8] += acadoWorkspace.evDDH[8];
acadoWorkspace.EH[lRun2 * 25 + 9] += acadoWorkspace.evDDH[9];
acadoWorkspace.EH[lRun2 * 25 + 10] += acadoWorkspace.evDDH[10];
acadoWorkspace.EH[lRun2 * 25 + 11] += acadoWorkspace.evDDH[11];
acadoWorkspace.EH[lRun2 * 25 + 12] += acadoWorkspace.evDDH[12];
acadoWorkspace.EH[lRun2 * 25 + 13] += acadoWorkspace.evDDH[13];
acadoWorkspace.EH[lRun2 * 25 + 14] += acadoWorkspace.evDDH[14];
acadoWorkspace.EH[lRun2 * 25 + 15] += acadoWorkspace.evDDH[15];
acadoWorkspace.EH[lRun2 * 25 + 16] += acadoWorkspace.evDDH[16];
acadoWorkspace.EH[lRun2 * 25 + 17] += acadoWorkspace.evDDH[17];
acadoWorkspace.EH[lRun2 * 25 + 18] += acadoWorkspace.evDDH[18];
acadoWorkspace.EH[lRun2 * 25 + 19] += acadoWorkspace.evDDH[19];
acadoWorkspace.EH[lRun2 * 25 + 20] += acadoWorkspace.evDDH[20];
acadoWorkspace.EH[lRun2 * 25 + 21] += acadoWorkspace.evDDH[21];
acadoWorkspace.EH[lRun2 * 25 + 22] += acadoWorkspace.evDDH[22];
acadoWorkspace.EH[lRun2 * 25 + 23] += acadoWorkspace.evDDH[23];
acadoWorkspace.EH[lRun2 * 25 + 24] += acadoWorkspace.evDDH[24];
}

}

void acado_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
acado_regularize( &(acadoWorkspace.EH[ lRun1 * 25 ]) );
acadoWorkspace.Q1[lRun1 * 9] = acadoWorkspace.EH[lRun1 * 25];
acadoWorkspace.Q1[lRun1 * 9 + 1] = acadoWorkspace.EH[lRun1 * 25 + 1];
acadoWorkspace.Q1[lRun1 * 9 + 2] = acadoWorkspace.EH[lRun1 * 25 + 2];
acadoWorkspace.Q1[lRun1 * 9 + 3] = acadoWorkspace.EH[lRun1 * 25 + 5];
acadoWorkspace.Q1[lRun1 * 9 + 4] = acadoWorkspace.EH[lRun1 * 25 + 6];
acadoWorkspace.Q1[lRun1 * 9 + 5] = acadoWorkspace.EH[lRun1 * 25 + 7];
acadoWorkspace.Q1[lRun1 * 9 + 6] = acadoWorkspace.EH[lRun1 * 25 + 10];
acadoWorkspace.Q1[lRun1 * 9 + 7] = acadoWorkspace.EH[lRun1 * 25 + 11];
acadoWorkspace.Q1[lRun1 * 9 + 8] = acadoWorkspace.EH[lRun1 * 25 + 12];
acadoWorkspace.S1[lRun1 * 6] = acadoWorkspace.EH[lRun1 * 25 + 3];
acadoWorkspace.S1[lRun1 * 6 + 1] = acadoWorkspace.EH[lRun1 * 25 + 4];
acadoWorkspace.S1[lRun1 * 6 + 2] = acadoWorkspace.EH[lRun1 * 25 + 8];
acadoWorkspace.S1[lRun1 * 6 + 3] = acadoWorkspace.EH[lRun1 * 25 + 9];
acadoWorkspace.S1[lRun1 * 6 + 4] = acadoWorkspace.EH[lRun1 * 25 + 13];
acadoWorkspace.S1[lRun1 * 6 + 5] = acadoWorkspace.EH[lRun1 * 25 + 14];
acadoWorkspace.R1[lRun1 * 4] = acadoWorkspace.EH[lRun1 * 25 + 18];
acadoWorkspace.R1[lRun1 * 4 + 1] = acadoWorkspace.EH[lRun1 * 25 + 19];
acadoWorkspace.R1[lRun1 * 4 + 2] = acadoWorkspace.EH[lRun1 * 25 + 23];
acadoWorkspace.R1[lRun1 * 4 + 3] = acadoWorkspace.EH[lRun1 * 25 + 24];
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
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[3] + Gx1[2]*Gx2[6];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[7];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[8];
Gx3[3] = + Gx1[3]*Gx2[0] + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[6];
Gx3[4] = + Gx1[3]*Gx2[1] + Gx1[4]*Gx2[4] + Gx1[5]*Gx2[7];
Gx3[5] = + Gx1[3]*Gx2[2] + Gx1[4]*Gx2[5] + Gx1[5]*Gx2[8];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[6];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[7];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[8];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5];
Gu2[2] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[5]*Gu1[4];
Gu2[3] = + Gx1[3]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[5]*Gu1[5];
Gu2[4] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[4];
Gu2[5] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[5];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 102] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + R11[0];
acadoWorkspace.H[iRow * 102 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + R11[1];
acadoWorkspace.H[iRow * 102 + 50] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + R11[2];
acadoWorkspace.H[iRow * 102 + 51] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + R11[3];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[3]*Gu1[2] + Gx1[6]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[3]*Gu1[3] + Gx1[6]*Gu1[5];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[7]*Gu1[4];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[7]*Gu1[5];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[8]*Gu1[4];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[8]*Gu1[5];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Gu2[1];
Gu3[2] = + Q11[3]*Gu1[0] + Q11[4]*Gu1[2] + Q11[5]*Gu1[4] + Gu2[2];
Gu3[3] = + Q11[3]*Gu1[1] + Q11[4]*Gu1[3] + Q11[5]*Gu1[5] + Gu2[3];
Gu3[4] = + Q11[6]*Gu1[0] + Q11[7]*Gu1[2] + Q11[8]*Gu1[4] + Gu2[4];
Gu3[5] = + Q11[6]*Gu1[1] + Q11[7]*Gu1[3] + Q11[8]*Gu1[5] + Gu2[5];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[3]*w11[1] + Gx1[6]*w11[2] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[4]*w11[1] + Gx1[7]*w11[2] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[5]*w11[1] + Gx1[8]*w11[2] + w12[2];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + w12[0];
w13[1] = + Q11[3]*w11[0] + Q11[4]*w11[1] + Q11[5]*w11[2] + w12[1];
w13[2] = + Q11[6]*w11[0] + Q11[7]*w11[1] + Q11[8]*w11[2] + w12[2];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
}

void acado_expansionStep2( real_t* const QDy1, real_t* const Q11, real_t* const w11, real_t* const Gu1, real_t* const U1, real_t* const Gx1, real_t* const mu1, real_t* const mu2 )
{
mu1[0] += QDy1[0];
mu1[1] += QDy1[1];
mu1[2] += QDy1[2];
mu1[0] += + w11[0]*Q11[0] + w11[1]*Q11[1] + w11[2]*Q11[2];
mu1[1] += + w11[0]*Q11[3] + w11[1]*Q11[4] + w11[2]*Q11[5];
mu1[2] += + w11[0]*Q11[6] + w11[1]*Q11[7] + w11[2]*Q11[8];
mu1[0] += + U1[0]*Gu1[0] + U1[1]*Gu1[1];
mu1[1] += + U1[0]*Gu1[2] + U1[1]*Gu1[3];
mu1[2] += + U1[0]*Gu1[4] + U1[1]*Gu1[5];
mu1[0] += + mu2[0]*Gx1[0] + mu2[1]*Gx1[3] + mu2[2]*Gx1[6];
mu1[1] += + mu2[0]*Gx1[1] + mu2[1]*Gx1[4] + mu2[2]*Gx1[7];
mu1[2] += + mu2[0]*Gx1[2] + mu2[1]*Gx1[5] + mu2[2]*Gx1[8];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol * 2)] = acadoWorkspace.H[(iCol * 100) + (iRow * 2)];
acadoWorkspace.H[(iRow * 100) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 100 + 50) + (iRow * 2)];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2)] = acadoWorkspace.H[(iCol * 100) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 100 + 50) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 100 + 50) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const RDy1 )
{
}

void acado_multQDy( real_t* const QDy1 )
{
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 50) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4];
acadoWorkspace.A[(row * 50) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2];
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
acado_multGxGx( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 9 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.C[ 9 ]), &(acadoWorkspace.C[ 18 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.C[ 18 ]), &(acadoWorkspace.C[ 27 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.C[ 27 ]), &(acadoWorkspace.C[ 36 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.C[ 36 ]), &(acadoWorkspace.C[ 45 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.C[ 45 ]), &(acadoWorkspace.C[ 54 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.C[ 54 ]), &(acadoWorkspace.C[ 63 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.C[ 63 ]), &(acadoWorkspace.C[ 72 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.C[ 72 ]), &(acadoWorkspace.C[ 81 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.C[ 81 ]), &(acadoWorkspace.C[ 90 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.C[ 90 ]), &(acadoWorkspace.C[ 99 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.C[ 99 ]), &(acadoWorkspace.C[ 108 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.C[ 108 ]), &(acadoWorkspace.C[ 117 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.C[ 117 ]), &(acadoWorkspace.C[ 126 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.C[ 126 ]), &(acadoWorkspace.C[ 135 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.C[ 135 ]), &(acadoWorkspace.C[ 144 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.C[ 144 ]), &(acadoWorkspace.C[ 153 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.C[ 153 ]), &(acadoWorkspace.C[ 162 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.C[ 162 ]), &(acadoWorkspace.C[ 171 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.C[ 171 ]), &(acadoWorkspace.C[ 180 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.C[ 180 ]), &(acadoWorkspace.C[ 189 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.C[ 189 ]), &(acadoWorkspace.C[ 198 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.C[ 198 ]), &(acadoWorkspace.C[ 207 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.C[ 207 ]), &(acadoWorkspace.C[ 216 ]) );
for (lRun2 = 0; lRun2 < 25; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 51)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 6 ]), &(acadoWorkspace.E[ lRun3 * 6 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 25; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (3)) * (3)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (3)) * (2)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (3)) * (2)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (25)) - (1)) * (3)) * (2)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 24; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 6 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 6 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (2)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 9 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (2)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 4 ]), &(acadoWorkspace.evGu[ lRun2 * 6 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 25; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

acadoWorkspace.sbar[3] = acadoWorkspace.d[0];
acadoWorkspace.sbar[4] = acadoWorkspace.d[1];
acadoWorkspace.sbar[5] = acadoWorkspace.d[2];
acadoWorkspace.sbar[6] = acadoWorkspace.d[3];
acadoWorkspace.sbar[7] = acadoWorkspace.d[4];
acadoWorkspace.sbar[8] = acadoWorkspace.d[5];
acadoWorkspace.sbar[9] = acadoWorkspace.d[6];
acadoWorkspace.sbar[10] = acadoWorkspace.d[7];
acadoWorkspace.sbar[11] = acadoWorkspace.d[8];
acadoWorkspace.sbar[12] = acadoWorkspace.d[9];
acadoWorkspace.sbar[13] = acadoWorkspace.d[10];
acadoWorkspace.sbar[14] = acadoWorkspace.d[11];
acadoWorkspace.sbar[15] = acadoWorkspace.d[12];
acadoWorkspace.sbar[16] = acadoWorkspace.d[13];
acadoWorkspace.sbar[17] = acadoWorkspace.d[14];
acadoWorkspace.sbar[18] = acadoWorkspace.d[15];
acadoWorkspace.sbar[19] = acadoWorkspace.d[16];
acadoWorkspace.sbar[20] = acadoWorkspace.d[17];
acadoWorkspace.sbar[21] = acadoWorkspace.d[18];
acadoWorkspace.sbar[22] = acadoWorkspace.d[19];
acadoWorkspace.sbar[23] = acadoWorkspace.d[20];
acadoWorkspace.sbar[24] = acadoWorkspace.d[21];
acadoWorkspace.sbar[25] = acadoWorkspace.d[22];
acadoWorkspace.sbar[26] = acadoWorkspace.d[23];
acadoWorkspace.sbar[27] = acadoWorkspace.d[24];
acadoWorkspace.sbar[28] = acadoWorkspace.d[25];
acadoWorkspace.sbar[29] = acadoWorkspace.d[26];
acadoWorkspace.sbar[30] = acadoWorkspace.d[27];
acadoWorkspace.sbar[31] = acadoWorkspace.d[28];
acadoWorkspace.sbar[32] = acadoWorkspace.d[29];
acadoWorkspace.sbar[33] = acadoWorkspace.d[30];
acadoWorkspace.sbar[34] = acadoWorkspace.d[31];
acadoWorkspace.sbar[35] = acadoWorkspace.d[32];
acadoWorkspace.sbar[36] = acadoWorkspace.d[33];
acadoWorkspace.sbar[37] = acadoWorkspace.d[34];
acadoWorkspace.sbar[38] = acadoWorkspace.d[35];
acadoWorkspace.sbar[39] = acadoWorkspace.d[36];
acadoWorkspace.sbar[40] = acadoWorkspace.d[37];
acadoWorkspace.sbar[41] = acadoWorkspace.d[38];
acadoWorkspace.sbar[42] = acadoWorkspace.d[39];
acadoWorkspace.sbar[43] = acadoWorkspace.d[40];
acadoWorkspace.sbar[44] = acadoWorkspace.d[41];
acadoWorkspace.sbar[45] = acadoWorkspace.d[42];
acadoWorkspace.sbar[46] = acadoWorkspace.d[43];
acadoWorkspace.sbar[47] = acadoWorkspace.d[44];
acadoWorkspace.sbar[48] = acadoWorkspace.d[45];
acadoWorkspace.sbar[49] = acadoWorkspace.d[46];
acadoWorkspace.sbar[50] = acadoWorkspace.d[47];
acadoWorkspace.sbar[51] = acadoWorkspace.d[48];
acadoWorkspace.sbar[52] = acadoWorkspace.d[49];
acadoWorkspace.sbar[53] = acadoWorkspace.d[50];
acadoWorkspace.sbar[54] = acadoWorkspace.d[51];
acadoWorkspace.sbar[55] = acadoWorkspace.d[52];
acadoWorkspace.sbar[56] = acadoWorkspace.d[53];
acadoWorkspace.sbar[57] = acadoWorkspace.d[54];
acadoWorkspace.sbar[58] = acadoWorkspace.d[55];
acadoWorkspace.sbar[59] = acadoWorkspace.d[56];
acadoWorkspace.sbar[60] = acadoWorkspace.d[57];
acadoWorkspace.sbar[61] = acadoWorkspace.d[58];
acadoWorkspace.sbar[62] = acadoWorkspace.d[59];
acadoWorkspace.sbar[63] = acadoWorkspace.d[60];
acadoWorkspace.sbar[64] = acadoWorkspace.d[61];
acadoWorkspace.sbar[65] = acadoWorkspace.d[62];
acadoWorkspace.sbar[66] = acadoWorkspace.d[63];
acadoWorkspace.sbar[67] = acadoWorkspace.d[64];
acadoWorkspace.sbar[68] = acadoWorkspace.d[65];
acadoWorkspace.sbar[69] = acadoWorkspace.d[66];
acadoWorkspace.sbar[70] = acadoWorkspace.d[67];
acadoWorkspace.sbar[71] = acadoWorkspace.d[68];
acadoWorkspace.sbar[72] = acadoWorkspace.d[69];
acadoWorkspace.sbar[73] = acadoWorkspace.d[70];
acadoWorkspace.sbar[74] = acadoWorkspace.d[71];
acadoWorkspace.sbar[75] = acadoWorkspace.d[72];
acadoWorkspace.sbar[76] = acadoWorkspace.d[73];
acadoWorkspace.sbar[77] = acadoWorkspace.d[74];



for (lRun1 = 0; lRun1 < 24; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun2) * (lRun2 * -1 + 49)) / (2)) + (lRun1);
lRun4 = lRun1 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun1 * 3 + 3 ]), &(acadoWorkspace.E[ lRun3 * 6 ]), lRun4, lRun2 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[52] = acadoWorkspace.evHu[2];
acadoWorkspace.A[53] = acadoWorkspace.evHu[3];
acadoWorkspace.A[104] = acadoWorkspace.evHu[4];
acadoWorkspace.A[105] = acadoWorkspace.evHu[5];
acadoWorkspace.A[156] = acadoWorkspace.evHu[6];
acadoWorkspace.A[157] = acadoWorkspace.evHu[7];
acadoWorkspace.A[208] = acadoWorkspace.evHu[8];
acadoWorkspace.A[209] = acadoWorkspace.evHu[9];
acadoWorkspace.A[260] = acadoWorkspace.evHu[10];
acadoWorkspace.A[261] = acadoWorkspace.evHu[11];
acadoWorkspace.A[312] = acadoWorkspace.evHu[12];
acadoWorkspace.A[313] = acadoWorkspace.evHu[13];
acadoWorkspace.A[364] = acadoWorkspace.evHu[14];
acadoWorkspace.A[365] = acadoWorkspace.evHu[15];
acadoWorkspace.A[416] = acadoWorkspace.evHu[16];
acadoWorkspace.A[417] = acadoWorkspace.evHu[17];
acadoWorkspace.A[468] = acadoWorkspace.evHu[18];
acadoWorkspace.A[469] = acadoWorkspace.evHu[19];
acadoWorkspace.A[520] = acadoWorkspace.evHu[20];
acadoWorkspace.A[521] = acadoWorkspace.evHu[21];
acadoWorkspace.A[572] = acadoWorkspace.evHu[22];
acadoWorkspace.A[573] = acadoWorkspace.evHu[23];
acadoWorkspace.A[624] = acadoWorkspace.evHu[24];
acadoWorkspace.A[625] = acadoWorkspace.evHu[25];
acadoWorkspace.A[676] = acadoWorkspace.evHu[26];
acadoWorkspace.A[677] = acadoWorkspace.evHu[27];
acadoWorkspace.A[728] = acadoWorkspace.evHu[28];
acadoWorkspace.A[729] = acadoWorkspace.evHu[29];
acadoWorkspace.A[780] = acadoWorkspace.evHu[30];
acadoWorkspace.A[781] = acadoWorkspace.evHu[31];
acadoWorkspace.A[832] = acadoWorkspace.evHu[32];
acadoWorkspace.A[833] = acadoWorkspace.evHu[33];
acadoWorkspace.A[884] = acadoWorkspace.evHu[34];
acadoWorkspace.A[885] = acadoWorkspace.evHu[35];
acadoWorkspace.A[936] = acadoWorkspace.evHu[36];
acadoWorkspace.A[937] = acadoWorkspace.evHu[37];
acadoWorkspace.A[988] = acadoWorkspace.evHu[38];
acadoWorkspace.A[989] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1040] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1041] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1092] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1093] = acadoWorkspace.evHu[43];
acadoWorkspace.A[1144] = acadoWorkspace.evHu[44];
acadoWorkspace.A[1145] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1196] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1197] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1248] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1249] = acadoWorkspace.evHu[49];
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

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[75] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[76] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[77] + acadoWorkspace.QDy[75];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[75] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[76] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[77] + acadoWorkspace.QDy[76];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[75] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[76] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[77] + acadoWorkspace.QDy[77];
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 138 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 46 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 138 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.g[ 46 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 207 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 69 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 132 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 126 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 114 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 114 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 171 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 57 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 102 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 102 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 153 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 51 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 90 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 78 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 78 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 117 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 66 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 66 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 99 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 33 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 42 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 42 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 30 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 18 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 6 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
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


acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 3 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 9 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 15 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 21 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 27 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 33 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 39 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 45 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 51 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 57 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 63 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 69 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.lbA[ 23 ]), &(acadoWorkspace.ubA[ 23 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );

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
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.d[0];
acadoWorkspace.sbar[4] = acadoWorkspace.d[1];
acadoWorkspace.sbar[5] = acadoWorkspace.d[2];
acadoWorkspace.sbar[6] = acadoWorkspace.d[3];
acadoWorkspace.sbar[7] = acadoWorkspace.d[4];
acadoWorkspace.sbar[8] = acadoWorkspace.d[5];
acadoWorkspace.sbar[9] = acadoWorkspace.d[6];
acadoWorkspace.sbar[10] = acadoWorkspace.d[7];
acadoWorkspace.sbar[11] = acadoWorkspace.d[8];
acadoWorkspace.sbar[12] = acadoWorkspace.d[9];
acadoWorkspace.sbar[13] = acadoWorkspace.d[10];
acadoWorkspace.sbar[14] = acadoWorkspace.d[11];
acadoWorkspace.sbar[15] = acadoWorkspace.d[12];
acadoWorkspace.sbar[16] = acadoWorkspace.d[13];
acadoWorkspace.sbar[17] = acadoWorkspace.d[14];
acadoWorkspace.sbar[18] = acadoWorkspace.d[15];
acadoWorkspace.sbar[19] = acadoWorkspace.d[16];
acadoWorkspace.sbar[20] = acadoWorkspace.d[17];
acadoWorkspace.sbar[21] = acadoWorkspace.d[18];
acadoWorkspace.sbar[22] = acadoWorkspace.d[19];
acadoWorkspace.sbar[23] = acadoWorkspace.d[20];
acadoWorkspace.sbar[24] = acadoWorkspace.d[21];
acadoWorkspace.sbar[25] = acadoWorkspace.d[22];
acadoWorkspace.sbar[26] = acadoWorkspace.d[23];
acadoWorkspace.sbar[27] = acadoWorkspace.d[24];
acadoWorkspace.sbar[28] = acadoWorkspace.d[25];
acadoWorkspace.sbar[29] = acadoWorkspace.d[26];
acadoWorkspace.sbar[30] = acadoWorkspace.d[27];
acadoWorkspace.sbar[31] = acadoWorkspace.d[28];
acadoWorkspace.sbar[32] = acadoWorkspace.d[29];
acadoWorkspace.sbar[33] = acadoWorkspace.d[30];
acadoWorkspace.sbar[34] = acadoWorkspace.d[31];
acadoWorkspace.sbar[35] = acadoWorkspace.d[32];
acadoWorkspace.sbar[36] = acadoWorkspace.d[33];
acadoWorkspace.sbar[37] = acadoWorkspace.d[34];
acadoWorkspace.sbar[38] = acadoWorkspace.d[35];
acadoWorkspace.sbar[39] = acadoWorkspace.d[36];
acadoWorkspace.sbar[40] = acadoWorkspace.d[37];
acadoWorkspace.sbar[41] = acadoWorkspace.d[38];
acadoWorkspace.sbar[42] = acadoWorkspace.d[39];
acadoWorkspace.sbar[43] = acadoWorkspace.d[40];
acadoWorkspace.sbar[44] = acadoWorkspace.d[41];
acadoWorkspace.sbar[45] = acadoWorkspace.d[42];
acadoWorkspace.sbar[46] = acadoWorkspace.d[43];
acadoWorkspace.sbar[47] = acadoWorkspace.d[44];
acadoWorkspace.sbar[48] = acadoWorkspace.d[45];
acadoWorkspace.sbar[49] = acadoWorkspace.d[46];
acadoWorkspace.sbar[50] = acadoWorkspace.d[47];
acadoWorkspace.sbar[51] = acadoWorkspace.d[48];
acadoWorkspace.sbar[52] = acadoWorkspace.d[49];
acadoWorkspace.sbar[53] = acadoWorkspace.d[50];
acadoWorkspace.sbar[54] = acadoWorkspace.d[51];
acadoWorkspace.sbar[55] = acadoWorkspace.d[52];
acadoWorkspace.sbar[56] = acadoWorkspace.d[53];
acadoWorkspace.sbar[57] = acadoWorkspace.d[54];
acadoWorkspace.sbar[58] = acadoWorkspace.d[55];
acadoWorkspace.sbar[59] = acadoWorkspace.d[56];
acadoWorkspace.sbar[60] = acadoWorkspace.d[57];
acadoWorkspace.sbar[61] = acadoWorkspace.d[58];
acadoWorkspace.sbar[62] = acadoWorkspace.d[59];
acadoWorkspace.sbar[63] = acadoWorkspace.d[60];
acadoWorkspace.sbar[64] = acadoWorkspace.d[61];
acadoWorkspace.sbar[65] = acadoWorkspace.d[62];
acadoWorkspace.sbar[66] = acadoWorkspace.d[63];
acadoWorkspace.sbar[67] = acadoWorkspace.d[64];
acadoWorkspace.sbar[68] = acadoWorkspace.d[65];
acadoWorkspace.sbar[69] = acadoWorkspace.d[66];
acadoWorkspace.sbar[70] = acadoWorkspace.d[67];
acadoWorkspace.sbar[71] = acadoWorkspace.d[68];
acadoWorkspace.sbar[72] = acadoWorkspace.d[69];
acadoWorkspace.sbar[73] = acadoWorkspace.d[70];
acadoWorkspace.sbar[74] = acadoWorkspace.d[71];
acadoWorkspace.sbar[75] = acadoWorkspace.d[72];
acadoWorkspace.sbar[76] = acadoWorkspace.d[73];
acadoWorkspace.sbar[77] = acadoWorkspace.d[74];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.evGu[ 66 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.evGu[ 78 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.evGu[ 102 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.evGu[ 114 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.evGu[ 126 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.evGu[ 138 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );
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
acadoVariables.mu[72] = 0.0000000000000000e+00;
acadoVariables.mu[73] = 0.0000000000000000e+00;
acadoVariables.mu[74] = 0.0000000000000000e+00;
acadoVariables.mu[72] += + acadoWorkspace.sbar[75]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[76]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[77]*acadoWorkspace.QN1[6];
acadoVariables.mu[73] += + acadoWorkspace.sbar[75]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[76]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[77]*acadoWorkspace.QN1[7];
acadoVariables.mu[74] += + acadoWorkspace.sbar[75]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[76]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[77]*acadoWorkspace.QN1[8];
acadoVariables.mu[72] += acadoWorkspace.QDy[75];
acadoVariables.mu[73] += acadoWorkspace.QDy[76];
acadoVariables.mu[74] += acadoWorkspace.QDy[77];
acadoVariables.mu[69] = 0.0000000000000000e+00;
acadoVariables.mu[69] -= + acadoWorkspace.y[74]*acadoWorkspace.evHx[72];
acadoVariables.mu[70] = 0.0000000000000000e+00;
acadoVariables.mu[70] -= + acadoWorkspace.y[74]*acadoWorkspace.evHx[73];
acadoVariables.mu[71] = 0.0000000000000000e+00;
acadoVariables.mu[71] -= + acadoWorkspace.y[74]*acadoWorkspace.evHx[74];
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoVariables.mu[ 69 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[66] = 0.0000000000000000e+00;
acadoVariables.mu[66] -= + acadoWorkspace.y[73]*acadoWorkspace.evHx[69];
acadoVariables.mu[67] = 0.0000000000000000e+00;
acadoVariables.mu[67] -= + acadoWorkspace.y[73]*acadoWorkspace.evHx[70];
acadoVariables.mu[68] = 0.0000000000000000e+00;
acadoVariables.mu[68] -= + acadoWorkspace.y[73]*acadoWorkspace.evHx[71];
acado_expansionStep2( &(acadoWorkspace.QDy[ 69 ]), &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.S1[ 138 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.evGx[ 207 ]), &(acadoVariables.mu[ 66 ]), &(acadoVariables.mu[ 69 ]) );
acadoVariables.mu[63] = 0.0000000000000000e+00;
acadoVariables.mu[63] -= + acadoWorkspace.y[72]*acadoWorkspace.evHx[66];
acadoVariables.mu[64] = 0.0000000000000000e+00;
acadoVariables.mu[64] -= + acadoWorkspace.y[72]*acadoWorkspace.evHx[67];
acadoVariables.mu[65] = 0.0000000000000000e+00;
acadoVariables.mu[65] -= + acadoWorkspace.y[72]*acadoWorkspace.evHx[68];
acado_expansionStep2( &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.S1[ 132 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.evGx[ 198 ]), &(acadoVariables.mu[ 63 ]), &(acadoVariables.mu[ 66 ]) );
acadoVariables.mu[60] = 0.0000000000000000e+00;
acadoVariables.mu[60] -= + acadoWorkspace.y[71]*acadoWorkspace.evHx[63];
acadoVariables.mu[61] = 0.0000000000000000e+00;
acadoVariables.mu[61] -= + acadoWorkspace.y[71]*acadoWorkspace.evHx[64];
acadoVariables.mu[62] = 0.0000000000000000e+00;
acadoVariables.mu[62] -= + acadoWorkspace.y[71]*acadoWorkspace.evHx[65];
acado_expansionStep2( &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.S1[ 126 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.evGx[ 189 ]), &(acadoVariables.mu[ 60 ]), &(acadoVariables.mu[ 63 ]) );
acadoVariables.mu[57] = 0.0000000000000000e+00;
acadoVariables.mu[57] -= + acadoWorkspace.y[70]*acadoWorkspace.evHx[60];
acadoVariables.mu[58] = 0.0000000000000000e+00;
acadoVariables.mu[58] -= + acadoWorkspace.y[70]*acadoWorkspace.evHx[61];
acadoVariables.mu[59] = 0.0000000000000000e+00;
acadoVariables.mu[59] -= + acadoWorkspace.y[70]*acadoWorkspace.evHx[62];
acado_expansionStep2( &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoVariables.mu[ 57 ]), &(acadoVariables.mu[ 60 ]) );
acadoVariables.mu[54] = 0.0000000000000000e+00;
acadoVariables.mu[54] -= + acadoWorkspace.y[69]*acadoWorkspace.evHx[57];
acadoVariables.mu[55] = 0.0000000000000000e+00;
acadoVariables.mu[55] -= + acadoWorkspace.y[69]*acadoWorkspace.evHx[58];
acadoVariables.mu[56] = 0.0000000000000000e+00;
acadoVariables.mu[56] -= + acadoWorkspace.y[69]*acadoWorkspace.evHx[59];
acado_expansionStep2( &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.S1[ 114 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoVariables.mu[ 54 ]), &(acadoVariables.mu[ 57 ]) );
acadoVariables.mu[51] = 0.0000000000000000e+00;
acadoVariables.mu[51] -= + acadoWorkspace.y[68]*acadoWorkspace.evHx[54];
acadoVariables.mu[52] = 0.0000000000000000e+00;
acadoVariables.mu[52] -= + acadoWorkspace.y[68]*acadoWorkspace.evHx[55];
acadoVariables.mu[53] = 0.0000000000000000e+00;
acadoVariables.mu[53] -= + acadoWorkspace.y[68]*acadoWorkspace.evHx[56];
acado_expansionStep2( &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoVariables.mu[ 51 ]), &(acadoVariables.mu[ 54 ]) );
acadoVariables.mu[48] = 0.0000000000000000e+00;
acadoVariables.mu[48] -= + acadoWorkspace.y[67]*acadoWorkspace.evHx[51];
acadoVariables.mu[49] = 0.0000000000000000e+00;
acadoVariables.mu[49] -= + acadoWorkspace.y[67]*acadoWorkspace.evHx[52];
acadoVariables.mu[50] = 0.0000000000000000e+00;
acadoVariables.mu[50] -= + acadoWorkspace.y[67]*acadoWorkspace.evHx[53];
acado_expansionStep2( &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.S1[ 102 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoVariables.mu[ 48 ]), &(acadoVariables.mu[ 51 ]) );
acadoVariables.mu[45] = 0.0000000000000000e+00;
acadoVariables.mu[45] -= + acadoWorkspace.y[66]*acadoWorkspace.evHx[48];
acadoVariables.mu[46] = 0.0000000000000000e+00;
acadoVariables.mu[46] -= + acadoWorkspace.y[66]*acadoWorkspace.evHx[49];
acadoVariables.mu[47] = 0.0000000000000000e+00;
acadoVariables.mu[47] -= + acadoWorkspace.y[66]*acadoWorkspace.evHx[50];
acado_expansionStep2( &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoVariables.mu[ 45 ]), &(acadoVariables.mu[ 48 ]) );
acadoVariables.mu[42] = 0.0000000000000000e+00;
acadoVariables.mu[42] -= + acadoWorkspace.y[65]*acadoWorkspace.evHx[45];
acadoVariables.mu[43] = 0.0000000000000000e+00;
acadoVariables.mu[43] -= + acadoWorkspace.y[65]*acadoWorkspace.evHx[46];
acadoVariables.mu[44] = 0.0000000000000000e+00;
acadoVariables.mu[44] -= + acadoWorkspace.y[65]*acadoWorkspace.evHx[47];
acado_expansionStep2( &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.S1[ 90 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoVariables.mu[ 42 ]), &(acadoVariables.mu[ 45 ]) );
acadoVariables.mu[39] = 0.0000000000000000e+00;
acadoVariables.mu[39] -= + acadoWorkspace.y[64]*acadoWorkspace.evHx[42];
acadoVariables.mu[40] = 0.0000000000000000e+00;
acadoVariables.mu[40] -= + acadoWorkspace.y[64]*acadoWorkspace.evHx[43];
acadoVariables.mu[41] = 0.0000000000000000e+00;
acadoVariables.mu[41] -= + acadoWorkspace.y[64]*acadoWorkspace.evHx[44];
acado_expansionStep2( &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoVariables.mu[ 39 ]), &(acadoVariables.mu[ 42 ]) );
acadoVariables.mu[36] = 0.0000000000000000e+00;
acadoVariables.mu[36] -= + acadoWorkspace.y[63]*acadoWorkspace.evHx[39];
acadoVariables.mu[37] = 0.0000000000000000e+00;
acadoVariables.mu[37] -= + acadoWorkspace.y[63]*acadoWorkspace.evHx[40];
acadoVariables.mu[38] = 0.0000000000000000e+00;
acadoVariables.mu[38] -= + acadoWorkspace.y[63]*acadoWorkspace.evHx[41];
acado_expansionStep2( &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.S1[ 78 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoVariables.mu[ 36 ]), &(acadoVariables.mu[ 39 ]) );
acadoVariables.mu[33] = 0.0000000000000000e+00;
acadoVariables.mu[33] -= + acadoWorkspace.y[62]*acadoWorkspace.evHx[36];
acadoVariables.mu[34] = 0.0000000000000000e+00;
acadoVariables.mu[34] -= + acadoWorkspace.y[62]*acadoWorkspace.evHx[37];
acadoVariables.mu[35] = 0.0000000000000000e+00;
acadoVariables.mu[35] -= + acadoWorkspace.y[62]*acadoWorkspace.evHx[38];
acado_expansionStep2( &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoVariables.mu[ 33 ]), &(acadoVariables.mu[ 36 ]) );
acadoVariables.mu[30] = 0.0000000000000000e+00;
acadoVariables.mu[30] -= + acadoWorkspace.y[61]*acadoWorkspace.evHx[33];
acadoVariables.mu[31] = 0.0000000000000000e+00;
acadoVariables.mu[31] -= + acadoWorkspace.y[61]*acadoWorkspace.evHx[34];
acadoVariables.mu[32] = 0.0000000000000000e+00;
acadoVariables.mu[32] -= + acadoWorkspace.y[61]*acadoWorkspace.evHx[35];
acado_expansionStep2( &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.S1[ 66 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoVariables.mu[ 30 ]), &(acadoVariables.mu[ 33 ]) );
acadoVariables.mu[27] = 0.0000000000000000e+00;
acadoVariables.mu[27] -= + acadoWorkspace.y[60]*acadoWorkspace.evHx[30];
acadoVariables.mu[28] = 0.0000000000000000e+00;
acadoVariables.mu[28] -= + acadoWorkspace.y[60]*acadoWorkspace.evHx[31];
acadoVariables.mu[29] = 0.0000000000000000e+00;
acadoVariables.mu[29] -= + acadoWorkspace.y[60]*acadoWorkspace.evHx[32];
acado_expansionStep2( &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoVariables.mu[ 27 ]), &(acadoVariables.mu[ 30 ]) );
acadoVariables.mu[24] = 0.0000000000000000e+00;
acadoVariables.mu[24] -= + acadoWorkspace.y[59]*acadoWorkspace.evHx[27];
acadoVariables.mu[25] = 0.0000000000000000e+00;
acadoVariables.mu[25] -= + acadoWorkspace.y[59]*acadoWorkspace.evHx[28];
acadoVariables.mu[26] = 0.0000000000000000e+00;
acadoVariables.mu[26] -= + acadoWorkspace.y[59]*acadoWorkspace.evHx[29];
acado_expansionStep2( &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoVariables.mu[ 24 ]), &(acadoVariables.mu[ 27 ]) );
acadoVariables.mu[21] = 0.0000000000000000e+00;
acadoVariables.mu[21] -= + acadoWorkspace.y[58]*acadoWorkspace.evHx[24];
acadoVariables.mu[22] = 0.0000000000000000e+00;
acadoVariables.mu[22] -= + acadoWorkspace.y[58]*acadoWorkspace.evHx[25];
acadoVariables.mu[23] = 0.0000000000000000e+00;
acadoVariables.mu[23] -= + acadoWorkspace.y[58]*acadoWorkspace.evHx[26];
acado_expansionStep2( &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoVariables.mu[ 21 ]), &(acadoVariables.mu[ 24 ]) );
acadoVariables.mu[18] = 0.0000000000000000e+00;
acadoVariables.mu[18] -= + acadoWorkspace.y[57]*acadoWorkspace.evHx[21];
acadoVariables.mu[19] = 0.0000000000000000e+00;
acadoVariables.mu[19] -= + acadoWorkspace.y[57]*acadoWorkspace.evHx[22];
acadoVariables.mu[20] = 0.0000000000000000e+00;
acadoVariables.mu[20] -= + acadoWorkspace.y[57]*acadoWorkspace.evHx[23];
acado_expansionStep2( &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.S1[ 42 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoVariables.mu[ 18 ]), &(acadoVariables.mu[ 21 ]) );
acadoVariables.mu[15] = 0.0000000000000000e+00;
acadoVariables.mu[15] -= + acadoWorkspace.y[56]*acadoWorkspace.evHx[18];
acadoVariables.mu[16] = 0.0000000000000000e+00;
acadoVariables.mu[16] -= + acadoWorkspace.y[56]*acadoWorkspace.evHx[19];
acadoVariables.mu[17] = 0.0000000000000000e+00;
acadoVariables.mu[17] -= + acadoWorkspace.y[56]*acadoWorkspace.evHx[20];
acado_expansionStep2( &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoVariables.mu[ 15 ]), &(acadoVariables.mu[ 18 ]) );
acadoVariables.mu[12] = 0.0000000000000000e+00;
acadoVariables.mu[12] -= + acadoWorkspace.y[55]*acadoWorkspace.evHx[15];
acadoVariables.mu[13] = 0.0000000000000000e+00;
acadoVariables.mu[13] -= + acadoWorkspace.y[55]*acadoWorkspace.evHx[16];
acadoVariables.mu[14] = 0.0000000000000000e+00;
acadoVariables.mu[14] -= + acadoWorkspace.y[55]*acadoWorkspace.evHx[17];
acado_expansionStep2( &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.S1[ 30 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoVariables.mu[ 12 ]), &(acadoVariables.mu[ 15 ]) );
acadoVariables.mu[9] = 0.0000000000000000e+00;
acadoVariables.mu[9] -= + acadoWorkspace.y[54]*acadoWorkspace.evHx[12];
acadoVariables.mu[10] = 0.0000000000000000e+00;
acadoVariables.mu[10] -= + acadoWorkspace.y[54]*acadoWorkspace.evHx[13];
acadoVariables.mu[11] = 0.0000000000000000e+00;
acadoVariables.mu[11] -= + acadoWorkspace.y[54]*acadoWorkspace.evHx[14];
acado_expansionStep2( &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoVariables.mu[ 9 ]), &(acadoVariables.mu[ 12 ]) );
acadoVariables.mu[6] = 0.0000000000000000e+00;
acadoVariables.mu[6] -= + acadoWorkspace.y[53]*acadoWorkspace.evHx[9];
acadoVariables.mu[7] = 0.0000000000000000e+00;
acadoVariables.mu[7] -= + acadoWorkspace.y[53]*acadoWorkspace.evHx[10];
acadoVariables.mu[8] = 0.0000000000000000e+00;
acadoVariables.mu[8] -= + acadoWorkspace.y[53]*acadoWorkspace.evHx[11];
acado_expansionStep2( &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.S1[ 18 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoVariables.mu[ 6 ]), &(acadoVariables.mu[ 9 ]) );
acadoVariables.mu[3] = 0.0000000000000000e+00;
acadoVariables.mu[3] -= + acadoWorkspace.y[52]*acadoWorkspace.evHx[6];
acadoVariables.mu[4] = 0.0000000000000000e+00;
acadoVariables.mu[4] -= + acadoWorkspace.y[52]*acadoWorkspace.evHx[7];
acadoVariables.mu[5] = 0.0000000000000000e+00;
acadoVariables.mu[5] -= + acadoWorkspace.y[52]*acadoWorkspace.evHx[8];
acado_expansionStep2( &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.S1[ 12 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoVariables.mu[ 3 ]), &(acadoVariables.mu[ 6 ]) );
acadoVariables.mu[0] = 0.0000000000000000e+00;
acadoVariables.mu[0] -= + acadoWorkspace.y[51]*acadoWorkspace.evHx[3];
acadoVariables.mu[1] = 0.0000000000000000e+00;
acadoVariables.mu[1] -= + acadoWorkspace.y[51]*acadoWorkspace.evHx[4];
acadoVariables.mu[2] = 0.0000000000000000e+00;
acadoVariables.mu[2] -= + acadoWorkspace.y[51]*acadoWorkspace.evHx[5];
acado_expansionStep2( &(acadoWorkspace.QDy[ 3 ]), &(acadoWorkspace.Q1[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.S1[ 6 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.evGx[ 9 ]), acadoVariables.mu, &(acadoVariables.mu[ 3 ]) );
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
acadoVariables.lbValues[2] = -1.0000000000000000e+00;
acadoVariables.lbValues[3] = -1.0000000000000000e+00;
acadoVariables.lbValues[4] = -1.0000000000000000e+00;
acadoVariables.lbValues[5] = -1.0000000000000000e+00;
acadoVariables.lbValues[6] = -1.0000000000000000e+00;
acadoVariables.lbValues[7] = -1.0000000000000000e+00;
acadoVariables.lbValues[8] = -1.0000000000000000e+00;
acadoVariables.lbValues[9] = -1.0000000000000000e+00;
acadoVariables.lbValues[10] = -1.0000000000000000e+00;
acadoVariables.lbValues[11] = -1.0000000000000000e+00;
acadoVariables.lbValues[12] = -1.0000000000000000e+00;
acadoVariables.lbValues[13] = -1.0000000000000000e+00;
acadoVariables.lbValues[14] = -1.0000000000000000e+00;
acadoVariables.lbValues[15] = -1.0000000000000000e+00;
acadoVariables.lbValues[16] = -1.0000000000000000e+00;
acadoVariables.lbValues[17] = -1.0000000000000000e+00;
acadoVariables.lbValues[18] = -1.0000000000000000e+00;
acadoVariables.lbValues[19] = -1.0000000000000000e+00;
acadoVariables.lbValues[20] = -1.0000000000000000e+00;
acadoVariables.lbValues[21] = -1.0000000000000000e+00;
acadoVariables.lbValues[22] = -1.0000000000000000e+00;
acadoVariables.lbValues[23] = -1.0000000000000000e+00;
acadoVariables.lbValues[24] = -1.0000000000000000e+00;
acadoVariables.lbValues[25] = -1.0000000000000000e+00;
acadoVariables.lbValues[26] = -1.0000000000000000e+00;
acadoVariables.lbValues[27] = -1.0000000000000000e+00;
acadoVariables.lbValues[28] = -1.0000000000000000e+00;
acadoVariables.lbValues[29] = -1.0000000000000000e+00;
acadoVariables.lbValues[30] = -1.0000000000000000e+00;
acadoVariables.lbValues[31] = -1.0000000000000000e+00;
acadoVariables.lbValues[32] = -1.0000000000000000e+00;
acadoVariables.lbValues[33] = -1.0000000000000000e+00;
acadoVariables.lbValues[34] = -1.0000000000000000e+00;
acadoVariables.lbValues[35] = -1.0000000000000000e+00;
acadoVariables.lbValues[36] = -1.0000000000000000e+00;
acadoVariables.lbValues[37] = -1.0000000000000000e+00;
acadoVariables.lbValues[38] = -1.0000000000000000e+00;
acadoVariables.lbValues[39] = -1.0000000000000000e+00;
acadoVariables.lbValues[40] = -1.0000000000000000e+00;
acadoVariables.lbValues[41] = -1.0000000000000000e+00;
acadoVariables.lbValues[42] = -1.0000000000000000e+00;
acadoVariables.lbValues[43] = -1.0000000000000000e+00;
acadoVariables.lbValues[44] = -1.0000000000000000e+00;
acadoVariables.lbValues[45] = -1.0000000000000000e+00;
acadoVariables.lbValues[46] = -1.0000000000000000e+00;
acadoVariables.lbValues[47] = -1.0000000000000000e+00;
acadoVariables.lbValues[48] = -1.0000000000000000e+00;
acadoVariables.lbValues[49] = -1.0000000000000000e+00;
acadoVariables.ubValues[0] = 1.0000000000000000e+00;
acadoVariables.ubValues[1] = 1.0000000000000000e+00;
acadoVariables.ubValues[2] = 1.0000000000000000e+00;
acadoVariables.ubValues[3] = 1.0000000000000000e+00;
acadoVariables.ubValues[4] = 1.0000000000000000e+00;
acadoVariables.ubValues[5] = 1.0000000000000000e+00;
acadoVariables.ubValues[6] = 1.0000000000000000e+00;
acadoVariables.ubValues[7] = 1.0000000000000000e+00;
acadoVariables.ubValues[8] = 1.0000000000000000e+00;
acadoVariables.ubValues[9] = 1.0000000000000000e+00;
acadoVariables.ubValues[10] = 1.0000000000000000e+00;
acadoVariables.ubValues[11] = 1.0000000000000000e+00;
acadoVariables.ubValues[12] = 1.0000000000000000e+00;
acadoVariables.ubValues[13] = 1.0000000000000000e+00;
acadoVariables.ubValues[14] = 1.0000000000000000e+00;
acadoVariables.ubValues[15] = 1.0000000000000000e+00;
acadoVariables.ubValues[16] = 1.0000000000000000e+00;
acadoVariables.ubValues[17] = 1.0000000000000000e+00;
acadoVariables.ubValues[18] = 1.0000000000000000e+00;
acadoVariables.ubValues[19] = 1.0000000000000000e+00;
acadoVariables.ubValues[20] = 1.0000000000000000e+00;
acadoVariables.ubValues[21] = 1.0000000000000000e+00;
acadoVariables.ubValues[22] = 1.0000000000000000e+00;
acadoVariables.ubValues[23] = 1.0000000000000000e+00;
acadoVariables.ubValues[24] = 1.0000000000000000e+00;
acadoVariables.ubValues[25] = 1.0000000000000000e+00;
acadoVariables.ubValues[26] = 1.0000000000000000e+00;
acadoVariables.ubValues[27] = 1.0000000000000000e+00;
acadoVariables.ubValues[28] = 1.0000000000000000e+00;
acadoVariables.ubValues[29] = 1.0000000000000000e+00;
acadoVariables.ubValues[30] = 1.0000000000000000e+00;
acadoVariables.ubValues[31] = 1.0000000000000000e+00;
acadoVariables.ubValues[32] = 1.0000000000000000e+00;
acadoVariables.ubValues[33] = 1.0000000000000000e+00;
acadoVariables.ubValues[34] = 1.0000000000000000e+00;
acadoVariables.ubValues[35] = 1.0000000000000000e+00;
acadoVariables.ubValues[36] = 1.0000000000000000e+00;
acadoVariables.ubValues[37] = 1.0000000000000000e+00;
acadoVariables.ubValues[38] = 1.0000000000000000e+00;
acadoVariables.ubValues[39] = 1.0000000000000000e+00;
acadoVariables.ubValues[40] = 1.0000000000000000e+00;
acadoVariables.ubValues[41] = 1.0000000000000000e+00;
acadoVariables.ubValues[42] = 1.0000000000000000e+00;
acadoVariables.ubValues[43] = 1.0000000000000000e+00;
acadoVariables.ubValues[44] = 1.0000000000000000e+00;
acadoVariables.ubValues[45] = 1.0000000000000000e+00;
acadoVariables.ubValues[46] = 1.0000000000000000e+00;
acadoVariables.ubValues[47] = 1.0000000000000000e+00;
acadoVariables.ubValues[48] = 1.0000000000000000e+00;
acadoVariables.ubValues[49] = 1.0000000000000000e+00;
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
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[36] = acadoVariables.u[index * 2];
acadoWorkspace.state[37] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[38] = acadoVariables.od[index * 19];
acadoWorkspace.state[39] = acadoVariables.od[index * 19 + 1];
acadoWorkspace.state[40] = acadoVariables.od[index * 19 + 2];
acadoWorkspace.state[41] = acadoVariables.od[index * 19 + 3];
acadoWorkspace.state[42] = acadoVariables.od[index * 19 + 4];
acadoWorkspace.state[43] = acadoVariables.od[index * 19 + 5];
acadoWorkspace.state[44] = acadoVariables.od[index * 19 + 6];
acadoWorkspace.state[45] = acadoVariables.od[index * 19 + 7];
acadoWorkspace.state[46] = acadoVariables.od[index * 19 + 8];
acadoWorkspace.state[47] = acadoVariables.od[index * 19 + 9];
acadoWorkspace.state[48] = acadoVariables.od[index * 19 + 10];
acadoWorkspace.state[49] = acadoVariables.od[index * 19 + 11];
acadoWorkspace.state[50] = acadoVariables.od[index * 19 + 12];
acadoWorkspace.state[51] = acadoVariables.od[index * 19 + 13];
acadoWorkspace.state[52] = acadoVariables.od[index * 19 + 14];
acadoWorkspace.state[53] = acadoVariables.od[index * 19 + 15];
acadoWorkspace.state[54] = acadoVariables.od[index * 19 + 16];
acadoWorkspace.state[55] = acadoVariables.od[index * 19 + 17];
acadoWorkspace.state[56] = acadoVariables.od[index * 19 + 18];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 25; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[75] = xEnd[0];
acadoVariables.x[76] = xEnd[1];
acadoVariables.x[77] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[75];
acadoWorkspace.state[1] = acadoVariables.x[76];
acadoWorkspace.state[2] = acadoVariables.x[77];
if (uEnd != 0)
{
acadoWorkspace.state[36] = uEnd[0];
acadoWorkspace.state[37] = uEnd[1];
}
else
{
acadoWorkspace.state[36] = acadoVariables.u[48];
acadoWorkspace.state[37] = acadoVariables.u[49];
}
acadoWorkspace.state[38] = acadoVariables.od[475];
acadoWorkspace.state[39] = acadoVariables.od[476];
acadoWorkspace.state[40] = acadoVariables.od[477];
acadoWorkspace.state[41] = acadoVariables.od[478];
acadoWorkspace.state[42] = acadoVariables.od[479];
acadoWorkspace.state[43] = acadoVariables.od[480];
acadoWorkspace.state[44] = acadoVariables.od[481];
acadoWorkspace.state[45] = acadoVariables.od[482];
acadoWorkspace.state[46] = acadoVariables.od[483];
acadoWorkspace.state[47] = acadoVariables.od[484];
acadoWorkspace.state[48] = acadoVariables.od[485];
acadoWorkspace.state[49] = acadoVariables.od[486];
acadoWorkspace.state[50] = acadoVariables.od[487];
acadoWorkspace.state[51] = acadoVariables.od[488];
acadoWorkspace.state[52] = acadoVariables.od[489];
acadoWorkspace.state[53] = acadoVariables.od[490];
acadoWorkspace.state[54] = acadoVariables.od[491];
acadoWorkspace.state[55] = acadoVariables.od[492];
acadoWorkspace.state[56] = acadoVariables.od[493];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[75] = acadoWorkspace.state[0];
acadoVariables.x[76] = acadoWorkspace.state[1];
acadoVariables.x[77] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 24; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[48] = uEnd[0];
acadoVariables.u[49] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49];
kkt = fabs( kkt );
for (index = 0; index < 50; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 25; ++index)
{
prd = acadoWorkspace.y[index + 50];
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 19];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 19 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 19 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 19 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 19 + 4];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 19 + 5];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 19 + 6];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 19 + 7];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 19 + 8];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 19 + 9];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 19 + 10];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 19 + 11];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 19 + 12];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 19 + 13];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 19 + 14];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 19 + 15];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 19 + 16];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 19 + 17];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 19 + 18];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[75];
acadoWorkspace.objValueIn[1] = acadoVariables.x[76];
acadoWorkspace.objValueIn[2] = acadoVariables.x[77];
acadoWorkspace.objValueIn[3] = acadoVariables.od[475];
acadoWorkspace.objValueIn[4] = acadoVariables.od[476];
acadoWorkspace.objValueIn[5] = acadoVariables.od[477];
acadoWorkspace.objValueIn[6] = acadoVariables.od[478];
acadoWorkspace.objValueIn[7] = acadoVariables.od[479];
acadoWorkspace.objValueIn[8] = acadoVariables.od[480];
acadoWorkspace.objValueIn[9] = acadoVariables.od[481];
acadoWorkspace.objValueIn[10] = acadoVariables.od[482];
acadoWorkspace.objValueIn[11] = acadoVariables.od[483];
acadoWorkspace.objValueIn[12] = acadoVariables.od[484];
acadoWorkspace.objValueIn[13] = acadoVariables.od[485];
acadoWorkspace.objValueIn[14] = acadoVariables.od[486];
acadoWorkspace.objValueIn[15] = acadoVariables.od[487];
acadoWorkspace.objValueIn[16] = acadoVariables.od[488];
acadoWorkspace.objValueIn[17] = acadoVariables.od[489];
acadoWorkspace.objValueIn[18] = acadoVariables.od[490];
acadoWorkspace.objValueIn[19] = acadoVariables.od[491];
acadoWorkspace.objValueIn[20] = acadoVariables.od[492];
acadoWorkspace.objValueIn[21] = acadoVariables.od[493];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
return objVal;
}

