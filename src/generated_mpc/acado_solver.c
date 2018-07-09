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
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[3] = acadoVariables.mu[lRun1 * 3];
acadoWorkspace.state[4] = acadoVariables.mu[lRun1 * 3 + 1];
acadoWorkspace.state[5] = acadoVariables.mu[lRun1 * 3 + 2];
acadoWorkspace.state[36] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[37] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[38] = acadoVariables.od[lRun1 * 18];
acadoWorkspace.state[39] = acadoVariables.od[lRun1 * 18 + 1];
acadoWorkspace.state[40] = acadoVariables.od[lRun1 * 18 + 2];
acadoWorkspace.state[41] = acadoVariables.od[lRun1 * 18 + 3];
acadoWorkspace.state[42] = acadoVariables.od[lRun1 * 18 + 4];
acadoWorkspace.state[43] = acadoVariables.od[lRun1 * 18 + 5];
acadoWorkspace.state[44] = acadoVariables.od[lRun1 * 18 + 6];
acadoWorkspace.state[45] = acadoVariables.od[lRun1 * 18 + 7];
acadoWorkspace.state[46] = acadoVariables.od[lRun1 * 18 + 8];
acadoWorkspace.state[47] = acadoVariables.od[lRun1 * 18 + 9];
acadoWorkspace.state[48] = acadoVariables.od[lRun1 * 18 + 10];
acadoWorkspace.state[49] = acadoVariables.od[lRun1 * 18 + 11];
acadoWorkspace.state[50] = acadoVariables.od[lRun1 * 18 + 12];
acadoWorkspace.state[51] = acadoVariables.od[lRun1 * 18 + 13];
acadoWorkspace.state[52] = acadoVariables.od[lRun1 * 18 + 14];
acadoWorkspace.state[53] = acadoVariables.od[lRun1 * 18 + 15];
acadoWorkspace.state[54] = acadoVariables.od[lRun1 * 18 + 16];
acadoWorkspace.state[55] = acadoVariables.od[lRun1 * 18 + 17];

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
const real_t* od = in + 7;
/* Vector of auxiliary variables; number of elements: 114. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(od[15]));
a[1] = ((xd[0]-(a[0]*od[12]))-od[13]);
a[2] = (cos(od[15]));
a[3] = (a[1]*a[2]);
a[4] = (sin(od[15]));
a[5] = ((xd[1]-(a[4]*od[12]))-od[14]);
a[6] = (sin(od[15]));
a[7] = ((real_t)(0.0000000000000000e+00)-a[6]);
a[8] = (a[3]+(a[5]*a[7]));
a[9] = ((real_t)(1.0000000000000000e+00)/((od[16]+od[11])*(od[16]+od[11])));
a[10] = (a[8]*a[9]);
a[11] = (a[10]*a[2]);
a[12] = (sin(od[15]));
a[13] = (a[1]*a[12]);
a[14] = (cos(od[15]));
a[15] = (a[13]+(a[5]*a[14]));
a[16] = ((real_t)(1.0000000000000000e+00)/((od[17]+od[11])*(od[17]+od[11])));
a[17] = (a[15]*a[16]);
a[18] = (a[11]+(a[17]*a[12]));
a[19] = (a[18]*a[1]);
a[20] = (a[10]*a[7]);
a[21] = (a[20]+(a[17]*a[14]));
a[22] = (a[19]+(a[21]*a[5]));
a[23] = (cos(od[15]));
a[24] = ((xd[0]+(a[23]*od[12]))-od[13]);
a[25] = (a[24]*a[2]);
a[26] = (sin(od[15]));
a[27] = ((xd[1]+(a[26]*od[12]))-od[14]);
a[28] = (a[25]+(a[27]*a[7]));
a[29] = (a[28]*a[9]);
a[30] = (a[29]*a[2]);
a[31] = (a[24]*a[12]);
a[32] = (a[31]+(a[27]*a[14]));
a[33] = (a[32]*a[16]);
a[34] = (a[30]+(a[33]*a[12]));
a[35] = (a[34]*a[24]);
a[36] = (a[29]*a[7]);
a[37] = (a[36]+(a[33]*a[14]));
a[38] = (a[35]+(a[37]*a[27]));
a[39] = (a[2]*a[9]);
a[40] = (a[39]*a[2]);
a[41] = (a[12]*a[16]);
a[42] = (a[41]*a[12]);
a[43] = (a[40]+a[42]);
a[44] = (a[43]*a[1]);
a[45] = (a[44]+a[18]);
a[46] = (a[39]*a[7]);
a[47] = (a[41]*a[14]);
a[48] = (a[46]+a[47]);
a[49] = (a[48]*a[5]);
a[50] = (a[45]+a[49]);
a[51] = (a[7]*a[9]);
a[52] = (a[51]*a[2]);
a[53] = (a[14]*a[16]);
a[54] = (a[53]*a[12]);
a[55] = (a[52]+a[54]);
a[56] = (a[55]*a[1]);
a[57] = (a[51]*a[7]);
a[58] = (a[53]*a[14]);
a[59] = (a[57]+a[58]);
a[60] = (a[59]*a[5]);
a[61] = (a[60]+a[21]);
a[62] = (a[56]+a[61]);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (a[2]*a[9]);
a[65] = (a[64]*a[2]);
a[66] = (a[12]*a[16]);
a[67] = (a[66]*a[12]);
a[68] = (a[65]+a[67]);
a[69] = (a[68]*a[24]);
a[70] = (a[69]+a[34]);
a[71] = (a[64]*a[7]);
a[72] = (a[66]*a[14]);
a[73] = (a[71]+a[72]);
a[74] = (a[73]*a[27]);
a[75] = (a[70]+a[74]);
a[76] = (a[7]*a[9]);
a[77] = (a[76]*a[2]);
a[78] = (a[14]*a[16]);
a[79] = (a[78]*a[12]);
a[80] = (a[77]+a[79]);
a[81] = (a[80]*a[24]);
a[82] = (a[76]*a[7]);
a[83] = (a[78]*a[14]);
a[84] = (a[82]+a[83]);
a[85] = (a[84]*a[27]);
a[86] = (a[85]+a[37]);
a[87] = (a[81]+a[86]);
a[88] = (real_t)(0.0000000000000000e+00);
a[89] = (real_t)(0.0000000000000000e+00);
a[90] = (real_t)(0.0000000000000000e+00);
a[91] = (real_t)(0.0000000000000000e+00);
a[92] = (real_t)(0.0000000000000000e+00);
a[93] = (a[43]*(real_t)(2.0000000000000000e+00));
a[94] = (a[68]*(real_t)(2.0000000000000000e+00));
a[95] = ((a[93]*xd[3])+(a[94]*xd[4]));
a[96] = (a[55]+a[48]);
a[97] = (a[80]+a[73]);
a[98] = ((a[96]*xd[3])+(a[97]*xd[4]));
a[99] = (real_t)(0.0000000000000000e+00);
a[100] = (real_t)(0.0000000000000000e+00);
a[101] = (real_t)(0.0000000000000000e+00);
a[102] = (a[59]*(real_t)(2.0000000000000000e+00));
a[103] = (a[84]*(real_t)(2.0000000000000000e+00));
a[104] = ((a[102]*xd[3])+(a[103]*xd[4]));
a[105] = (real_t)(0.0000000000000000e+00);
a[106] = (real_t)(0.0000000000000000e+00);
a[107] = (real_t)(0.0000000000000000e+00);
a[108] = (real_t)(0.0000000000000000e+00);
a[109] = (real_t)(0.0000000000000000e+00);
a[110] = (real_t)(0.0000000000000000e+00);
a[111] = (real_t)(0.0000000000000000e+00);
a[112] = (real_t)(0.0000000000000000e+00);
a[113] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[22];
out[1] = a[38];
out[2] = a[50];
out[3] = a[62];
out[4] = a[63];
out[5] = a[75];
out[6] = a[87];
out[7] = a[88];
out[8] = a[89];
out[9] = a[90];
out[10] = a[91];
out[11] = a[92];
out[12] = a[95];
out[13] = a[98];
out[14] = a[99];
out[15] = a[100];
out[16] = a[101];
out[17] = a[98];
out[18] = a[104];
out[19] = a[105];
out[20] = a[106];
out[21] = a[107];
out[22] = a[99];
out[23] = a[105];
out[24] = a[108];
out[25] = a[109];
out[26] = a[110];
out[27] = a[100];
out[28] = a[106];
out[29] = a[109];
out[30] = a[111];
out[31] = a[112];
out[32] = a[101];
out[33] = a[107];
out[34] = a[110];
out[35] = a[112];
out[36] = a[113];
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
for (runObj = 0; runObj < 50; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 18];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 18 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 18 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 18 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 18 + 4];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 18 + 5];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 18 + 6];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 18 + 7];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 18 + 8];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 18 + 9];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 18 + 10];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 18 + 11];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 18 + 12];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 18 + 13];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 18 + 14];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 18 + 15];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 18 + 16];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 18 + 17];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjTerm( &(acadoWorkspace.objValueOut[ 6 ]), &(acadoWorkspace.objValueOut[ 15 ]), &(acadoWorkspace.objValueOut[ 21 ]), &(acadoWorkspace.EH[ runObj * 25 ]) );
acado_addObjLinearTerm( &(acadoWorkspace.QDy[ runObj * 3 ]), &(acadoWorkspace.g[ runObj * 2 ]), &(acadoWorkspace.objValueOut[ 1 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.od[900];
acadoWorkspace.objValueIn[4] = acadoVariables.od[901];
acadoWorkspace.objValueIn[5] = acadoVariables.od[902];
acadoWorkspace.objValueIn[6] = acadoVariables.od[903];
acadoWorkspace.objValueIn[7] = acadoVariables.od[904];
acadoWorkspace.objValueIn[8] = acadoVariables.od[905];
acadoWorkspace.objValueIn[9] = acadoVariables.od[906];
acadoWorkspace.objValueIn[10] = acadoVariables.od[907];
acadoWorkspace.objValueIn[11] = acadoVariables.od[908];
acadoWorkspace.objValueIn[12] = acadoVariables.od[909];
acadoWorkspace.objValueIn[13] = acadoVariables.od[910];
acadoWorkspace.objValueIn[14] = acadoVariables.od[911];
acadoWorkspace.objValueIn[15] = acadoVariables.od[912];
acadoWorkspace.objValueIn[16] = acadoVariables.od[913];
acadoWorkspace.objValueIn[17] = acadoVariables.od[914];
acadoWorkspace.objValueIn[18] = acadoVariables.od[915];
acadoWorkspace.objValueIn[19] = acadoVariables.od[916];
acadoWorkspace.objValueIn[20] = acadoVariables.od[917];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjEndTerm( &(acadoWorkspace.objValueOut[ 4 ]), acadoWorkspace.EH_N );
acadoWorkspace.QDy[150] = acadoWorkspace.objValueOut[1];
acadoWorkspace.QDy[151] = acadoWorkspace.objValueOut[2];
acadoWorkspace.QDy[152] = acadoWorkspace.objValueOut[3];

for (lRun2 = 0; lRun2 < 50; ++lRun2)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun2 * 3];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun2 * 3 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun2 * 3 + 2];
acadoWorkspace.conValueIn[3] = acadoWorkspace.y[lRun2 * 2 + 100];
acadoWorkspace.conValueIn[4] = acadoWorkspace.y[lRun2 * 2 + 101];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun2 * 2];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun2 * 2 + 1];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun2 * 18];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun2 * 18 + 1];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun2 * 18 + 2];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun2 * 18 + 3];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun2 * 18 + 4];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun2 * 18 + 5];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun2 * 18 + 6];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun2 * 18 + 7];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun2 * 18 + 8];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun2 * 18 + 9];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun2 * 18 + 10];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun2 * 18 + 11];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun2 * 18 + 12];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun2 * 18 + 13];
acadoWorkspace.conValueIn[21] = acadoVariables.od[lRun2 * 18 + 14];
acadoWorkspace.conValueIn[22] = acadoVariables.od[lRun2 * 18 + 15];
acadoWorkspace.conValueIn[23] = acadoVariables.od[lRun2 * 18 + 16];
acadoWorkspace.conValueIn[24] = acadoVariables.od[lRun2 * 18 + 17];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun2 * 2] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun2 * 2 + 1] = acadoWorkspace.conValueOut[1];

acadoWorkspace.evHx[lRun2 * 6] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun2 * 6 + 1] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun2 * 6 + 2] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun2 * 6 + 3] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun2 * 6 + 4] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun2 * 6 + 5] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHu[lRun2 * 4] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHu[lRun2 * 4 + 1] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHu[lRun2 * 4 + 2] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHu[lRun2 * 4 + 3] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evDDH[0] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evDDH[1] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evDDH[2] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evDDH[3] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evDDH[4] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evDDH[5] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evDDH[6] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evDDH[7] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evDDH[8] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evDDH[9] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evDDH[10] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evDDH[11] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evDDH[12] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evDDH[13] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evDDH[14] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evDDH[15] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evDDH[16] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evDDH[17] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evDDH[18] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evDDH[19] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evDDH[20] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evDDH[21] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evDDH[22] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evDDH[23] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evDDH[24] = acadoWorkspace.conValueOut[36];
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
for (lRun1 = 0; lRun1 < 50; ++lRun1)
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
acadoWorkspace.H[(iRow * 200) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 200) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 200) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 200) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 202] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + R11[0];
acadoWorkspace.H[iRow * 202 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + R11[1];
acadoWorkspace.H[iRow * 202 + 100] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + R11[2];
acadoWorkspace.H[iRow * 202 + 101] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + R11[3];
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
acadoWorkspace.H[(iRow * 200) + (iCol * 2)] = acadoWorkspace.H[(iCol * 200) + (iRow * 2)];
acadoWorkspace.H[(iRow * 200) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 200 + 100) + (iRow * 2)];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2)] = acadoWorkspace.H[(iCol * 200) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 200 + 100) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 200 + 100) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const RDy1 )
{
}

void acado_multQDy( real_t* const QDy1 )
{
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 200) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4];
acadoWorkspace.A[(row * 200) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5];
acadoWorkspace.A[(row * 200 + 100) + (col * 2)] = + Hx[3]*E[0] + Hx[4]*E[2] + Hx[5]*E[4];
acadoWorkspace.A[(row * 200 + 100) + (col * 2 + 1)] = + Hx[3]*E[1] + Hx[4]*E[3] + Hx[5]*E[5];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2];
acadoWorkspace.evHxd[1] = + Hx[3]*tmpd[0] + Hx[4]*tmpd[1] + Hx[5]*tmpd[2];
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
acado_multGxGx( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.C[ 216 ]), &(acadoWorkspace.C[ 225 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.C[ 225 ]), &(acadoWorkspace.C[ 234 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.C[ 234 ]), &(acadoWorkspace.C[ 243 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.C[ 243 ]), &(acadoWorkspace.C[ 252 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.C[ 252 ]), &(acadoWorkspace.C[ 261 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.C[ 261 ]), &(acadoWorkspace.C[ 270 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.C[ 270 ]), &(acadoWorkspace.C[ 279 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.C[ 279 ]), &(acadoWorkspace.C[ 288 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.C[ 288 ]), &(acadoWorkspace.C[ 297 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.C[ 297 ]), &(acadoWorkspace.C[ 306 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.C[ 306 ]), &(acadoWorkspace.C[ 315 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.C[ 315 ]), &(acadoWorkspace.C[ 324 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.C[ 324 ]), &(acadoWorkspace.C[ 333 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.C[ 333 ]), &(acadoWorkspace.C[ 342 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.C[ 342 ]), &(acadoWorkspace.C[ 351 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.C[ 351 ]), &(acadoWorkspace.C[ 360 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.C[ 360 ]), &(acadoWorkspace.C[ 369 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.C[ 369 ]), &(acadoWorkspace.C[ 378 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.C[ 378 ]), &(acadoWorkspace.C[ 387 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.C[ 387 ]), &(acadoWorkspace.C[ 396 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.C[ 396 ]), &(acadoWorkspace.C[ 405 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.C[ 405 ]), &(acadoWorkspace.C[ 414 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.C[ 414 ]), &(acadoWorkspace.C[ 423 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.C[ 423 ]), &(acadoWorkspace.C[ 432 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.C[ 432 ]), &(acadoWorkspace.C[ 441 ]) );
for (lRun2 = 0; lRun2 < 50; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 101)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 6 ]), &(acadoWorkspace.E[ lRun3 * 6 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 50; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (3)) * (3)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (3)) * (2)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (3)) * (2)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (50)) - (1)) * (3)) * (2)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 49; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 6 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 6 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (2)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 9 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (2)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 4 ]), &(acadoWorkspace.evGu[ lRun2 * 6 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 50; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];




for (lRun1 = 0; lRun1 < 49; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun2) * (lRun2 * -1 + 99)) / (2)) + (lRun1);
lRun4 = lRun1 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun1 * 6 + 6 ]), &(acadoWorkspace.E[ lRun3 * 6 ]), lRun4, lRun2 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[100] = acadoWorkspace.evHu[2];
acadoWorkspace.A[101] = acadoWorkspace.evHu[3];
acadoWorkspace.A[202] = acadoWorkspace.evHu[4];
acadoWorkspace.A[203] = acadoWorkspace.evHu[5];
acadoWorkspace.A[302] = acadoWorkspace.evHu[6];
acadoWorkspace.A[303] = acadoWorkspace.evHu[7];
acadoWorkspace.A[404] = acadoWorkspace.evHu[8];
acadoWorkspace.A[405] = acadoWorkspace.evHu[9];
acadoWorkspace.A[504] = acadoWorkspace.evHu[10];
acadoWorkspace.A[505] = acadoWorkspace.evHu[11];
acadoWorkspace.A[606] = acadoWorkspace.evHu[12];
acadoWorkspace.A[607] = acadoWorkspace.evHu[13];
acadoWorkspace.A[706] = acadoWorkspace.evHu[14];
acadoWorkspace.A[707] = acadoWorkspace.evHu[15];
acadoWorkspace.A[808] = acadoWorkspace.evHu[16];
acadoWorkspace.A[809] = acadoWorkspace.evHu[17];
acadoWorkspace.A[908] = acadoWorkspace.evHu[18];
acadoWorkspace.A[909] = acadoWorkspace.evHu[19];
acadoWorkspace.A[1010] = acadoWorkspace.evHu[20];
acadoWorkspace.A[1011] = acadoWorkspace.evHu[21];
acadoWorkspace.A[1110] = acadoWorkspace.evHu[22];
acadoWorkspace.A[1111] = acadoWorkspace.evHu[23];
acadoWorkspace.A[1212] = acadoWorkspace.evHu[24];
acadoWorkspace.A[1213] = acadoWorkspace.evHu[25];
acadoWorkspace.A[1312] = acadoWorkspace.evHu[26];
acadoWorkspace.A[1313] = acadoWorkspace.evHu[27];
acadoWorkspace.A[1414] = acadoWorkspace.evHu[28];
acadoWorkspace.A[1415] = acadoWorkspace.evHu[29];
acadoWorkspace.A[1514] = acadoWorkspace.evHu[30];
acadoWorkspace.A[1515] = acadoWorkspace.evHu[31];
acadoWorkspace.A[1616] = acadoWorkspace.evHu[32];
acadoWorkspace.A[1617] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1716] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1717] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1818] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1819] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1918] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1919] = acadoWorkspace.evHu[39];
acadoWorkspace.A[2020] = acadoWorkspace.evHu[40];
acadoWorkspace.A[2021] = acadoWorkspace.evHu[41];
acadoWorkspace.A[2120] = acadoWorkspace.evHu[42];
acadoWorkspace.A[2121] = acadoWorkspace.evHu[43];
acadoWorkspace.A[2222] = acadoWorkspace.evHu[44];
acadoWorkspace.A[2223] = acadoWorkspace.evHu[45];
acadoWorkspace.A[2322] = acadoWorkspace.evHu[46];
acadoWorkspace.A[2323] = acadoWorkspace.evHu[47];
acadoWorkspace.A[2424] = acadoWorkspace.evHu[48];
acadoWorkspace.A[2425] = acadoWorkspace.evHu[49];
acadoWorkspace.A[2524] = acadoWorkspace.evHu[50];
acadoWorkspace.A[2525] = acadoWorkspace.evHu[51];
acadoWorkspace.A[2626] = acadoWorkspace.evHu[52];
acadoWorkspace.A[2627] = acadoWorkspace.evHu[53];
acadoWorkspace.A[2726] = acadoWorkspace.evHu[54];
acadoWorkspace.A[2727] = acadoWorkspace.evHu[55];
acadoWorkspace.A[2828] = acadoWorkspace.evHu[56];
acadoWorkspace.A[2829] = acadoWorkspace.evHu[57];
acadoWorkspace.A[2928] = acadoWorkspace.evHu[58];
acadoWorkspace.A[2929] = acadoWorkspace.evHu[59];
acadoWorkspace.A[3030] = acadoWorkspace.evHu[60];
acadoWorkspace.A[3031] = acadoWorkspace.evHu[61];
acadoWorkspace.A[3130] = acadoWorkspace.evHu[62];
acadoWorkspace.A[3131] = acadoWorkspace.evHu[63];
acadoWorkspace.A[3232] = acadoWorkspace.evHu[64];
acadoWorkspace.A[3233] = acadoWorkspace.evHu[65];
acadoWorkspace.A[3332] = acadoWorkspace.evHu[66];
acadoWorkspace.A[3333] = acadoWorkspace.evHu[67];
acadoWorkspace.A[3434] = acadoWorkspace.evHu[68];
acadoWorkspace.A[3435] = acadoWorkspace.evHu[69];
acadoWorkspace.A[3534] = acadoWorkspace.evHu[70];
acadoWorkspace.A[3535] = acadoWorkspace.evHu[71];
acadoWorkspace.A[3636] = acadoWorkspace.evHu[72];
acadoWorkspace.A[3637] = acadoWorkspace.evHu[73];
acadoWorkspace.A[3736] = acadoWorkspace.evHu[74];
acadoWorkspace.A[3737] = acadoWorkspace.evHu[75];
acadoWorkspace.A[3838] = acadoWorkspace.evHu[76];
acadoWorkspace.A[3839] = acadoWorkspace.evHu[77];
acadoWorkspace.A[3938] = acadoWorkspace.evHu[78];
acadoWorkspace.A[3939] = acadoWorkspace.evHu[79];
acadoWorkspace.A[4040] = acadoWorkspace.evHu[80];
acadoWorkspace.A[4041] = acadoWorkspace.evHu[81];
acadoWorkspace.A[4140] = acadoWorkspace.evHu[82];
acadoWorkspace.A[4141] = acadoWorkspace.evHu[83];
acadoWorkspace.A[4242] = acadoWorkspace.evHu[84];
acadoWorkspace.A[4243] = acadoWorkspace.evHu[85];
acadoWorkspace.A[4342] = acadoWorkspace.evHu[86];
acadoWorkspace.A[4343] = acadoWorkspace.evHu[87];
acadoWorkspace.A[4444] = acadoWorkspace.evHu[88];
acadoWorkspace.A[4445] = acadoWorkspace.evHu[89];
acadoWorkspace.A[4544] = acadoWorkspace.evHu[90];
acadoWorkspace.A[4545] = acadoWorkspace.evHu[91];
acadoWorkspace.A[4646] = acadoWorkspace.evHu[92];
acadoWorkspace.A[4647] = acadoWorkspace.evHu[93];
acadoWorkspace.A[4746] = acadoWorkspace.evHu[94];
acadoWorkspace.A[4747] = acadoWorkspace.evHu[95];
acadoWorkspace.A[4848] = acadoWorkspace.evHu[96];
acadoWorkspace.A[4849] = acadoWorkspace.evHu[97];
acadoWorkspace.A[4948] = acadoWorkspace.evHu[98];
acadoWorkspace.A[4949] = acadoWorkspace.evHu[99];
acadoWorkspace.A[5050] = acadoWorkspace.evHu[100];
acadoWorkspace.A[5051] = acadoWorkspace.evHu[101];
acadoWorkspace.A[5150] = acadoWorkspace.evHu[102];
acadoWorkspace.A[5151] = acadoWorkspace.evHu[103];
acadoWorkspace.A[5252] = acadoWorkspace.evHu[104];
acadoWorkspace.A[5253] = acadoWorkspace.evHu[105];
acadoWorkspace.A[5352] = acadoWorkspace.evHu[106];
acadoWorkspace.A[5353] = acadoWorkspace.evHu[107];
acadoWorkspace.A[5454] = acadoWorkspace.evHu[108];
acadoWorkspace.A[5455] = acadoWorkspace.evHu[109];
acadoWorkspace.A[5554] = acadoWorkspace.evHu[110];
acadoWorkspace.A[5555] = acadoWorkspace.evHu[111];
acadoWorkspace.A[5656] = acadoWorkspace.evHu[112];
acadoWorkspace.A[5657] = acadoWorkspace.evHu[113];
acadoWorkspace.A[5756] = acadoWorkspace.evHu[114];
acadoWorkspace.A[5757] = acadoWorkspace.evHu[115];
acadoWorkspace.A[5858] = acadoWorkspace.evHu[116];
acadoWorkspace.A[5859] = acadoWorkspace.evHu[117];
acadoWorkspace.A[5958] = acadoWorkspace.evHu[118];
acadoWorkspace.A[5959] = acadoWorkspace.evHu[119];
acadoWorkspace.A[6060] = acadoWorkspace.evHu[120];
acadoWorkspace.A[6061] = acadoWorkspace.evHu[121];
acadoWorkspace.A[6160] = acadoWorkspace.evHu[122];
acadoWorkspace.A[6161] = acadoWorkspace.evHu[123];
acadoWorkspace.A[6262] = acadoWorkspace.evHu[124];
acadoWorkspace.A[6263] = acadoWorkspace.evHu[125];
acadoWorkspace.A[6362] = acadoWorkspace.evHu[126];
acadoWorkspace.A[6363] = acadoWorkspace.evHu[127];
acadoWorkspace.A[6464] = acadoWorkspace.evHu[128];
acadoWorkspace.A[6465] = acadoWorkspace.evHu[129];
acadoWorkspace.A[6564] = acadoWorkspace.evHu[130];
acadoWorkspace.A[6565] = acadoWorkspace.evHu[131];
acadoWorkspace.A[6666] = acadoWorkspace.evHu[132];
acadoWorkspace.A[6667] = acadoWorkspace.evHu[133];
acadoWorkspace.A[6766] = acadoWorkspace.evHu[134];
acadoWorkspace.A[6767] = acadoWorkspace.evHu[135];
acadoWorkspace.A[6868] = acadoWorkspace.evHu[136];
acadoWorkspace.A[6869] = acadoWorkspace.evHu[137];
acadoWorkspace.A[6968] = acadoWorkspace.evHu[138];
acadoWorkspace.A[6969] = acadoWorkspace.evHu[139];
acadoWorkspace.A[7070] = acadoWorkspace.evHu[140];
acadoWorkspace.A[7071] = acadoWorkspace.evHu[141];
acadoWorkspace.A[7170] = acadoWorkspace.evHu[142];
acadoWorkspace.A[7171] = acadoWorkspace.evHu[143];
acadoWorkspace.A[7272] = acadoWorkspace.evHu[144];
acadoWorkspace.A[7273] = acadoWorkspace.evHu[145];
acadoWorkspace.A[7372] = acadoWorkspace.evHu[146];
acadoWorkspace.A[7373] = acadoWorkspace.evHu[147];
acadoWorkspace.A[7474] = acadoWorkspace.evHu[148];
acadoWorkspace.A[7475] = acadoWorkspace.evHu[149];
acadoWorkspace.A[7574] = acadoWorkspace.evHu[150];
acadoWorkspace.A[7575] = acadoWorkspace.evHu[151];
acadoWorkspace.A[7676] = acadoWorkspace.evHu[152];
acadoWorkspace.A[7677] = acadoWorkspace.evHu[153];
acadoWorkspace.A[7776] = acadoWorkspace.evHu[154];
acadoWorkspace.A[7777] = acadoWorkspace.evHu[155];
acadoWorkspace.A[7878] = acadoWorkspace.evHu[156];
acadoWorkspace.A[7879] = acadoWorkspace.evHu[157];
acadoWorkspace.A[7978] = acadoWorkspace.evHu[158];
acadoWorkspace.A[7979] = acadoWorkspace.evHu[159];
acadoWorkspace.A[8080] = acadoWorkspace.evHu[160];
acadoWorkspace.A[8081] = acadoWorkspace.evHu[161];
acadoWorkspace.A[8180] = acadoWorkspace.evHu[162];
acadoWorkspace.A[8181] = acadoWorkspace.evHu[163];
acadoWorkspace.A[8282] = acadoWorkspace.evHu[164];
acadoWorkspace.A[8283] = acadoWorkspace.evHu[165];
acadoWorkspace.A[8382] = acadoWorkspace.evHu[166];
acadoWorkspace.A[8383] = acadoWorkspace.evHu[167];
acadoWorkspace.A[8484] = acadoWorkspace.evHu[168];
acadoWorkspace.A[8485] = acadoWorkspace.evHu[169];
acadoWorkspace.A[8584] = acadoWorkspace.evHu[170];
acadoWorkspace.A[8585] = acadoWorkspace.evHu[171];
acadoWorkspace.A[8686] = acadoWorkspace.evHu[172];
acadoWorkspace.A[8687] = acadoWorkspace.evHu[173];
acadoWorkspace.A[8786] = acadoWorkspace.evHu[174];
acadoWorkspace.A[8787] = acadoWorkspace.evHu[175];
acadoWorkspace.A[8888] = acadoWorkspace.evHu[176];
acadoWorkspace.A[8889] = acadoWorkspace.evHu[177];
acadoWorkspace.A[8988] = acadoWorkspace.evHu[178];
acadoWorkspace.A[8989] = acadoWorkspace.evHu[179];
acadoWorkspace.A[9090] = acadoWorkspace.evHu[180];
acadoWorkspace.A[9091] = acadoWorkspace.evHu[181];
acadoWorkspace.A[9190] = acadoWorkspace.evHu[182];
acadoWorkspace.A[9191] = acadoWorkspace.evHu[183];
acadoWorkspace.A[9292] = acadoWorkspace.evHu[184];
acadoWorkspace.A[9293] = acadoWorkspace.evHu[185];
acadoWorkspace.A[9392] = acadoWorkspace.evHu[186];
acadoWorkspace.A[9393] = acadoWorkspace.evHu[187];
acadoWorkspace.A[9494] = acadoWorkspace.evHu[188];
acadoWorkspace.A[9495] = acadoWorkspace.evHu[189];
acadoWorkspace.A[9594] = acadoWorkspace.evHu[190];
acadoWorkspace.A[9595] = acadoWorkspace.evHu[191];
acadoWorkspace.A[9696] = acadoWorkspace.evHu[192];
acadoWorkspace.A[9697] = acadoWorkspace.evHu[193];
acadoWorkspace.A[9796] = acadoWorkspace.evHu[194];
acadoWorkspace.A[9797] = acadoWorkspace.evHu[195];
acadoWorkspace.A[9898] = acadoWorkspace.evHu[196];
acadoWorkspace.A[9899] = acadoWorkspace.evHu[197];
acadoWorkspace.A[9998] = acadoWorkspace.evHu[198];
acadoWorkspace.A[9999] = acadoWorkspace.evHu[199];
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
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - acadoWorkspace.evH[59];
acadoWorkspace.lbA[60] = acadoVariables.lbAValues[60] - acadoWorkspace.evH[60];
acadoWorkspace.lbA[61] = acadoVariables.lbAValues[61] - acadoWorkspace.evH[61];
acadoWorkspace.lbA[62] = acadoVariables.lbAValues[62] - acadoWorkspace.evH[62];
acadoWorkspace.lbA[63] = acadoVariables.lbAValues[63] - acadoWorkspace.evH[63];
acadoWorkspace.lbA[64] = acadoVariables.lbAValues[64] - acadoWorkspace.evH[64];
acadoWorkspace.lbA[65] = acadoVariables.lbAValues[65] - acadoWorkspace.evH[65];
acadoWorkspace.lbA[66] = acadoVariables.lbAValues[66] - acadoWorkspace.evH[66];
acadoWorkspace.lbA[67] = acadoVariables.lbAValues[67] - acadoWorkspace.evH[67];
acadoWorkspace.lbA[68] = acadoVariables.lbAValues[68] - acadoWorkspace.evH[68];
acadoWorkspace.lbA[69] = acadoVariables.lbAValues[69] - acadoWorkspace.evH[69];
acadoWorkspace.lbA[70] = acadoVariables.lbAValues[70] - acadoWorkspace.evH[70];
acadoWorkspace.lbA[71] = acadoVariables.lbAValues[71] - acadoWorkspace.evH[71];
acadoWorkspace.lbA[72] = acadoVariables.lbAValues[72] - acadoWorkspace.evH[72];
acadoWorkspace.lbA[73] = acadoVariables.lbAValues[73] - acadoWorkspace.evH[73];
acadoWorkspace.lbA[74] = acadoVariables.lbAValues[74] - acadoWorkspace.evH[74];
acadoWorkspace.lbA[75] = acadoVariables.lbAValues[75] - acadoWorkspace.evH[75];
acadoWorkspace.lbA[76] = acadoVariables.lbAValues[76] - acadoWorkspace.evH[76];
acadoWorkspace.lbA[77] = acadoVariables.lbAValues[77] - acadoWorkspace.evH[77];
acadoWorkspace.lbA[78] = acadoVariables.lbAValues[78] - acadoWorkspace.evH[78];
acadoWorkspace.lbA[79] = acadoVariables.lbAValues[79] - acadoWorkspace.evH[79];
acadoWorkspace.lbA[80] = acadoVariables.lbAValues[80] - acadoWorkspace.evH[80];
acadoWorkspace.lbA[81] = acadoVariables.lbAValues[81] - acadoWorkspace.evH[81];
acadoWorkspace.lbA[82] = acadoVariables.lbAValues[82] - acadoWorkspace.evH[82];
acadoWorkspace.lbA[83] = acadoVariables.lbAValues[83] - acadoWorkspace.evH[83];
acadoWorkspace.lbA[84] = acadoVariables.lbAValues[84] - acadoWorkspace.evH[84];
acadoWorkspace.lbA[85] = acadoVariables.lbAValues[85] - acadoWorkspace.evH[85];
acadoWorkspace.lbA[86] = acadoVariables.lbAValues[86] - acadoWorkspace.evH[86];
acadoWorkspace.lbA[87] = acadoVariables.lbAValues[87] - acadoWorkspace.evH[87];
acadoWorkspace.lbA[88] = acadoVariables.lbAValues[88] - acadoWorkspace.evH[88];
acadoWorkspace.lbA[89] = acadoVariables.lbAValues[89] - acadoWorkspace.evH[89];
acadoWorkspace.lbA[90] = acadoVariables.lbAValues[90] - acadoWorkspace.evH[90];
acadoWorkspace.lbA[91] = acadoVariables.lbAValues[91] - acadoWorkspace.evH[91];
acadoWorkspace.lbA[92] = acadoVariables.lbAValues[92] - acadoWorkspace.evH[92];
acadoWorkspace.lbA[93] = acadoVariables.lbAValues[93] - acadoWorkspace.evH[93];
acadoWorkspace.lbA[94] = acadoVariables.lbAValues[94] - acadoWorkspace.evH[94];
acadoWorkspace.lbA[95] = acadoVariables.lbAValues[95] - acadoWorkspace.evH[95];
acadoWorkspace.lbA[96] = acadoVariables.lbAValues[96] - acadoWorkspace.evH[96];
acadoWorkspace.lbA[97] = acadoVariables.lbAValues[97] - acadoWorkspace.evH[97];
acadoWorkspace.lbA[98] = acadoVariables.lbAValues[98] - acadoWorkspace.evH[98];
acadoWorkspace.lbA[99] = acadoVariables.lbAValues[99] - acadoWorkspace.evH[99];

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
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - acadoWorkspace.evH[59];
acadoWorkspace.ubA[60] = acadoVariables.ubAValues[60] - acadoWorkspace.evH[60];
acadoWorkspace.ubA[61] = acadoVariables.ubAValues[61] - acadoWorkspace.evH[61];
acadoWorkspace.ubA[62] = acadoVariables.ubAValues[62] - acadoWorkspace.evH[62];
acadoWorkspace.ubA[63] = acadoVariables.ubAValues[63] - acadoWorkspace.evH[63];
acadoWorkspace.ubA[64] = acadoVariables.ubAValues[64] - acadoWorkspace.evH[64];
acadoWorkspace.ubA[65] = acadoVariables.ubAValues[65] - acadoWorkspace.evH[65];
acadoWorkspace.ubA[66] = acadoVariables.ubAValues[66] - acadoWorkspace.evH[66];
acadoWorkspace.ubA[67] = acadoVariables.ubAValues[67] - acadoWorkspace.evH[67];
acadoWorkspace.ubA[68] = acadoVariables.ubAValues[68] - acadoWorkspace.evH[68];
acadoWorkspace.ubA[69] = acadoVariables.ubAValues[69] - acadoWorkspace.evH[69];
acadoWorkspace.ubA[70] = acadoVariables.ubAValues[70] - acadoWorkspace.evH[70];
acadoWorkspace.ubA[71] = acadoVariables.ubAValues[71] - acadoWorkspace.evH[71];
acadoWorkspace.ubA[72] = acadoVariables.ubAValues[72] - acadoWorkspace.evH[72];
acadoWorkspace.ubA[73] = acadoVariables.ubAValues[73] - acadoWorkspace.evH[73];
acadoWorkspace.ubA[74] = acadoVariables.ubAValues[74] - acadoWorkspace.evH[74];
acadoWorkspace.ubA[75] = acadoVariables.ubAValues[75] - acadoWorkspace.evH[75];
acadoWorkspace.ubA[76] = acadoVariables.ubAValues[76] - acadoWorkspace.evH[76];
acadoWorkspace.ubA[77] = acadoVariables.ubAValues[77] - acadoWorkspace.evH[77];
acadoWorkspace.ubA[78] = acadoVariables.ubAValues[78] - acadoWorkspace.evH[78];
acadoWorkspace.ubA[79] = acadoVariables.ubAValues[79] - acadoWorkspace.evH[79];
acadoWorkspace.ubA[80] = acadoVariables.ubAValues[80] - acadoWorkspace.evH[80];
acadoWorkspace.ubA[81] = acadoVariables.ubAValues[81] - acadoWorkspace.evH[81];
acadoWorkspace.ubA[82] = acadoVariables.ubAValues[82] - acadoWorkspace.evH[82];
acadoWorkspace.ubA[83] = acadoVariables.ubAValues[83] - acadoWorkspace.evH[83];
acadoWorkspace.ubA[84] = acadoVariables.ubAValues[84] - acadoWorkspace.evH[84];
acadoWorkspace.ubA[85] = acadoVariables.ubAValues[85] - acadoWorkspace.evH[85];
acadoWorkspace.ubA[86] = acadoVariables.ubAValues[86] - acadoWorkspace.evH[86];
acadoWorkspace.ubA[87] = acadoVariables.ubAValues[87] - acadoWorkspace.evH[87];
acadoWorkspace.ubA[88] = acadoVariables.ubAValues[88] - acadoWorkspace.evH[88];
acadoWorkspace.ubA[89] = acadoVariables.ubAValues[89] - acadoWorkspace.evH[89];
acadoWorkspace.ubA[90] = acadoVariables.ubAValues[90] - acadoWorkspace.evH[90];
acadoWorkspace.ubA[91] = acadoVariables.ubAValues[91] - acadoWorkspace.evH[91];
acadoWorkspace.ubA[92] = acadoVariables.ubAValues[92] - acadoWorkspace.evH[92];
acadoWorkspace.ubA[93] = acadoVariables.ubAValues[93] - acadoWorkspace.evH[93];
acadoWorkspace.ubA[94] = acadoVariables.ubAValues[94] - acadoWorkspace.evH[94];
acadoWorkspace.ubA[95] = acadoVariables.ubAValues[95] - acadoWorkspace.evH[95];
acadoWorkspace.ubA[96] = acadoVariables.ubAValues[96] - acadoWorkspace.evH[96];
acadoWorkspace.ubA[97] = acadoVariables.ubAValues[97] - acadoWorkspace.evH[97];
acadoWorkspace.ubA[98] = acadoVariables.ubAValues[98] - acadoWorkspace.evH[98];
acadoWorkspace.ubA[99] = acadoVariables.ubAValues[99] - acadoWorkspace.evH[99];

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
acado_macASbar( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[152] + acadoWorkspace.QDy[150];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[152] + acadoWorkspace.QDy[151];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[150] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[151] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[152] + acadoWorkspace.QDy[152];
acado_macBTw1( &(acadoWorkspace.evGu[ 294 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 98 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 294 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.g[ 98 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 441 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 147 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 288 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 282 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 94 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 282 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.g[ 94 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 423 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 141 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 276 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 92 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 276 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.g[ 92 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 414 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 138 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 270 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 264 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 88 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 264 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.g[ 88 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 258 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 86 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 258 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.g[ 86 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 387 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 129 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 252 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 246 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 82 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 246 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.g[ 82 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 369 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 123 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 80 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.g[ 80 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 234 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 228 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 228 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 342 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 222 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 74 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 222 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.g[ 74 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 333 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 111 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 210 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 70 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 210 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.g[ 70 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 315 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 105 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 204 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 204 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 306 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 198 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 192 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 186 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 62 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 186 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.g[ 62 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 279 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 93 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 180 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 174 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 58 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 174 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.g[ 58 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 261 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 87 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 168 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 162 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 156 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 150 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 50 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 150 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.g[ 50 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 75 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
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


acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 6 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.lbA[ 38 ]), &(acadoWorkspace.ubA[ 38 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 126 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.lbA[ 42 ]), &(acadoWorkspace.ubA[ 42 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.lbA[ 44 ]), &(acadoWorkspace.ubA[ 44 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 138 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.lbA[ 48 ]), &(acadoWorkspace.ubA[ 48 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.lbA[ 50 ]), &(acadoWorkspace.ubA[ 50 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.lbA[ 52 ]), &(acadoWorkspace.ubA[ 52 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 162 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.lbA[ 54 ]), &(acadoWorkspace.ubA[ 54 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.lbA[ 56 ]), &(acadoWorkspace.ubA[ 56 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 174 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.lbA[ 58 ]), &(acadoWorkspace.ubA[ 58 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.lbA[ 60 ]), &(acadoWorkspace.ubA[ 60 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 186 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.lbA[ 62 ]), &(acadoWorkspace.ubA[ 62 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 198 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.lbA[ 66 ]), &(acadoWorkspace.ubA[ 66 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.lbA[ 68 ]), &(acadoWorkspace.ubA[ 68 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.lbA[ 70 ]), &(acadoWorkspace.ubA[ 70 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 222 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.lbA[ 74 ]), &(acadoWorkspace.ubA[ 74 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.lbA[ 76 ]), &(acadoWorkspace.ubA[ 76 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 234 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.lbA[ 78 ]), &(acadoWorkspace.ubA[ 78 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 246 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.lbA[ 82 ]), &(acadoWorkspace.ubA[ 82 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 258 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.lbA[ 86 ]), &(acadoWorkspace.ubA[ 86 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 264 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.lbA[ 88 ]), &(acadoWorkspace.ubA[ 88 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.lbA[ 90 ]), &(acadoWorkspace.ubA[ 90 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 276 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.lbA[ 92 ]), &(acadoWorkspace.ubA[ 92 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 282 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.lbA[ 94 ]), &(acadoWorkspace.ubA[ 94 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 294 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.lbA[ 98 ]), &(acadoWorkspace.ubA[ 98 ]) );

}

void acado_expand(  )
{
int lRun1;
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
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
for (lRun1 = 0; lRun1 < 150; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];

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
acado_expansionStep( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.evGu[ 150 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.evGu[ 174 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.evGu[ 186 ]), &(acadoWorkspace.x[ 62 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.evGu[ 198 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.evGu[ 210 ]), &(acadoWorkspace.x[ 70 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.evGu[ 222 ]), &(acadoWorkspace.x[ 74 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.evGu[ 234 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.evGu[ 246 ]), &(acadoWorkspace.x[ 82 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.evGu[ 252 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.evGu[ 258 ]), &(acadoWorkspace.x[ 86 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGu[ 264 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.evGu[ 276 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.evGu[ 282 ]), &(acadoWorkspace.x[ 94 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.evGu[ 294 ]), &(acadoWorkspace.x[ 98 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );
for (lRun1 = 0; lRun1 < 153; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

acadoVariables.mu[147] = 0.0000000000000000e+00;
acadoVariables.mu[148] = 0.0000000000000000e+00;
acadoVariables.mu[149] = 0.0000000000000000e+00;
acadoVariables.mu[147] += + acadoWorkspace.sbar[150]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[151]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[152]*acadoWorkspace.QN1[6];
acadoVariables.mu[148] += + acadoWorkspace.sbar[150]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[151]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[152]*acadoWorkspace.QN1[7];
acadoVariables.mu[149] += + acadoWorkspace.sbar[150]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[151]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[152]*acadoWorkspace.QN1[8];
acadoVariables.mu[147] += acadoWorkspace.QDy[150];
acadoVariables.mu[148] += acadoWorkspace.QDy[151];
acadoVariables.mu[149] += acadoWorkspace.QDy[152];
acadoVariables.mu[144] = 0.0000000000000000e+00;
acadoVariables.mu[144] -= + acadoWorkspace.y[198]*acadoWorkspace.evHx[294] + acadoWorkspace.y[199]*acadoWorkspace.evHx[297];
acadoVariables.mu[145] = 0.0000000000000000e+00;
acadoVariables.mu[145] -= + acadoWorkspace.y[198]*acadoWorkspace.evHx[295] + acadoWorkspace.y[199]*acadoWorkspace.evHx[298];
acadoVariables.mu[146] = 0.0000000000000000e+00;
acadoVariables.mu[146] -= + acadoWorkspace.y[198]*acadoWorkspace.evHx[296] + acadoWorkspace.y[199]*acadoWorkspace.evHx[299];
acado_expansionStep2( &(acadoWorkspace.QDy[ 147 ]), &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.S1[ 294 ]), &(acadoWorkspace.x[ 98 ]), &(acadoWorkspace.evGx[ 441 ]), &(acadoVariables.mu[ 144 ]), &(acadoVariables.mu[ 147 ]) );
acadoVariables.mu[141] = 0.0000000000000000e+00;
acadoVariables.mu[141] -= + acadoWorkspace.y[196]*acadoWorkspace.evHx[288] + acadoWorkspace.y[197]*acadoWorkspace.evHx[291];
acadoVariables.mu[142] = 0.0000000000000000e+00;
acadoVariables.mu[142] -= + acadoWorkspace.y[196]*acadoWorkspace.evHx[289] + acadoWorkspace.y[197]*acadoWorkspace.evHx[292];
acadoVariables.mu[143] = 0.0000000000000000e+00;
acadoVariables.mu[143] -= + acadoWorkspace.y[196]*acadoWorkspace.evHx[290] + acadoWorkspace.y[197]*acadoWorkspace.evHx[293];
acado_expansionStep2( &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.S1[ 288 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoVariables.mu[ 141 ]), &(acadoVariables.mu[ 144 ]) );
acadoVariables.mu[138] = 0.0000000000000000e+00;
acadoVariables.mu[138] -= + acadoWorkspace.y[194]*acadoWorkspace.evHx[282] + acadoWorkspace.y[195]*acadoWorkspace.evHx[285];
acadoVariables.mu[139] = 0.0000000000000000e+00;
acadoVariables.mu[139] -= + acadoWorkspace.y[194]*acadoWorkspace.evHx[283] + acadoWorkspace.y[195]*acadoWorkspace.evHx[286];
acadoVariables.mu[140] = 0.0000000000000000e+00;
acadoVariables.mu[140] -= + acadoWorkspace.y[194]*acadoWorkspace.evHx[284] + acadoWorkspace.y[195]*acadoWorkspace.evHx[287];
acado_expansionStep2( &(acadoWorkspace.QDy[ 141 ]), &(acadoWorkspace.Q1[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.S1[ 282 ]), &(acadoWorkspace.x[ 94 ]), &(acadoWorkspace.evGx[ 423 ]), &(acadoVariables.mu[ 138 ]), &(acadoVariables.mu[ 141 ]) );
acadoVariables.mu[135] = 0.0000000000000000e+00;
acadoVariables.mu[135] -= + acadoWorkspace.y[192]*acadoWorkspace.evHx[276] + acadoWorkspace.y[193]*acadoWorkspace.evHx[279];
acadoVariables.mu[136] = 0.0000000000000000e+00;
acadoVariables.mu[136] -= + acadoWorkspace.y[192]*acadoWorkspace.evHx[277] + acadoWorkspace.y[193]*acadoWorkspace.evHx[280];
acadoVariables.mu[137] = 0.0000000000000000e+00;
acadoVariables.mu[137] -= + acadoWorkspace.y[192]*acadoWorkspace.evHx[278] + acadoWorkspace.y[193]*acadoWorkspace.evHx[281];
acado_expansionStep2( &(acadoWorkspace.QDy[ 138 ]), &(acadoWorkspace.Q1[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.S1[ 276 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.evGx[ 414 ]), &(acadoVariables.mu[ 135 ]), &(acadoVariables.mu[ 138 ]) );
acadoVariables.mu[132] = 0.0000000000000000e+00;
acadoVariables.mu[132] -= + acadoWorkspace.y[190]*acadoWorkspace.evHx[270] + acadoWorkspace.y[191]*acadoWorkspace.evHx[273];
acadoVariables.mu[133] = 0.0000000000000000e+00;
acadoVariables.mu[133] -= + acadoWorkspace.y[190]*acadoWorkspace.evHx[271] + acadoWorkspace.y[191]*acadoWorkspace.evHx[274];
acadoVariables.mu[134] = 0.0000000000000000e+00;
acadoVariables.mu[134] -= + acadoWorkspace.y[190]*acadoWorkspace.evHx[272] + acadoWorkspace.y[191]*acadoWorkspace.evHx[275];
acado_expansionStep2( &(acadoWorkspace.QDy[ 135 ]), &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.S1[ 270 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoVariables.mu[ 132 ]), &(acadoVariables.mu[ 135 ]) );
acadoVariables.mu[129] = 0.0000000000000000e+00;
acadoVariables.mu[129] -= + acadoWorkspace.y[188]*acadoWorkspace.evHx[264] + acadoWorkspace.y[189]*acadoWorkspace.evHx[267];
acadoVariables.mu[130] = 0.0000000000000000e+00;
acadoVariables.mu[130] -= + acadoWorkspace.y[188]*acadoWorkspace.evHx[265] + acadoWorkspace.y[189]*acadoWorkspace.evHx[268];
acadoVariables.mu[131] = 0.0000000000000000e+00;
acadoVariables.mu[131] -= + acadoWorkspace.y[188]*acadoWorkspace.evHx[266] + acadoWorkspace.y[189]*acadoWorkspace.evHx[269];
acado_expansionStep2( &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.S1[ 264 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoVariables.mu[ 129 ]), &(acadoVariables.mu[ 132 ]) );
acadoVariables.mu[126] = 0.0000000000000000e+00;
acadoVariables.mu[126] -= + acadoWorkspace.y[186]*acadoWorkspace.evHx[258] + acadoWorkspace.y[187]*acadoWorkspace.evHx[261];
acadoVariables.mu[127] = 0.0000000000000000e+00;
acadoVariables.mu[127] -= + acadoWorkspace.y[186]*acadoWorkspace.evHx[259] + acadoWorkspace.y[187]*acadoWorkspace.evHx[262];
acadoVariables.mu[128] = 0.0000000000000000e+00;
acadoVariables.mu[128] -= + acadoWorkspace.y[186]*acadoWorkspace.evHx[260] + acadoWorkspace.y[187]*acadoWorkspace.evHx[263];
acado_expansionStep2( &(acadoWorkspace.QDy[ 129 ]), &(acadoWorkspace.Q1[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.S1[ 258 ]), &(acadoWorkspace.x[ 86 ]), &(acadoWorkspace.evGx[ 387 ]), &(acadoVariables.mu[ 126 ]), &(acadoVariables.mu[ 129 ]) );
acadoVariables.mu[123] = 0.0000000000000000e+00;
acadoVariables.mu[123] -= + acadoWorkspace.y[184]*acadoWorkspace.evHx[252] + acadoWorkspace.y[185]*acadoWorkspace.evHx[255];
acadoVariables.mu[124] = 0.0000000000000000e+00;
acadoVariables.mu[124] -= + acadoWorkspace.y[184]*acadoWorkspace.evHx[253] + acadoWorkspace.y[185]*acadoWorkspace.evHx[256];
acadoVariables.mu[125] = 0.0000000000000000e+00;
acadoVariables.mu[125] -= + acadoWorkspace.y[184]*acadoWorkspace.evHx[254] + acadoWorkspace.y[185]*acadoWorkspace.evHx[257];
acado_expansionStep2( &(acadoWorkspace.QDy[ 126 ]), &(acadoWorkspace.Q1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.S1[ 252 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.evGx[ 378 ]), &(acadoVariables.mu[ 123 ]), &(acadoVariables.mu[ 126 ]) );
acadoVariables.mu[120] = 0.0000000000000000e+00;
acadoVariables.mu[120] -= + acadoWorkspace.y[182]*acadoWorkspace.evHx[246] + acadoWorkspace.y[183]*acadoWorkspace.evHx[249];
acadoVariables.mu[121] = 0.0000000000000000e+00;
acadoVariables.mu[121] -= + acadoWorkspace.y[182]*acadoWorkspace.evHx[247] + acadoWorkspace.y[183]*acadoWorkspace.evHx[250];
acadoVariables.mu[122] = 0.0000000000000000e+00;
acadoVariables.mu[122] -= + acadoWorkspace.y[182]*acadoWorkspace.evHx[248] + acadoWorkspace.y[183]*acadoWorkspace.evHx[251];
acado_expansionStep2( &(acadoWorkspace.QDy[ 123 ]), &(acadoWorkspace.Q1[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.S1[ 246 ]), &(acadoWorkspace.x[ 82 ]), &(acadoWorkspace.evGx[ 369 ]), &(acadoVariables.mu[ 120 ]), &(acadoVariables.mu[ 123 ]) );
acadoVariables.mu[117] = 0.0000000000000000e+00;
acadoVariables.mu[117] -= + acadoWorkspace.y[180]*acadoWorkspace.evHx[240] + acadoWorkspace.y[181]*acadoWorkspace.evHx[243];
acadoVariables.mu[118] = 0.0000000000000000e+00;
acadoVariables.mu[118] -= + acadoWorkspace.y[180]*acadoWorkspace.evHx[241] + acadoWorkspace.y[181]*acadoWorkspace.evHx[244];
acadoVariables.mu[119] = 0.0000000000000000e+00;
acadoVariables.mu[119] -= + acadoWorkspace.y[180]*acadoWorkspace.evHx[242] + acadoWorkspace.y[181]*acadoWorkspace.evHx[245];
acado_expansionStep2( &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.S1[ 240 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoVariables.mu[ 117 ]), &(acadoVariables.mu[ 120 ]) );
acadoVariables.mu[114] = 0.0000000000000000e+00;
acadoVariables.mu[114] -= + acadoWorkspace.y[178]*acadoWorkspace.evHx[234] + acadoWorkspace.y[179]*acadoWorkspace.evHx[237];
acadoVariables.mu[115] = 0.0000000000000000e+00;
acadoVariables.mu[115] -= + acadoWorkspace.y[178]*acadoWorkspace.evHx[235] + acadoWorkspace.y[179]*acadoWorkspace.evHx[238];
acadoVariables.mu[116] = 0.0000000000000000e+00;
acadoVariables.mu[116] -= + acadoWorkspace.y[178]*acadoWorkspace.evHx[236] + acadoWorkspace.y[179]*acadoWorkspace.evHx[239];
acado_expansionStep2( &(acadoWorkspace.QDy[ 117 ]), &(acadoWorkspace.Q1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.S1[ 234 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.evGx[ 351 ]), &(acadoVariables.mu[ 114 ]), &(acadoVariables.mu[ 117 ]) );
acadoVariables.mu[111] = 0.0000000000000000e+00;
acadoVariables.mu[111] -= + acadoWorkspace.y[176]*acadoWorkspace.evHx[228] + acadoWorkspace.y[177]*acadoWorkspace.evHx[231];
acadoVariables.mu[112] = 0.0000000000000000e+00;
acadoVariables.mu[112] -= + acadoWorkspace.y[176]*acadoWorkspace.evHx[229] + acadoWorkspace.y[177]*acadoWorkspace.evHx[232];
acadoVariables.mu[113] = 0.0000000000000000e+00;
acadoVariables.mu[113] -= + acadoWorkspace.y[176]*acadoWorkspace.evHx[230] + acadoWorkspace.y[177]*acadoWorkspace.evHx[233];
acado_expansionStep2( &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.Q1[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.S1[ 228 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.evGx[ 342 ]), &(acadoVariables.mu[ 111 ]), &(acadoVariables.mu[ 114 ]) );
acadoVariables.mu[108] = 0.0000000000000000e+00;
acadoVariables.mu[108] -= + acadoWorkspace.y[174]*acadoWorkspace.evHx[222] + acadoWorkspace.y[175]*acadoWorkspace.evHx[225];
acadoVariables.mu[109] = 0.0000000000000000e+00;
acadoVariables.mu[109] -= + acadoWorkspace.y[174]*acadoWorkspace.evHx[223] + acadoWorkspace.y[175]*acadoWorkspace.evHx[226];
acadoVariables.mu[110] = 0.0000000000000000e+00;
acadoVariables.mu[110] -= + acadoWorkspace.y[174]*acadoWorkspace.evHx[224] + acadoWorkspace.y[175]*acadoWorkspace.evHx[227];
acado_expansionStep2( &(acadoWorkspace.QDy[ 111 ]), &(acadoWorkspace.Q1[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.S1[ 222 ]), &(acadoWorkspace.x[ 74 ]), &(acadoWorkspace.evGx[ 333 ]), &(acadoVariables.mu[ 108 ]), &(acadoVariables.mu[ 111 ]) );
acadoVariables.mu[105] = 0.0000000000000000e+00;
acadoVariables.mu[105] -= + acadoWorkspace.y[172]*acadoWorkspace.evHx[216] + acadoWorkspace.y[173]*acadoWorkspace.evHx[219];
acadoVariables.mu[106] = 0.0000000000000000e+00;
acadoVariables.mu[106] -= + acadoWorkspace.y[172]*acadoWorkspace.evHx[217] + acadoWorkspace.y[173]*acadoWorkspace.evHx[220];
acadoVariables.mu[107] = 0.0000000000000000e+00;
acadoVariables.mu[107] -= + acadoWorkspace.y[172]*acadoWorkspace.evHx[218] + acadoWorkspace.y[173]*acadoWorkspace.evHx[221];
acado_expansionStep2( &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoVariables.mu[ 105 ]), &(acadoVariables.mu[ 108 ]) );
acadoVariables.mu[102] = 0.0000000000000000e+00;
acadoVariables.mu[102] -= + acadoWorkspace.y[170]*acadoWorkspace.evHx[210] + acadoWorkspace.y[171]*acadoWorkspace.evHx[213];
acadoVariables.mu[103] = 0.0000000000000000e+00;
acadoVariables.mu[103] -= + acadoWorkspace.y[170]*acadoWorkspace.evHx[211] + acadoWorkspace.y[171]*acadoWorkspace.evHx[214];
acadoVariables.mu[104] = 0.0000000000000000e+00;
acadoVariables.mu[104] -= + acadoWorkspace.y[170]*acadoWorkspace.evHx[212] + acadoWorkspace.y[171]*acadoWorkspace.evHx[215];
acado_expansionStep2( &(acadoWorkspace.QDy[ 105 ]), &(acadoWorkspace.Q1[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.S1[ 210 ]), &(acadoWorkspace.x[ 70 ]), &(acadoWorkspace.evGx[ 315 ]), &(acadoVariables.mu[ 102 ]), &(acadoVariables.mu[ 105 ]) );
acadoVariables.mu[99] = 0.0000000000000000e+00;
acadoVariables.mu[99] -= + acadoWorkspace.y[168]*acadoWorkspace.evHx[204] + acadoWorkspace.y[169]*acadoWorkspace.evHx[207];
acadoVariables.mu[100] = 0.0000000000000000e+00;
acadoVariables.mu[100] -= + acadoWorkspace.y[168]*acadoWorkspace.evHx[205] + acadoWorkspace.y[169]*acadoWorkspace.evHx[208];
acadoVariables.mu[101] = 0.0000000000000000e+00;
acadoVariables.mu[101] -= + acadoWorkspace.y[168]*acadoWorkspace.evHx[206] + acadoWorkspace.y[169]*acadoWorkspace.evHx[209];
acado_expansionStep2( &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.Q1[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.S1[ 204 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.evGx[ 306 ]), &(acadoVariables.mu[ 99 ]), &(acadoVariables.mu[ 102 ]) );
acadoVariables.mu[96] = 0.0000000000000000e+00;
acadoVariables.mu[96] -= + acadoWorkspace.y[166]*acadoWorkspace.evHx[198] + acadoWorkspace.y[167]*acadoWorkspace.evHx[201];
acadoVariables.mu[97] = 0.0000000000000000e+00;
acadoVariables.mu[97] -= + acadoWorkspace.y[166]*acadoWorkspace.evHx[199] + acadoWorkspace.y[167]*acadoWorkspace.evHx[202];
acadoVariables.mu[98] = 0.0000000000000000e+00;
acadoVariables.mu[98] -= + acadoWorkspace.y[166]*acadoWorkspace.evHx[200] + acadoWorkspace.y[167]*acadoWorkspace.evHx[203];
acado_expansionStep2( &(acadoWorkspace.QDy[ 99 ]), &(acadoWorkspace.Q1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.S1[ 198 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.evGx[ 297 ]), &(acadoVariables.mu[ 96 ]), &(acadoVariables.mu[ 99 ]) );
acadoVariables.mu[93] = 0.0000000000000000e+00;
acadoVariables.mu[93] -= + acadoWorkspace.y[164]*acadoWorkspace.evHx[192] + acadoWorkspace.y[165]*acadoWorkspace.evHx[195];
acadoVariables.mu[94] = 0.0000000000000000e+00;
acadoVariables.mu[94] -= + acadoWorkspace.y[164]*acadoWorkspace.evHx[193] + acadoWorkspace.y[165]*acadoWorkspace.evHx[196];
acadoVariables.mu[95] = 0.0000000000000000e+00;
acadoVariables.mu[95] -= + acadoWorkspace.y[164]*acadoWorkspace.evHx[194] + acadoWorkspace.y[165]*acadoWorkspace.evHx[197];
acado_expansionStep2( &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.S1[ 192 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoVariables.mu[ 93 ]), &(acadoVariables.mu[ 96 ]) );
acadoVariables.mu[90] = 0.0000000000000000e+00;
acadoVariables.mu[90] -= + acadoWorkspace.y[162]*acadoWorkspace.evHx[186] + acadoWorkspace.y[163]*acadoWorkspace.evHx[189];
acadoVariables.mu[91] = 0.0000000000000000e+00;
acadoVariables.mu[91] -= + acadoWorkspace.y[162]*acadoWorkspace.evHx[187] + acadoWorkspace.y[163]*acadoWorkspace.evHx[190];
acadoVariables.mu[92] = 0.0000000000000000e+00;
acadoVariables.mu[92] -= + acadoWorkspace.y[162]*acadoWorkspace.evHx[188] + acadoWorkspace.y[163]*acadoWorkspace.evHx[191];
acado_expansionStep2( &(acadoWorkspace.QDy[ 93 ]), &(acadoWorkspace.Q1[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.S1[ 186 ]), &(acadoWorkspace.x[ 62 ]), &(acadoWorkspace.evGx[ 279 ]), &(acadoVariables.mu[ 90 ]), &(acadoVariables.mu[ 93 ]) );
acadoVariables.mu[87] = 0.0000000000000000e+00;
acadoVariables.mu[87] -= + acadoWorkspace.y[160]*acadoWorkspace.evHx[180] + acadoWorkspace.y[161]*acadoWorkspace.evHx[183];
acadoVariables.mu[88] = 0.0000000000000000e+00;
acadoVariables.mu[88] -= + acadoWorkspace.y[160]*acadoWorkspace.evHx[181] + acadoWorkspace.y[161]*acadoWorkspace.evHx[184];
acadoVariables.mu[89] = 0.0000000000000000e+00;
acadoVariables.mu[89] -= + acadoWorkspace.y[160]*acadoWorkspace.evHx[182] + acadoWorkspace.y[161]*acadoWorkspace.evHx[185];
acado_expansionStep2( &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.Q1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.S1[ 180 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.evGx[ 270 ]), &(acadoVariables.mu[ 87 ]), &(acadoVariables.mu[ 90 ]) );
acadoVariables.mu[84] = 0.0000000000000000e+00;
acadoVariables.mu[84] -= + acadoWorkspace.y[158]*acadoWorkspace.evHx[174] + acadoWorkspace.y[159]*acadoWorkspace.evHx[177];
acadoVariables.mu[85] = 0.0000000000000000e+00;
acadoVariables.mu[85] -= + acadoWorkspace.y[158]*acadoWorkspace.evHx[175] + acadoWorkspace.y[159]*acadoWorkspace.evHx[178];
acadoVariables.mu[86] = 0.0000000000000000e+00;
acadoVariables.mu[86] -= + acadoWorkspace.y[158]*acadoWorkspace.evHx[176] + acadoWorkspace.y[159]*acadoWorkspace.evHx[179];
acado_expansionStep2( &(acadoWorkspace.QDy[ 87 ]), &(acadoWorkspace.Q1[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.S1[ 174 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.evGx[ 261 ]), &(acadoVariables.mu[ 84 ]), &(acadoVariables.mu[ 87 ]) );
acadoVariables.mu[81] = 0.0000000000000000e+00;
acadoVariables.mu[81] -= + acadoWorkspace.y[156]*acadoWorkspace.evHx[168] + acadoWorkspace.y[157]*acadoWorkspace.evHx[171];
acadoVariables.mu[82] = 0.0000000000000000e+00;
acadoVariables.mu[82] -= + acadoWorkspace.y[156]*acadoWorkspace.evHx[169] + acadoWorkspace.y[157]*acadoWorkspace.evHx[172];
acadoVariables.mu[83] = 0.0000000000000000e+00;
acadoVariables.mu[83] -= + acadoWorkspace.y[156]*acadoWorkspace.evHx[170] + acadoWorkspace.y[157]*acadoWorkspace.evHx[173];
acado_expansionStep2( &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.S1[ 168 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoVariables.mu[ 81 ]), &(acadoVariables.mu[ 84 ]) );
acadoVariables.mu[78] = 0.0000000000000000e+00;
acadoVariables.mu[78] -= + acadoWorkspace.y[154]*acadoWorkspace.evHx[162] + acadoWorkspace.y[155]*acadoWorkspace.evHx[165];
acadoVariables.mu[79] = 0.0000000000000000e+00;
acadoVariables.mu[79] -= + acadoWorkspace.y[154]*acadoWorkspace.evHx[163] + acadoWorkspace.y[155]*acadoWorkspace.evHx[166];
acadoVariables.mu[80] = 0.0000000000000000e+00;
acadoVariables.mu[80] -= + acadoWorkspace.y[154]*acadoWorkspace.evHx[164] + acadoWorkspace.y[155]*acadoWorkspace.evHx[167];
acado_expansionStep2( &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.S1[ 162 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.evGx[ 243 ]), &(acadoVariables.mu[ 78 ]), &(acadoVariables.mu[ 81 ]) );
acadoVariables.mu[75] = 0.0000000000000000e+00;
acadoVariables.mu[75] -= + acadoWorkspace.y[152]*acadoWorkspace.evHx[156] + acadoWorkspace.y[153]*acadoWorkspace.evHx[159];
acadoVariables.mu[76] = 0.0000000000000000e+00;
acadoVariables.mu[76] -= + acadoWorkspace.y[152]*acadoWorkspace.evHx[157] + acadoWorkspace.y[153]*acadoWorkspace.evHx[160];
acadoVariables.mu[77] = 0.0000000000000000e+00;
acadoVariables.mu[77] -= + acadoWorkspace.y[152]*acadoWorkspace.evHx[158] + acadoWorkspace.y[153]*acadoWorkspace.evHx[161];
acado_expansionStep2( &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.Q1[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.S1[ 156 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.evGx[ 234 ]), &(acadoVariables.mu[ 75 ]), &(acadoVariables.mu[ 78 ]) );
acadoVariables.mu[72] = 0.0000000000000000e+00;
acadoVariables.mu[72] -= + acadoWorkspace.y[150]*acadoWorkspace.evHx[150] + acadoWorkspace.y[151]*acadoWorkspace.evHx[153];
acadoVariables.mu[73] = 0.0000000000000000e+00;
acadoVariables.mu[73] -= + acadoWorkspace.y[150]*acadoWorkspace.evHx[151] + acadoWorkspace.y[151]*acadoWorkspace.evHx[154];
acadoVariables.mu[74] = 0.0000000000000000e+00;
acadoVariables.mu[74] -= + acadoWorkspace.y[150]*acadoWorkspace.evHx[152] + acadoWorkspace.y[151]*acadoWorkspace.evHx[155];
acado_expansionStep2( &(acadoWorkspace.QDy[ 75 ]), &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.S1[ 150 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoVariables.mu[ 72 ]), &(acadoVariables.mu[ 75 ]) );
acadoVariables.mu[69] = 0.0000000000000000e+00;
acadoVariables.mu[69] -= + acadoWorkspace.y[148]*acadoWorkspace.evHx[144] + acadoWorkspace.y[149]*acadoWorkspace.evHx[147];
acadoVariables.mu[70] = 0.0000000000000000e+00;
acadoVariables.mu[70] -= + acadoWorkspace.y[148]*acadoWorkspace.evHx[145] + acadoWorkspace.y[149]*acadoWorkspace.evHx[148];
acadoVariables.mu[71] = 0.0000000000000000e+00;
acadoVariables.mu[71] -= + acadoWorkspace.y[148]*acadoWorkspace.evHx[146] + acadoWorkspace.y[149]*acadoWorkspace.evHx[149];
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 144 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoVariables.mu[ 69 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[66] = 0.0000000000000000e+00;
acadoVariables.mu[66] -= + acadoWorkspace.y[146]*acadoWorkspace.evHx[138] + acadoWorkspace.y[147]*acadoWorkspace.evHx[141];
acadoVariables.mu[67] = 0.0000000000000000e+00;
acadoVariables.mu[67] -= + acadoWorkspace.y[146]*acadoWorkspace.evHx[139] + acadoWorkspace.y[147]*acadoWorkspace.evHx[142];
acadoVariables.mu[68] = 0.0000000000000000e+00;
acadoVariables.mu[68] -= + acadoWorkspace.y[146]*acadoWorkspace.evHx[140] + acadoWorkspace.y[147]*acadoWorkspace.evHx[143];
acado_expansionStep2( &(acadoWorkspace.QDy[ 69 ]), &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.S1[ 138 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.evGx[ 207 ]), &(acadoVariables.mu[ 66 ]), &(acadoVariables.mu[ 69 ]) );
acadoVariables.mu[63] = 0.0000000000000000e+00;
acadoVariables.mu[63] -= + acadoWorkspace.y[144]*acadoWorkspace.evHx[132] + acadoWorkspace.y[145]*acadoWorkspace.evHx[135];
acadoVariables.mu[64] = 0.0000000000000000e+00;
acadoVariables.mu[64] -= + acadoWorkspace.y[144]*acadoWorkspace.evHx[133] + acadoWorkspace.y[145]*acadoWorkspace.evHx[136];
acadoVariables.mu[65] = 0.0000000000000000e+00;
acadoVariables.mu[65] -= + acadoWorkspace.y[144]*acadoWorkspace.evHx[134] + acadoWorkspace.y[145]*acadoWorkspace.evHx[137];
acado_expansionStep2( &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.S1[ 132 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.evGx[ 198 ]), &(acadoVariables.mu[ 63 ]), &(acadoVariables.mu[ 66 ]) );
acadoVariables.mu[60] = 0.0000000000000000e+00;
acadoVariables.mu[60] -= + acadoWorkspace.y[142]*acadoWorkspace.evHx[126] + acadoWorkspace.y[143]*acadoWorkspace.evHx[129];
acadoVariables.mu[61] = 0.0000000000000000e+00;
acadoVariables.mu[61] -= + acadoWorkspace.y[142]*acadoWorkspace.evHx[127] + acadoWorkspace.y[143]*acadoWorkspace.evHx[130];
acadoVariables.mu[62] = 0.0000000000000000e+00;
acadoVariables.mu[62] -= + acadoWorkspace.y[142]*acadoWorkspace.evHx[128] + acadoWorkspace.y[143]*acadoWorkspace.evHx[131];
acado_expansionStep2( &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.S1[ 126 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.evGx[ 189 ]), &(acadoVariables.mu[ 60 ]), &(acadoVariables.mu[ 63 ]) );
acadoVariables.mu[57] = 0.0000000000000000e+00;
acadoVariables.mu[57] -= + acadoWorkspace.y[140]*acadoWorkspace.evHx[120] + acadoWorkspace.y[141]*acadoWorkspace.evHx[123];
acadoVariables.mu[58] = 0.0000000000000000e+00;
acadoVariables.mu[58] -= + acadoWorkspace.y[140]*acadoWorkspace.evHx[121] + acadoWorkspace.y[141]*acadoWorkspace.evHx[124];
acadoVariables.mu[59] = 0.0000000000000000e+00;
acadoVariables.mu[59] -= + acadoWorkspace.y[140]*acadoWorkspace.evHx[122] + acadoWorkspace.y[141]*acadoWorkspace.evHx[125];
acado_expansionStep2( &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.S1[ 120 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoVariables.mu[ 57 ]), &(acadoVariables.mu[ 60 ]) );
acadoVariables.mu[54] = 0.0000000000000000e+00;
acadoVariables.mu[54] -= + acadoWorkspace.y[138]*acadoWorkspace.evHx[114] + acadoWorkspace.y[139]*acadoWorkspace.evHx[117];
acadoVariables.mu[55] = 0.0000000000000000e+00;
acadoVariables.mu[55] -= + acadoWorkspace.y[138]*acadoWorkspace.evHx[115] + acadoWorkspace.y[139]*acadoWorkspace.evHx[118];
acadoVariables.mu[56] = 0.0000000000000000e+00;
acadoVariables.mu[56] -= + acadoWorkspace.y[138]*acadoWorkspace.evHx[116] + acadoWorkspace.y[139]*acadoWorkspace.evHx[119];
acado_expansionStep2( &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.S1[ 114 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoVariables.mu[ 54 ]), &(acadoVariables.mu[ 57 ]) );
acadoVariables.mu[51] = 0.0000000000000000e+00;
acadoVariables.mu[51] -= + acadoWorkspace.y[136]*acadoWorkspace.evHx[108] + acadoWorkspace.y[137]*acadoWorkspace.evHx[111];
acadoVariables.mu[52] = 0.0000000000000000e+00;
acadoVariables.mu[52] -= + acadoWorkspace.y[136]*acadoWorkspace.evHx[109] + acadoWorkspace.y[137]*acadoWorkspace.evHx[112];
acadoVariables.mu[53] = 0.0000000000000000e+00;
acadoVariables.mu[53] -= + acadoWorkspace.y[136]*acadoWorkspace.evHx[110] + acadoWorkspace.y[137]*acadoWorkspace.evHx[113];
acado_expansionStep2( &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoVariables.mu[ 51 ]), &(acadoVariables.mu[ 54 ]) );
acadoVariables.mu[48] = 0.0000000000000000e+00;
acadoVariables.mu[48] -= + acadoWorkspace.y[134]*acadoWorkspace.evHx[102] + acadoWorkspace.y[135]*acadoWorkspace.evHx[105];
acadoVariables.mu[49] = 0.0000000000000000e+00;
acadoVariables.mu[49] -= + acadoWorkspace.y[134]*acadoWorkspace.evHx[103] + acadoWorkspace.y[135]*acadoWorkspace.evHx[106];
acadoVariables.mu[50] = 0.0000000000000000e+00;
acadoVariables.mu[50] -= + acadoWorkspace.y[134]*acadoWorkspace.evHx[104] + acadoWorkspace.y[135]*acadoWorkspace.evHx[107];
acado_expansionStep2( &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.S1[ 102 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoVariables.mu[ 48 ]), &(acadoVariables.mu[ 51 ]) );
acadoVariables.mu[45] = 0.0000000000000000e+00;
acadoVariables.mu[45] -= + acadoWorkspace.y[132]*acadoWorkspace.evHx[96] + acadoWorkspace.y[133]*acadoWorkspace.evHx[99];
acadoVariables.mu[46] = 0.0000000000000000e+00;
acadoVariables.mu[46] -= + acadoWorkspace.y[132]*acadoWorkspace.evHx[97] + acadoWorkspace.y[133]*acadoWorkspace.evHx[100];
acadoVariables.mu[47] = 0.0000000000000000e+00;
acadoVariables.mu[47] -= + acadoWorkspace.y[132]*acadoWorkspace.evHx[98] + acadoWorkspace.y[133]*acadoWorkspace.evHx[101];
acado_expansionStep2( &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.S1[ 96 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoVariables.mu[ 45 ]), &(acadoVariables.mu[ 48 ]) );
acadoVariables.mu[42] = 0.0000000000000000e+00;
acadoVariables.mu[42] -= + acadoWorkspace.y[130]*acadoWorkspace.evHx[90] + acadoWorkspace.y[131]*acadoWorkspace.evHx[93];
acadoVariables.mu[43] = 0.0000000000000000e+00;
acadoVariables.mu[43] -= + acadoWorkspace.y[130]*acadoWorkspace.evHx[91] + acadoWorkspace.y[131]*acadoWorkspace.evHx[94];
acadoVariables.mu[44] = 0.0000000000000000e+00;
acadoVariables.mu[44] -= + acadoWorkspace.y[130]*acadoWorkspace.evHx[92] + acadoWorkspace.y[131]*acadoWorkspace.evHx[95];
acado_expansionStep2( &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.S1[ 90 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoVariables.mu[ 42 ]), &(acadoVariables.mu[ 45 ]) );
acadoVariables.mu[39] = 0.0000000000000000e+00;
acadoVariables.mu[39] -= + acadoWorkspace.y[128]*acadoWorkspace.evHx[84] + acadoWorkspace.y[129]*acadoWorkspace.evHx[87];
acadoVariables.mu[40] = 0.0000000000000000e+00;
acadoVariables.mu[40] -= + acadoWorkspace.y[128]*acadoWorkspace.evHx[85] + acadoWorkspace.y[129]*acadoWorkspace.evHx[88];
acadoVariables.mu[41] = 0.0000000000000000e+00;
acadoVariables.mu[41] -= + acadoWorkspace.y[128]*acadoWorkspace.evHx[86] + acadoWorkspace.y[129]*acadoWorkspace.evHx[89];
acado_expansionStep2( &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.S1[ 84 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoVariables.mu[ 39 ]), &(acadoVariables.mu[ 42 ]) );
acadoVariables.mu[36] = 0.0000000000000000e+00;
acadoVariables.mu[36] -= + acadoWorkspace.y[126]*acadoWorkspace.evHx[78] + acadoWorkspace.y[127]*acadoWorkspace.evHx[81];
acadoVariables.mu[37] = 0.0000000000000000e+00;
acadoVariables.mu[37] -= + acadoWorkspace.y[126]*acadoWorkspace.evHx[79] + acadoWorkspace.y[127]*acadoWorkspace.evHx[82];
acadoVariables.mu[38] = 0.0000000000000000e+00;
acadoVariables.mu[38] -= + acadoWorkspace.y[126]*acadoWorkspace.evHx[80] + acadoWorkspace.y[127]*acadoWorkspace.evHx[83];
acado_expansionStep2( &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.S1[ 78 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoVariables.mu[ 36 ]), &(acadoVariables.mu[ 39 ]) );
acadoVariables.mu[33] = 0.0000000000000000e+00;
acadoVariables.mu[33] -= + acadoWorkspace.y[124]*acadoWorkspace.evHx[72] + acadoWorkspace.y[125]*acadoWorkspace.evHx[75];
acadoVariables.mu[34] = 0.0000000000000000e+00;
acadoVariables.mu[34] -= + acadoWorkspace.y[124]*acadoWorkspace.evHx[73] + acadoWorkspace.y[125]*acadoWorkspace.evHx[76];
acadoVariables.mu[35] = 0.0000000000000000e+00;
acadoVariables.mu[35] -= + acadoWorkspace.y[124]*acadoWorkspace.evHx[74] + acadoWorkspace.y[125]*acadoWorkspace.evHx[77];
acado_expansionStep2( &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.S1[ 72 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoVariables.mu[ 33 ]), &(acadoVariables.mu[ 36 ]) );
acadoVariables.mu[30] = 0.0000000000000000e+00;
acadoVariables.mu[30] -= + acadoWorkspace.y[122]*acadoWorkspace.evHx[66] + acadoWorkspace.y[123]*acadoWorkspace.evHx[69];
acadoVariables.mu[31] = 0.0000000000000000e+00;
acadoVariables.mu[31] -= + acadoWorkspace.y[122]*acadoWorkspace.evHx[67] + acadoWorkspace.y[123]*acadoWorkspace.evHx[70];
acadoVariables.mu[32] = 0.0000000000000000e+00;
acadoVariables.mu[32] -= + acadoWorkspace.y[122]*acadoWorkspace.evHx[68] + acadoWorkspace.y[123]*acadoWorkspace.evHx[71];
acado_expansionStep2( &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.S1[ 66 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoVariables.mu[ 30 ]), &(acadoVariables.mu[ 33 ]) );
acadoVariables.mu[27] = 0.0000000000000000e+00;
acadoVariables.mu[27] -= + acadoWorkspace.y[120]*acadoWorkspace.evHx[60] + acadoWorkspace.y[121]*acadoWorkspace.evHx[63];
acadoVariables.mu[28] = 0.0000000000000000e+00;
acadoVariables.mu[28] -= + acadoWorkspace.y[120]*acadoWorkspace.evHx[61] + acadoWorkspace.y[121]*acadoWorkspace.evHx[64];
acadoVariables.mu[29] = 0.0000000000000000e+00;
acadoVariables.mu[29] -= + acadoWorkspace.y[120]*acadoWorkspace.evHx[62] + acadoWorkspace.y[121]*acadoWorkspace.evHx[65];
acado_expansionStep2( &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.S1[ 60 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoVariables.mu[ 27 ]), &(acadoVariables.mu[ 30 ]) );
acadoVariables.mu[24] = 0.0000000000000000e+00;
acadoVariables.mu[24] -= + acadoWorkspace.y[118]*acadoWorkspace.evHx[54] + acadoWorkspace.y[119]*acadoWorkspace.evHx[57];
acadoVariables.mu[25] = 0.0000000000000000e+00;
acadoVariables.mu[25] -= + acadoWorkspace.y[118]*acadoWorkspace.evHx[55] + acadoWorkspace.y[119]*acadoWorkspace.evHx[58];
acadoVariables.mu[26] = 0.0000000000000000e+00;
acadoVariables.mu[26] -= + acadoWorkspace.y[118]*acadoWorkspace.evHx[56] + acadoWorkspace.y[119]*acadoWorkspace.evHx[59];
acado_expansionStep2( &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoVariables.mu[ 24 ]), &(acadoVariables.mu[ 27 ]) );
acadoVariables.mu[21] = 0.0000000000000000e+00;
acadoVariables.mu[21] -= + acadoWorkspace.y[116]*acadoWorkspace.evHx[48] + acadoWorkspace.y[117]*acadoWorkspace.evHx[51];
acadoVariables.mu[22] = 0.0000000000000000e+00;
acadoVariables.mu[22] -= + acadoWorkspace.y[116]*acadoWorkspace.evHx[49] + acadoWorkspace.y[117]*acadoWorkspace.evHx[52];
acadoVariables.mu[23] = 0.0000000000000000e+00;
acadoVariables.mu[23] -= + acadoWorkspace.y[116]*acadoWorkspace.evHx[50] + acadoWorkspace.y[117]*acadoWorkspace.evHx[53];
acado_expansionStep2( &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.S1[ 48 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoVariables.mu[ 21 ]), &(acadoVariables.mu[ 24 ]) );
acadoVariables.mu[18] = 0.0000000000000000e+00;
acadoVariables.mu[18] -= + acadoWorkspace.y[114]*acadoWorkspace.evHx[42] + acadoWorkspace.y[115]*acadoWorkspace.evHx[45];
acadoVariables.mu[19] = 0.0000000000000000e+00;
acadoVariables.mu[19] -= + acadoWorkspace.y[114]*acadoWorkspace.evHx[43] + acadoWorkspace.y[115]*acadoWorkspace.evHx[46];
acadoVariables.mu[20] = 0.0000000000000000e+00;
acadoVariables.mu[20] -= + acadoWorkspace.y[114]*acadoWorkspace.evHx[44] + acadoWorkspace.y[115]*acadoWorkspace.evHx[47];
acado_expansionStep2( &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.S1[ 42 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoVariables.mu[ 18 ]), &(acadoVariables.mu[ 21 ]) );
acadoVariables.mu[15] = 0.0000000000000000e+00;
acadoVariables.mu[15] -= + acadoWorkspace.y[112]*acadoWorkspace.evHx[36] + acadoWorkspace.y[113]*acadoWorkspace.evHx[39];
acadoVariables.mu[16] = 0.0000000000000000e+00;
acadoVariables.mu[16] -= + acadoWorkspace.y[112]*acadoWorkspace.evHx[37] + acadoWorkspace.y[113]*acadoWorkspace.evHx[40];
acadoVariables.mu[17] = 0.0000000000000000e+00;
acadoVariables.mu[17] -= + acadoWorkspace.y[112]*acadoWorkspace.evHx[38] + acadoWorkspace.y[113]*acadoWorkspace.evHx[41];
acado_expansionStep2( &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.S1[ 36 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoVariables.mu[ 15 ]), &(acadoVariables.mu[ 18 ]) );
acadoVariables.mu[12] = 0.0000000000000000e+00;
acadoVariables.mu[12] -= + acadoWorkspace.y[110]*acadoWorkspace.evHx[30] + acadoWorkspace.y[111]*acadoWorkspace.evHx[33];
acadoVariables.mu[13] = 0.0000000000000000e+00;
acadoVariables.mu[13] -= + acadoWorkspace.y[110]*acadoWorkspace.evHx[31] + acadoWorkspace.y[111]*acadoWorkspace.evHx[34];
acadoVariables.mu[14] = 0.0000000000000000e+00;
acadoVariables.mu[14] -= + acadoWorkspace.y[110]*acadoWorkspace.evHx[32] + acadoWorkspace.y[111]*acadoWorkspace.evHx[35];
acado_expansionStep2( &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.S1[ 30 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoVariables.mu[ 12 ]), &(acadoVariables.mu[ 15 ]) );
acadoVariables.mu[9] = 0.0000000000000000e+00;
acadoVariables.mu[9] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[24] + acadoWorkspace.y[109]*acadoWorkspace.evHx[27];
acadoVariables.mu[10] = 0.0000000000000000e+00;
acadoVariables.mu[10] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[25] + acadoWorkspace.y[109]*acadoWorkspace.evHx[28];
acadoVariables.mu[11] = 0.0000000000000000e+00;
acadoVariables.mu[11] -= + acadoWorkspace.y[108]*acadoWorkspace.evHx[26] + acadoWorkspace.y[109]*acadoWorkspace.evHx[29];
acado_expansionStep2( &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.S1[ 24 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoVariables.mu[ 9 ]), &(acadoVariables.mu[ 12 ]) );
acadoVariables.mu[6] = 0.0000000000000000e+00;
acadoVariables.mu[6] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[18] + acadoWorkspace.y[107]*acadoWorkspace.evHx[21];
acadoVariables.mu[7] = 0.0000000000000000e+00;
acadoVariables.mu[7] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[19] + acadoWorkspace.y[107]*acadoWorkspace.evHx[22];
acadoVariables.mu[8] = 0.0000000000000000e+00;
acadoVariables.mu[8] -= + acadoWorkspace.y[106]*acadoWorkspace.evHx[20] + acadoWorkspace.y[107]*acadoWorkspace.evHx[23];
acado_expansionStep2( &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.S1[ 18 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoVariables.mu[ 6 ]), &(acadoVariables.mu[ 9 ]) );
acadoVariables.mu[3] = 0.0000000000000000e+00;
acadoVariables.mu[3] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[12] + acadoWorkspace.y[105]*acadoWorkspace.evHx[15];
acadoVariables.mu[4] = 0.0000000000000000e+00;
acadoVariables.mu[4] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[13] + acadoWorkspace.y[105]*acadoWorkspace.evHx[16];
acadoVariables.mu[5] = 0.0000000000000000e+00;
acadoVariables.mu[5] -= + acadoWorkspace.y[104]*acadoWorkspace.evHx[14] + acadoWorkspace.y[105]*acadoWorkspace.evHx[17];
acado_expansionStep2( &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.S1[ 12 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoVariables.mu[ 3 ]), &(acadoVariables.mu[ 6 ]) );
acadoVariables.mu[0] = 0.0000000000000000e+00;
acadoVariables.mu[0] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[6] + acadoWorkspace.y[103]*acadoWorkspace.evHx[9];
acadoVariables.mu[1] = 0.0000000000000000e+00;
acadoVariables.mu[1] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[7] + acadoWorkspace.y[103]*acadoWorkspace.evHx[10];
acadoVariables.mu[2] = 0.0000000000000000e+00;
acadoVariables.mu[2] -= + acadoWorkspace.y[102]*acadoWorkspace.evHx[8] + acadoWorkspace.y[103]*acadoWorkspace.evHx[11];
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
acadoVariables.lbValues[50] = -1.0000000000000000e+00;
acadoVariables.lbValues[51] = -1.0000000000000000e+00;
acadoVariables.lbValues[52] = -1.0000000000000000e+00;
acadoVariables.lbValues[53] = -1.0000000000000000e+00;
acadoVariables.lbValues[54] = -1.0000000000000000e+00;
acadoVariables.lbValues[55] = -1.0000000000000000e+00;
acadoVariables.lbValues[56] = -1.0000000000000000e+00;
acadoVariables.lbValues[57] = -1.0000000000000000e+00;
acadoVariables.lbValues[58] = -1.0000000000000000e+00;
acadoVariables.lbValues[59] = -1.0000000000000000e+00;
acadoVariables.lbValues[60] = -1.0000000000000000e+00;
acadoVariables.lbValues[61] = -1.0000000000000000e+00;
acadoVariables.lbValues[62] = -1.0000000000000000e+00;
acadoVariables.lbValues[63] = -1.0000000000000000e+00;
acadoVariables.lbValues[64] = -1.0000000000000000e+00;
acadoVariables.lbValues[65] = -1.0000000000000000e+00;
acadoVariables.lbValues[66] = -1.0000000000000000e+00;
acadoVariables.lbValues[67] = -1.0000000000000000e+00;
acadoVariables.lbValues[68] = -1.0000000000000000e+00;
acadoVariables.lbValues[69] = -1.0000000000000000e+00;
acadoVariables.lbValues[70] = -1.0000000000000000e+00;
acadoVariables.lbValues[71] = -1.0000000000000000e+00;
acadoVariables.lbValues[72] = -1.0000000000000000e+00;
acadoVariables.lbValues[73] = -1.0000000000000000e+00;
acadoVariables.lbValues[74] = -1.0000000000000000e+00;
acadoVariables.lbValues[75] = -1.0000000000000000e+00;
acadoVariables.lbValues[76] = -1.0000000000000000e+00;
acadoVariables.lbValues[77] = -1.0000000000000000e+00;
acadoVariables.lbValues[78] = -1.0000000000000000e+00;
acadoVariables.lbValues[79] = -1.0000000000000000e+00;
acadoVariables.lbValues[80] = -1.0000000000000000e+00;
acadoVariables.lbValues[81] = -1.0000000000000000e+00;
acadoVariables.lbValues[82] = -1.0000000000000000e+00;
acadoVariables.lbValues[83] = -1.0000000000000000e+00;
acadoVariables.lbValues[84] = -1.0000000000000000e+00;
acadoVariables.lbValues[85] = -1.0000000000000000e+00;
acadoVariables.lbValues[86] = -1.0000000000000000e+00;
acadoVariables.lbValues[87] = -1.0000000000000000e+00;
acadoVariables.lbValues[88] = -1.0000000000000000e+00;
acadoVariables.lbValues[89] = -1.0000000000000000e+00;
acadoVariables.lbValues[90] = -1.0000000000000000e+00;
acadoVariables.lbValues[91] = -1.0000000000000000e+00;
acadoVariables.lbValues[92] = -1.0000000000000000e+00;
acadoVariables.lbValues[93] = -1.0000000000000000e+00;
acadoVariables.lbValues[94] = -1.0000000000000000e+00;
acadoVariables.lbValues[95] = -1.0000000000000000e+00;
acadoVariables.lbValues[96] = -1.0000000000000000e+00;
acadoVariables.lbValues[97] = -1.0000000000000000e+00;
acadoVariables.lbValues[98] = -1.0000000000000000e+00;
acadoVariables.lbValues[99] = -1.0000000000000000e+00;
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
acadoVariables.ubValues[50] = 1.0000000000000000e+00;
acadoVariables.ubValues[51] = 1.0000000000000000e+00;
acadoVariables.ubValues[52] = 1.0000000000000000e+00;
acadoVariables.ubValues[53] = 1.0000000000000000e+00;
acadoVariables.ubValues[54] = 1.0000000000000000e+00;
acadoVariables.ubValues[55] = 1.0000000000000000e+00;
acadoVariables.ubValues[56] = 1.0000000000000000e+00;
acadoVariables.ubValues[57] = 1.0000000000000000e+00;
acadoVariables.ubValues[58] = 1.0000000000000000e+00;
acadoVariables.ubValues[59] = 1.0000000000000000e+00;
acadoVariables.ubValues[60] = 1.0000000000000000e+00;
acadoVariables.ubValues[61] = 1.0000000000000000e+00;
acadoVariables.ubValues[62] = 1.0000000000000000e+00;
acadoVariables.ubValues[63] = 1.0000000000000000e+00;
acadoVariables.ubValues[64] = 1.0000000000000000e+00;
acadoVariables.ubValues[65] = 1.0000000000000000e+00;
acadoVariables.ubValues[66] = 1.0000000000000000e+00;
acadoVariables.ubValues[67] = 1.0000000000000000e+00;
acadoVariables.ubValues[68] = 1.0000000000000000e+00;
acadoVariables.ubValues[69] = 1.0000000000000000e+00;
acadoVariables.ubValues[70] = 1.0000000000000000e+00;
acadoVariables.ubValues[71] = 1.0000000000000000e+00;
acadoVariables.ubValues[72] = 1.0000000000000000e+00;
acadoVariables.ubValues[73] = 1.0000000000000000e+00;
acadoVariables.ubValues[74] = 1.0000000000000000e+00;
acadoVariables.ubValues[75] = 1.0000000000000000e+00;
acadoVariables.ubValues[76] = 1.0000000000000000e+00;
acadoVariables.ubValues[77] = 1.0000000000000000e+00;
acadoVariables.ubValues[78] = 1.0000000000000000e+00;
acadoVariables.ubValues[79] = 1.0000000000000000e+00;
acadoVariables.ubValues[80] = 1.0000000000000000e+00;
acadoVariables.ubValues[81] = 1.0000000000000000e+00;
acadoVariables.ubValues[82] = 1.0000000000000000e+00;
acadoVariables.ubValues[83] = 1.0000000000000000e+00;
acadoVariables.ubValues[84] = 1.0000000000000000e+00;
acadoVariables.ubValues[85] = 1.0000000000000000e+00;
acadoVariables.ubValues[86] = 1.0000000000000000e+00;
acadoVariables.ubValues[87] = 1.0000000000000000e+00;
acadoVariables.ubValues[88] = 1.0000000000000000e+00;
acadoVariables.ubValues[89] = 1.0000000000000000e+00;
acadoVariables.ubValues[90] = 1.0000000000000000e+00;
acadoVariables.ubValues[91] = 1.0000000000000000e+00;
acadoVariables.ubValues[92] = 1.0000000000000000e+00;
acadoVariables.ubValues[93] = 1.0000000000000000e+00;
acadoVariables.ubValues[94] = 1.0000000000000000e+00;
acadoVariables.ubValues[95] = 1.0000000000000000e+00;
acadoVariables.ubValues[96] = 1.0000000000000000e+00;
acadoVariables.ubValues[97] = 1.0000000000000000e+00;
acadoVariables.ubValues[98] = 1.0000000000000000e+00;
acadoVariables.ubValues[99] = 1.0000000000000000e+00;
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
acadoVariables.lbAValues[50] = 1.0000000000000000e+00;
acadoVariables.lbAValues[51] = 1.0000000000000000e+00;
acadoVariables.lbAValues[52] = 1.0000000000000000e+00;
acadoVariables.lbAValues[53] = 1.0000000000000000e+00;
acadoVariables.lbAValues[54] = 1.0000000000000000e+00;
acadoVariables.lbAValues[55] = 1.0000000000000000e+00;
acadoVariables.lbAValues[56] = 1.0000000000000000e+00;
acadoVariables.lbAValues[57] = 1.0000000000000000e+00;
acadoVariables.lbAValues[58] = 1.0000000000000000e+00;
acadoVariables.lbAValues[59] = 1.0000000000000000e+00;
acadoVariables.lbAValues[60] = 1.0000000000000000e+00;
acadoVariables.lbAValues[61] = 1.0000000000000000e+00;
acadoVariables.lbAValues[62] = 1.0000000000000000e+00;
acadoVariables.lbAValues[63] = 1.0000000000000000e+00;
acadoVariables.lbAValues[64] = 1.0000000000000000e+00;
acadoVariables.lbAValues[65] = 1.0000000000000000e+00;
acadoVariables.lbAValues[66] = 1.0000000000000000e+00;
acadoVariables.lbAValues[67] = 1.0000000000000000e+00;
acadoVariables.lbAValues[68] = 1.0000000000000000e+00;
acadoVariables.lbAValues[69] = 1.0000000000000000e+00;
acadoVariables.lbAValues[70] = 1.0000000000000000e+00;
acadoVariables.lbAValues[71] = 1.0000000000000000e+00;
acadoVariables.lbAValues[72] = 1.0000000000000000e+00;
acadoVariables.lbAValues[73] = 1.0000000000000000e+00;
acadoVariables.lbAValues[74] = 1.0000000000000000e+00;
acadoVariables.lbAValues[75] = 1.0000000000000000e+00;
acadoVariables.lbAValues[76] = 1.0000000000000000e+00;
acadoVariables.lbAValues[77] = 1.0000000000000000e+00;
acadoVariables.lbAValues[78] = 1.0000000000000000e+00;
acadoVariables.lbAValues[79] = 1.0000000000000000e+00;
acadoVariables.lbAValues[80] = 1.0000000000000000e+00;
acadoVariables.lbAValues[81] = 1.0000000000000000e+00;
acadoVariables.lbAValues[82] = 1.0000000000000000e+00;
acadoVariables.lbAValues[83] = 1.0000000000000000e+00;
acadoVariables.lbAValues[84] = 1.0000000000000000e+00;
acadoVariables.lbAValues[85] = 1.0000000000000000e+00;
acadoVariables.lbAValues[86] = 1.0000000000000000e+00;
acadoVariables.lbAValues[87] = 1.0000000000000000e+00;
acadoVariables.lbAValues[88] = 1.0000000000000000e+00;
acadoVariables.lbAValues[89] = 1.0000000000000000e+00;
acadoVariables.lbAValues[90] = 1.0000000000000000e+00;
acadoVariables.lbAValues[91] = 1.0000000000000000e+00;
acadoVariables.lbAValues[92] = 1.0000000000000000e+00;
acadoVariables.lbAValues[93] = 1.0000000000000000e+00;
acadoVariables.lbAValues[94] = 1.0000000000000000e+00;
acadoVariables.lbAValues[95] = 1.0000000000000000e+00;
acadoVariables.lbAValues[96] = 1.0000000000000000e+00;
acadoVariables.lbAValues[97] = 1.0000000000000000e+00;
acadoVariables.lbAValues[98] = 1.0000000000000000e+00;
acadoVariables.lbAValues[99] = 1.0000000000000000e+00;
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
acadoVariables.ubAValues[50] = 1.0000000000000000e+12;
acadoVariables.ubAValues[51] = 1.0000000000000000e+12;
acadoVariables.ubAValues[52] = 1.0000000000000000e+12;
acadoVariables.ubAValues[53] = 1.0000000000000000e+12;
acadoVariables.ubAValues[54] = 1.0000000000000000e+12;
acadoVariables.ubAValues[55] = 1.0000000000000000e+12;
acadoVariables.ubAValues[56] = 1.0000000000000000e+12;
acadoVariables.ubAValues[57] = 1.0000000000000000e+12;
acadoVariables.ubAValues[58] = 1.0000000000000000e+12;
acadoVariables.ubAValues[59] = 1.0000000000000000e+12;
acadoVariables.ubAValues[60] = 1.0000000000000000e+12;
acadoVariables.ubAValues[61] = 1.0000000000000000e+12;
acadoVariables.ubAValues[62] = 1.0000000000000000e+12;
acadoVariables.ubAValues[63] = 1.0000000000000000e+12;
acadoVariables.ubAValues[64] = 1.0000000000000000e+12;
acadoVariables.ubAValues[65] = 1.0000000000000000e+12;
acadoVariables.ubAValues[66] = 1.0000000000000000e+12;
acadoVariables.ubAValues[67] = 1.0000000000000000e+12;
acadoVariables.ubAValues[68] = 1.0000000000000000e+12;
acadoVariables.ubAValues[69] = 1.0000000000000000e+12;
acadoVariables.ubAValues[70] = 1.0000000000000000e+12;
acadoVariables.ubAValues[71] = 1.0000000000000000e+12;
acadoVariables.ubAValues[72] = 1.0000000000000000e+12;
acadoVariables.ubAValues[73] = 1.0000000000000000e+12;
acadoVariables.ubAValues[74] = 1.0000000000000000e+12;
acadoVariables.ubAValues[75] = 1.0000000000000000e+12;
acadoVariables.ubAValues[76] = 1.0000000000000000e+12;
acadoVariables.ubAValues[77] = 1.0000000000000000e+12;
acadoVariables.ubAValues[78] = 1.0000000000000000e+12;
acadoVariables.ubAValues[79] = 1.0000000000000000e+12;
acadoVariables.ubAValues[80] = 1.0000000000000000e+12;
acadoVariables.ubAValues[81] = 1.0000000000000000e+12;
acadoVariables.ubAValues[82] = 1.0000000000000000e+12;
acadoVariables.ubAValues[83] = 1.0000000000000000e+12;
acadoVariables.ubAValues[84] = 1.0000000000000000e+12;
acadoVariables.ubAValues[85] = 1.0000000000000000e+12;
acadoVariables.ubAValues[86] = 1.0000000000000000e+12;
acadoVariables.ubAValues[87] = 1.0000000000000000e+12;
acadoVariables.ubAValues[88] = 1.0000000000000000e+12;
acadoVariables.ubAValues[89] = 1.0000000000000000e+12;
acadoVariables.ubAValues[90] = 1.0000000000000000e+12;
acadoVariables.ubAValues[91] = 1.0000000000000000e+12;
acadoVariables.ubAValues[92] = 1.0000000000000000e+12;
acadoVariables.ubAValues[93] = 1.0000000000000000e+12;
acadoVariables.ubAValues[94] = 1.0000000000000000e+12;
acadoVariables.ubAValues[95] = 1.0000000000000000e+12;
acadoVariables.ubAValues[96] = 1.0000000000000000e+12;
acadoVariables.ubAValues[97] = 1.0000000000000000e+12;
acadoVariables.ubAValues[98] = 1.0000000000000000e+12;
acadoVariables.ubAValues[99] = 1.0000000000000000e+12;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 50; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[36] = acadoVariables.u[index * 2];
acadoWorkspace.state[37] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[38] = acadoVariables.od[index * 18];
acadoWorkspace.state[39] = acadoVariables.od[index * 18 + 1];
acadoWorkspace.state[40] = acadoVariables.od[index * 18 + 2];
acadoWorkspace.state[41] = acadoVariables.od[index * 18 + 3];
acadoWorkspace.state[42] = acadoVariables.od[index * 18 + 4];
acadoWorkspace.state[43] = acadoVariables.od[index * 18 + 5];
acadoWorkspace.state[44] = acadoVariables.od[index * 18 + 6];
acadoWorkspace.state[45] = acadoVariables.od[index * 18 + 7];
acadoWorkspace.state[46] = acadoVariables.od[index * 18 + 8];
acadoWorkspace.state[47] = acadoVariables.od[index * 18 + 9];
acadoWorkspace.state[48] = acadoVariables.od[index * 18 + 10];
acadoWorkspace.state[49] = acadoVariables.od[index * 18 + 11];
acadoWorkspace.state[50] = acadoVariables.od[index * 18 + 12];
acadoWorkspace.state[51] = acadoVariables.od[index * 18 + 13];
acadoWorkspace.state[52] = acadoVariables.od[index * 18 + 14];
acadoWorkspace.state[53] = acadoVariables.od[index * 18 + 15];
acadoWorkspace.state[54] = acadoVariables.od[index * 18 + 16];
acadoWorkspace.state[55] = acadoVariables.od[index * 18 + 17];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 50; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[150] = xEnd[0];
acadoVariables.x[151] = xEnd[1];
acadoVariables.x[152] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[150];
acadoWorkspace.state[1] = acadoVariables.x[151];
acadoWorkspace.state[2] = acadoVariables.x[152];
if (uEnd != 0)
{
acadoWorkspace.state[36] = uEnd[0];
acadoWorkspace.state[37] = uEnd[1];
}
else
{
acadoWorkspace.state[36] = acadoVariables.u[98];
acadoWorkspace.state[37] = acadoVariables.u[99];
}
acadoWorkspace.state[38] = acadoVariables.od[900];
acadoWorkspace.state[39] = acadoVariables.od[901];
acadoWorkspace.state[40] = acadoVariables.od[902];
acadoWorkspace.state[41] = acadoVariables.od[903];
acadoWorkspace.state[42] = acadoVariables.od[904];
acadoWorkspace.state[43] = acadoVariables.od[905];
acadoWorkspace.state[44] = acadoVariables.od[906];
acadoWorkspace.state[45] = acadoVariables.od[907];
acadoWorkspace.state[46] = acadoVariables.od[908];
acadoWorkspace.state[47] = acadoVariables.od[909];
acadoWorkspace.state[48] = acadoVariables.od[910];
acadoWorkspace.state[49] = acadoVariables.od[911];
acadoWorkspace.state[50] = acadoVariables.od[912];
acadoWorkspace.state[51] = acadoVariables.od[913];
acadoWorkspace.state[52] = acadoVariables.od[914];
acadoWorkspace.state[53] = acadoVariables.od[915];
acadoWorkspace.state[54] = acadoVariables.od[916];
acadoWorkspace.state[55] = acadoVariables.od[917];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[150] = acadoWorkspace.state[0];
acadoVariables.x[151] = acadoWorkspace.state[1];
acadoVariables.x[152] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 49; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[98] = uEnd[0];
acadoVariables.u[99] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99];
kkt = fabs( kkt );
for (index = 0; index < 100; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 100; ++index)
{
prd = acadoWorkspace.y[index + 100];
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 18];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 18 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 18 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 18 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 18 + 4];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 18 + 5];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 18 + 6];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 18 + 7];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 18 + 8];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 18 + 9];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 18 + 10];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 18 + 11];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 18 + 12];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 18 + 13];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 18 + 14];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 18 + 15];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 18 + 16];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 18 + 17];

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.od[900];
acadoWorkspace.objValueIn[4] = acadoVariables.od[901];
acadoWorkspace.objValueIn[5] = acadoVariables.od[902];
acadoWorkspace.objValueIn[6] = acadoVariables.od[903];
acadoWorkspace.objValueIn[7] = acadoVariables.od[904];
acadoWorkspace.objValueIn[8] = acadoVariables.od[905];
acadoWorkspace.objValueIn[9] = acadoVariables.od[906];
acadoWorkspace.objValueIn[10] = acadoVariables.od[907];
acadoWorkspace.objValueIn[11] = acadoVariables.od[908];
acadoWorkspace.objValueIn[12] = acadoVariables.od[909];
acadoWorkspace.objValueIn[13] = acadoVariables.od[910];
acadoWorkspace.objValueIn[14] = acadoVariables.od[911];
acadoWorkspace.objValueIn[15] = acadoVariables.od[912];
acadoWorkspace.objValueIn[16] = acadoVariables.od[913];
acadoWorkspace.objValueIn[17] = acadoVariables.od[914];
acadoWorkspace.objValueIn[18] = acadoVariables.od[915];
acadoWorkspace.objValueIn[19] = acadoVariables.od[916];
acadoWorkspace.objValueIn[20] = acadoVariables.od[917];
acado_evaluateMayer( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
return objVal;
}

