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


void acado_acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
/* Vector of auxiliary variables; number of elements: 2. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));

/* Compute outputs: */
out[0] = (u[0]*a[0]);
out[1] = (u[0]*a[1]);
out[2] = u[1];
out[3] = u[2];
}



void acado_acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
/* Vector of auxiliary variables; number of elements: 4. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[1] = (cos(xd[2]));
a[2] = (cos(xd[2]));
a[3] = (sin(xd[2]));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (u[0]*a[0]);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = a[1];
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (u[0]*a[2]);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = a[3];
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(1.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(1.0000000000000000e+00);
}





void acado_acado_backward(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 36;
/* Vector of auxiliary variables; number of elements: 54. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[1] = (xd[32]*u[0]);
a[2] = (cos(xd[2]));
a[3] = (xd[33]*u[0]);
a[4] = ((real_t)(-1.0000000000000000e+00)*(cos(xd[2])));
a[5] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[6] = ((a[4]*a[1])+(a[5]*a[3]));
a[7] = (a[6]*xd[18]);
a[8] = (xd[18]*a[7]);
a[9] = (xd[19]*a[7]);
a[10] = (a[6]*xd[19]);
a[11] = (xd[19]*a[10]);
a[12] = (xd[20]*a[7]);
a[13] = (xd[20]*a[10]);
a[14] = (a[6]*xd[20]);
a[15] = (xd[20]*a[14]);
a[16] = (xd[21]*a[7]);
a[17] = (xd[21]*a[10]);
a[18] = (xd[21]*a[14]);
a[19] = (a[6]*xd[21]);
a[20] = (xd[21]*a[19]);
a[21] = (xd[22]*a[7]);
a[22] = ((a[0]*xd[32])+(a[2]*xd[33]));
a[23] = (a[22]*xd[18]);
a[24] = (a[21]+a[23]);
a[25] = (xd[22]*a[10]);
a[26] = (a[22]*xd[19]);
a[27] = (a[25]+a[26]);
a[28] = (xd[22]*a[14]);
a[29] = (a[22]*xd[20]);
a[30] = (a[28]+a[29]);
a[31] = (xd[22]*a[19]);
a[32] = (a[22]*xd[21]);
a[33] = (a[31]+a[32]);
a[34] = (a[6]*xd[22]);
a[35] = (a[34]+a[22]);
a[36] = (xd[22]*a[35]);
a[37] = (a[22]*xd[22]);
a[38] = (a[36]+a[37]);
a[39] = (xd[23]*a[7]);
a[40] = (xd[23]*a[10]);
a[41] = (xd[23]*a[14]);
a[42] = (xd[23]*a[19]);
a[43] = (xd[23]*a[35]);
a[44] = (a[6]*xd[23]);
a[45] = (xd[23]*a[44]);
a[46] = (xd[24]*a[7]);
a[47] = (xd[24]*a[10]);
a[48] = (xd[24]*a[14]);
a[49] = (xd[24]*a[19]);
a[50] = (xd[24]*a[35]);
a[51] = (xd[24]*a[44]);
a[52] = (a[6]*xd[24]);
a[53] = (xd[24]*a[52]);

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = ((a[0]*a[1])+(a[2]*a[3]));
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = a[8];
out[5] = a[9];
out[6] = a[11];
out[7] = a[12];
out[8] = a[13];
out[9] = a[15];
out[10] = a[16];
out[11] = a[17];
out[12] = a[18];
out[13] = a[20];
out[14] = a[24];
out[15] = a[27];
out[16] = a[30];
out[17] = a[33];
out[18] = a[38];
out[19] = a[39];
out[20] = a[40];
out[21] = a[41];
out[22] = a[42];
out[23] = a[43];
out[24] = a[45];
out[25] = a[46];
out[26] = a[47];
out[27] = a[48];
out[28] = a[49];
out[29] = a[50];
out[30] = a[51];
out[31] = a[53];
}



void acado_solve_dim8_triangular( real_t* const A, real_t* const b )
{

b[7] = b[7]/A[63];
b[6] -= + A[55]*b[7];
b[6] = b[6]/A[54];
b[5] -= + A[47]*b[7];
b[5] -= + A[46]*b[6];
b[5] = b[5]/A[45];
b[4] -= + A[39]*b[7];
b[4] -= + A[38]*b[6];
b[4] -= + A[37]*b[5];
b[4] = b[4]/A[36];
b[3] -= + A[31]*b[7];
b[3] -= + A[30]*b[6];
b[3] -= + A[29]*b[5];
b[3] -= + A[28]*b[4];
b[3] = b[3]/A[27];
b[2] -= + A[23]*b[7];
b[2] -= + A[22]*b[6];
b[2] -= + A[21]*b[5];
b[2] -= + A[20]*b[4];
b[2] -= + A[19]*b[3];
b[2] = b[2]/A[18];
b[1] -= + A[15]*b[7];
b[1] -= + A[14]*b[6];
b[1] -= + A[13]*b[5];
b[1] -= + A[12]*b[4];
b[1] -= + A[11]*b[3];
b[1] -= + A[10]*b[2];
b[1] = b[1]/A[9];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim8_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 8; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (7); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*8+i]);
	for( j=(i+1); j < 8; j++ ) {
		temp = fabs(A[j*8+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 8; ++k)
{
	acadoWorkspace.rk_dim8_swap = A[i*8+k];
	A[i*8+k] = A[indexMax*8+k];
	A[indexMax*8+k] = acadoWorkspace.rk_dim8_swap;
}
	acadoWorkspace.rk_dim8_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim8_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*8+i];
	for( j=i+1; j < 8; j++ ) {
		A[j*8+i] = -A[j*8+i]/A[i*8+i];
		for( k=i+1; k < 8; k++ ) {
			A[j*8+k] += A[j*8+i] * A[i*8+k];
		}
		b[j] += A[j*8+i] * b[i];
	}
}
det *= A[63];
det = fabs(det);
acado_solve_dim8_triangular( A, b );
return det;
}

void acado_solve_dim8_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim8_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim8_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim8_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim8_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim8_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim8_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim8_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim8_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim8_bPerm[1] += A[8]*acadoWorkspace.rk_dim8_bPerm[0];

acadoWorkspace.rk_dim8_bPerm[2] += A[16]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[2] += A[17]*acadoWorkspace.rk_dim8_bPerm[1];

acadoWorkspace.rk_dim8_bPerm[3] += A[24]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[3] += A[25]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[3] += A[26]*acadoWorkspace.rk_dim8_bPerm[2];

acadoWorkspace.rk_dim8_bPerm[4] += A[32]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[4] += A[33]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[4] += A[34]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[4] += A[35]*acadoWorkspace.rk_dim8_bPerm[3];

acadoWorkspace.rk_dim8_bPerm[5] += A[40]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[5] += A[41]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[5] += A[42]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[5] += A[43]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[5] += A[44]*acadoWorkspace.rk_dim8_bPerm[4];

acadoWorkspace.rk_dim8_bPerm[6] += A[48]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[6] += A[49]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[6] += A[50]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[6] += A[51]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[6] += A[52]*acadoWorkspace.rk_dim8_bPerm[4];
acadoWorkspace.rk_dim8_bPerm[6] += A[53]*acadoWorkspace.rk_dim8_bPerm[5];

acadoWorkspace.rk_dim8_bPerm[7] += A[56]*acadoWorkspace.rk_dim8_bPerm[0];
acadoWorkspace.rk_dim8_bPerm[7] += A[57]*acadoWorkspace.rk_dim8_bPerm[1];
acadoWorkspace.rk_dim8_bPerm[7] += A[58]*acadoWorkspace.rk_dim8_bPerm[2];
acadoWorkspace.rk_dim8_bPerm[7] += A[59]*acadoWorkspace.rk_dim8_bPerm[3];
acadoWorkspace.rk_dim8_bPerm[7] += A[60]*acadoWorkspace.rk_dim8_bPerm[4];
acadoWorkspace.rk_dim8_bPerm[7] += A[61]*acadoWorkspace.rk_dim8_bPerm[5];
acadoWorkspace.rk_dim8_bPerm[7] += A[62]*acadoWorkspace.rk_dim8_bPerm[6];


acado_solve_dim8_triangular( A, acadoWorkspace.rk_dim8_bPerm );
b[0] = acadoWorkspace.rk_dim8_bPerm[0];
b[1] = acadoWorkspace.rk_dim8_bPerm[1];
b[2] = acadoWorkspace.rk_dim8_bPerm[2];
b[3] = acadoWorkspace.rk_dim8_bPerm[3];
b[4] = acadoWorkspace.rk_dim8_bPerm[4];
b[5] = acadoWorkspace.rk_dim8_bPerm[5];
b[6] = acadoWorkspace.rk_dim8_bPerm[6];
b[7] = acadoWorkspace.rk_dim8_bPerm[7];
}

void acado_solve_dim8_transpose_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{
int i;
int j;

real_t tmp_var;

acadoWorkspace.rk_dim8_bPerm_trans[0] = b[0];
acadoWorkspace.rk_dim8_bPerm_trans[1] = b[1];
acadoWorkspace.rk_dim8_bPerm_trans[2] = b[2];
acadoWorkspace.rk_dim8_bPerm_trans[3] = b[3];
acadoWorkspace.rk_dim8_bPerm_trans[4] = b[4];
acadoWorkspace.rk_dim8_bPerm_trans[5] = b[5];
acadoWorkspace.rk_dim8_bPerm_trans[6] = b[6];
acadoWorkspace.rk_dim8_bPerm_trans[7] = b[7];
for (j = 0; j < 8; ++j)
{
for (i = 0; i < j; ++i)
{
acadoWorkspace.rk_dim8_bPerm_trans[j] -= + A[(i * 8) + (j)]*acadoWorkspace.rk_dim8_bPerm_trans[i];
}
tmp_var = 1.0/A[j*9];
acadoWorkspace.rk_dim8_bPerm_trans[j] = + acadoWorkspace.rk_dim8_bPerm_trans[j]*tmp_var;
}
for (i = 7; -1 < i; --i)
{
for (j = 7; i < j; --j)
{
acadoWorkspace.rk_dim8_bPerm_trans[i] += + A[(j * 8) + (i)]*acadoWorkspace.rk_dim8_bPerm_trans[j];
}
}
for (i = 0; i < 8; ++i)
{
j = rk_perm[i];
b[j] = acadoWorkspace.rk_dim8_bPerm_trans[i];
}
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t Ah_mat[ 4 ] = 
{ 1.2500000000000001e-02, 2.6933756729740646e-02, 
-1.9337567297406434e-03, 1.2500000000000001e-02 };


/* Fixed step size:0.05 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[4] = rk_eta[64];
acadoWorkspace.rk_xxx[5] = rk_eta[65];
acadoWorkspace.rk_xxx[6] = rk_eta[66];
acadoWorkspace.rk_xxx[7] = rk_eta[67];
acadoWorkspace.rk_xxx[8] = rk_eta[68];
acadoWorkspace.rk_xxx[9] = rk_eta[69];
acadoWorkspace.rk_xxx[10] = rk_eta[70];
acadoWorkspace.rk_xxx[11] = rk_eta[71];
acadoWorkspace.rk_xxx[12] = rk_eta[72];
acadoWorkspace.rk_xxx[13] = rk_eta[73];
acadoWorkspace.rk_xxx[14] = rk_eta[74];
acadoWorkspace.rk_xxx[15] = rk_eta[75];
acadoWorkspace.rk_xxx[16] = rk_eta[76];
acadoWorkspace.rk_xxx[17] = rk_eta[77];
acadoWorkspace.rk_xxx[18] = rk_eta[78];
acadoWorkspace.rk_xxx[19] = rk_eta[79];
acadoWorkspace.rk_xxx[20] = rk_eta[80];
acadoWorkspace.rk_xxx[21] = rk_eta[81];
acadoWorkspace.rk_xxx[22] = rk_eta[82];
acadoWorkspace.rk_xxx[23] = rk_eta[83];
acadoWorkspace.rk_xxx[24] = rk_eta[84];
acadoWorkspace.rk_xxx[25] = rk_eta[85];
acadoWorkspace.rk_seed[36] = rk_eta[64];
acadoWorkspace.rk_seed[37] = rk_eta[65];
acadoWorkspace.rk_seed[38] = rk_eta[66];
acadoWorkspace.rk_seed[39] = rk_eta[67];
acadoWorkspace.rk_seed[40] = rk_eta[68];
acadoWorkspace.rk_seed[41] = rk_eta[69];
acadoWorkspace.rk_seed[42] = rk_eta[70];
acadoWorkspace.rk_seed[43] = rk_eta[71];
acadoWorkspace.rk_seed[44] = rk_eta[72];
acadoWorkspace.rk_seed[45] = rk_eta[73];
acadoWorkspace.rk_seed[46] = rk_eta[74];
acadoWorkspace.rk_seed[47] = rk_eta[75];
acadoWorkspace.rk_seed[48] = rk_eta[76];
acadoWorkspace.rk_seed[49] = rk_eta[77];
acadoWorkspace.rk_seed[50] = rk_eta[78];
acadoWorkspace.rk_seed[51] = rk_eta[79];
acadoWorkspace.rk_seed[52] = rk_eta[80];
acadoWorkspace.rk_seed[53] = rk_eta[81];
acadoWorkspace.rk_seed[54] = rk_eta[82];
acadoWorkspace.rk_seed[55] = rk_eta[83];
acadoWorkspace.rk_seed[56] = rk_eta[84];
acadoWorkspace.rk_seed[57] = rk_eta[85];

/* ------------ Forward loop ------------: */
for (run = 0; run < 1; ++run)
{
if( run > 0 ) {
for (i = 0; i < 4; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 7] = rk_eta[i * 4 + 8];
acadoWorkspace.rk_diffsPrev2[i * 7 + 1] = rk_eta[i * 4 + 9];
acadoWorkspace.rk_diffsPrev2[i * 7 + 2] = rk_eta[i * 4 + 10];
acadoWorkspace.rk_diffsPrev2[i * 7 + 3] = rk_eta[i * 4 + 11];
acadoWorkspace.rk_diffsPrev2[i * 7 + 4] = rk_eta[i * 3 + 24];
acadoWorkspace.rk_diffsPrev2[i * 7 + 5] = rk_eta[i * 3 + 25];
acadoWorkspace.rk_diffsPrev2[i * 7 + 6] = rk_eta[i * 3 + 26];
}
}
else{
acadoWorkspace.rk_diffsPrev2[0] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[8] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[9] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[10] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[11] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[12] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[13] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[14] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[15] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[16] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[17] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[18] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[19] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[20] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[21] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[22] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[23] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[24] = 1.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[25] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[26] = 0.0000000000000000e+00;
acadoWorkspace.rk_diffsPrev2[27] = 0.0000000000000000e+00;
}
acadoWorkspace.rk_S_traj[run * 28] = acadoWorkspace.rk_diffsPrev2[0];
acadoWorkspace.rk_S_traj[run * 28 + 1] = acadoWorkspace.rk_diffsPrev2[1];
acadoWorkspace.rk_S_traj[run * 28 + 2] = acadoWorkspace.rk_diffsPrev2[2];
acadoWorkspace.rk_S_traj[run * 28 + 3] = acadoWorkspace.rk_diffsPrev2[3];
acadoWorkspace.rk_S_traj[run * 28 + 4] = acadoWorkspace.rk_diffsPrev2[4];
acadoWorkspace.rk_S_traj[run * 28 + 5] = acadoWorkspace.rk_diffsPrev2[5];
acadoWorkspace.rk_S_traj[run * 28 + 6] = acadoWorkspace.rk_diffsPrev2[6];
acadoWorkspace.rk_S_traj[run * 28 + 7] = acadoWorkspace.rk_diffsPrev2[7];
acadoWorkspace.rk_S_traj[run * 28 + 8] = acadoWorkspace.rk_diffsPrev2[8];
acadoWorkspace.rk_S_traj[run * 28 + 9] = acadoWorkspace.rk_diffsPrev2[9];
acadoWorkspace.rk_S_traj[run * 28 + 10] = acadoWorkspace.rk_diffsPrev2[10];
acadoWorkspace.rk_S_traj[run * 28 + 11] = acadoWorkspace.rk_diffsPrev2[11];
acadoWorkspace.rk_S_traj[run * 28 + 12] = acadoWorkspace.rk_diffsPrev2[12];
acadoWorkspace.rk_S_traj[run * 28 + 13] = acadoWorkspace.rk_diffsPrev2[13];
acadoWorkspace.rk_S_traj[run * 28 + 14] = acadoWorkspace.rk_diffsPrev2[14];
acadoWorkspace.rk_S_traj[run * 28 + 15] = acadoWorkspace.rk_diffsPrev2[15];
acadoWorkspace.rk_S_traj[run * 28 + 16] = acadoWorkspace.rk_diffsPrev2[16];
acadoWorkspace.rk_S_traj[run * 28 + 17] = acadoWorkspace.rk_diffsPrev2[17];
acadoWorkspace.rk_S_traj[run * 28 + 18] = acadoWorkspace.rk_diffsPrev2[18];
acadoWorkspace.rk_S_traj[run * 28 + 19] = acadoWorkspace.rk_diffsPrev2[19];
acadoWorkspace.rk_S_traj[run * 28 + 20] = acadoWorkspace.rk_diffsPrev2[20];
acadoWorkspace.rk_S_traj[run * 28 + 21] = acadoWorkspace.rk_diffsPrev2[21];
acadoWorkspace.rk_S_traj[run * 28 + 22] = acadoWorkspace.rk_diffsPrev2[22];
acadoWorkspace.rk_S_traj[run * 28 + 23] = acadoWorkspace.rk_diffsPrev2[23];
acadoWorkspace.rk_S_traj[run * 28 + 24] = acadoWorkspace.rk_diffsPrev2[24];
acadoWorkspace.rk_S_traj[run * 28 + 25] = acadoWorkspace.rk_diffsPrev2[25];
acadoWorkspace.rk_S_traj[run * 28 + 26] = acadoWorkspace.rk_diffsPrev2[26];
acadoWorkspace.rk_S_traj[run * 28 + 27] = acadoWorkspace.rk_diffsPrev2[27];
tmp_index2 = run * 4;
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 4; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = (tmp_index2) + (j);
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2 + 1];
}
acado_acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 28 ]) );
for (j = 0; j < 4; ++j)
{
tmp_index1 = (run1 * 4) + (j);
acadoWorkspace.rk_A[tmp_index1 * 8] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 1] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 2] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 3] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 3)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 8 + 4] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 5] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 6] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 7] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 3)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j + 4)] -= 1.0000000000000000e+00;
}
acado_acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 4] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (run1)] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 4 + 1] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (run1)] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 4 + 2] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (run1)] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 4 + 3] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (run1)] - acadoWorkspace.rk_rhsTemp[3];
}
det = acado_solve_dim8_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (j)] += acadoWorkspace.rk_b[j * 4];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (j)] += acadoWorkspace.rk_b[j * 4 + 1];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (j)] += acadoWorkspace.rk_b[j * 4 + 2];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (j)] += acadoWorkspace.rk_b[j * 4 + 3];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 4; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = (tmp_index2) + (j);
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2 + 1];
}
acado_acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 4] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (run1)] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 4 + 1] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (run1)] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 4 + 2] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (run1)] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 4 + 3] = acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (run1)] - acadoWorkspace.rk_rhsTemp[3];
}
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2) + (j)] += acadoWorkspace.rk_b[j * 4];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 2) + (j)] += acadoWorkspace.rk_b[j * 4 + 1];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 4) + (j)] += acadoWorkspace.rk_b[j * 4 + 2];
acadoWorkspace.rk_Ktraj[(tmp_index2 * 2 + 6) + (j)] += acadoWorkspace.rk_b[j * 4 + 3];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 4; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = (tmp_index2) + (j);
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2];
acadoWorkspace.rk_xxx[j] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[tmp_index1 * 2 + 1];
}
acado_acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 28 ]) );
for (j = 0; j < 4; ++j)
{
tmp_index1 = (run1 * 4) + (j);
acadoWorkspace.rk_A[tmp_index1 * 8] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 1] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 2] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 3] = + Ah_mat[run1 * 2]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 3)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j)] -= 1.0000000000000000e+00;
acadoWorkspace.rk_A[tmp_index1 * 8 + 4] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 5] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 6] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 8 + 7] = + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_diffsTemp2[(run1 * 28) + (j * 7 + 3)];
if( 1 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 8) + (j + 4)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 4; ++j)
{
tmp_index1 = (tmp_index2 * 2) + (run1 * 4);
i = (tmp_index2) + (j);
acadoWorkspace.rk_stageV_traj[(0) + ((0) + ((tmp_index1) + (j)))] = rk_eta[j];
acadoWorkspace.rk_stageV_traj[(0) + ((0) + ((tmp_index1) + (j)))] += + Ah_mat[run1 * 2]*acadoWorkspace.rk_Ktraj[i * 2];
acadoWorkspace.rk_stageV_traj[(0) + ((0) + ((tmp_index1) + (j)))] += + Ah_mat[run1 * 2 + 1]*acadoWorkspace.rk_Ktraj[i * 2 + 1];
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_b[i * 4] = - acadoWorkspace.rk_diffsTemp2[(i * 28) + (run1)];
acadoWorkspace.rk_b[i * 4 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 28) + (run1 + 7)];
acadoWorkspace.rk_b[i * 4 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 28) + (run1 + 14)];
acadoWorkspace.rk_b[i * 4 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 28) + (run1 + 21)];
}
if( 0 == run1 ) {
det = acado_solve_dim8_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
}
 else {
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
}
for (i = 0; i < 2; ++i)
{
tmp_index1 = (run * 28) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4];
tmp_index1 = (run * 28 + 7) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4 + 1];
tmp_index1 = (run * 28 + 14) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4 + 2];
tmp_index1 = (run * 28 + 21) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4 + 3];
}
for (i = 0; i < 4; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 7) + (run1)] = (i == run1-0);
tmp_index1 = ((run * 4) + (i)) * (7);
tmp_index2 = (tmp_index1) + (run1);
acadoWorkspace.rk_diffsNew2[(i * 7) + (run1)] += + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 4; ++j)
{
tmp_index1 = (i * 4) + (j);
tmp_index2 = (run1) + (j * 7);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 28) + (tmp_index2 + 4)];
}
}
acado_solve_dim8_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim8_perm );
for (i = 0; i < 2; ++i)
{
tmp_index1 = (run * 28 + 4) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4];
tmp_index1 = (run * 28 + 11) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4 + 1];
tmp_index1 = (run * 28 + 18) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4 + 2];
tmp_index1 = (run * 28 + 25) + (run1);
acadoWorkspace.rk_diffKtraj[(tmp_index1 * 2) + (i)] = acadoWorkspace.rk_b[i * 4 + 3];
}
for (i = 0; i < 4; ++i)
{
tmp_index1 = ((run * 4) + (i)) * (7);
tmp_index2 = (tmp_index1 + 4) + (run1);
acadoWorkspace.rk_diffsNew2[(i * 7) + (run1 + 4)] = + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
acadoWorkspace.rk_A_traj[run * 64] = acadoWorkspace.rk_A[0];
acadoWorkspace.rk_A_traj[run * 64 + 1] = acadoWorkspace.rk_A[1];
acadoWorkspace.rk_A_traj[run * 64 + 2] = acadoWorkspace.rk_A[2];
acadoWorkspace.rk_A_traj[run * 64 + 3] = acadoWorkspace.rk_A[3];
acadoWorkspace.rk_A_traj[run * 64 + 4] = acadoWorkspace.rk_A[4];
acadoWorkspace.rk_A_traj[run * 64 + 5] = acadoWorkspace.rk_A[5];
acadoWorkspace.rk_A_traj[run * 64 + 6] = acadoWorkspace.rk_A[6];
acadoWorkspace.rk_A_traj[run * 64 + 7] = acadoWorkspace.rk_A[7];
acadoWorkspace.rk_A_traj[run * 64 + 8] = acadoWorkspace.rk_A[8];
acadoWorkspace.rk_A_traj[run * 64 + 9] = acadoWorkspace.rk_A[9];
acadoWorkspace.rk_A_traj[run * 64 + 10] = acadoWorkspace.rk_A[10];
acadoWorkspace.rk_A_traj[run * 64 + 11] = acadoWorkspace.rk_A[11];
acadoWorkspace.rk_A_traj[run * 64 + 12] = acadoWorkspace.rk_A[12];
acadoWorkspace.rk_A_traj[run * 64 + 13] = acadoWorkspace.rk_A[13];
acadoWorkspace.rk_A_traj[run * 64 + 14] = acadoWorkspace.rk_A[14];
acadoWorkspace.rk_A_traj[run * 64 + 15] = acadoWorkspace.rk_A[15];
acadoWorkspace.rk_A_traj[run * 64 + 16] = acadoWorkspace.rk_A[16];
acadoWorkspace.rk_A_traj[run * 64 + 17] = acadoWorkspace.rk_A[17];
acadoWorkspace.rk_A_traj[run * 64 + 18] = acadoWorkspace.rk_A[18];
acadoWorkspace.rk_A_traj[run * 64 + 19] = acadoWorkspace.rk_A[19];
acadoWorkspace.rk_A_traj[run * 64 + 20] = acadoWorkspace.rk_A[20];
acadoWorkspace.rk_A_traj[run * 64 + 21] = acadoWorkspace.rk_A[21];
acadoWorkspace.rk_A_traj[run * 64 + 22] = acadoWorkspace.rk_A[22];
acadoWorkspace.rk_A_traj[run * 64 + 23] = acadoWorkspace.rk_A[23];
acadoWorkspace.rk_A_traj[run * 64 + 24] = acadoWorkspace.rk_A[24];
acadoWorkspace.rk_A_traj[run * 64 + 25] = acadoWorkspace.rk_A[25];
acadoWorkspace.rk_A_traj[run * 64 + 26] = acadoWorkspace.rk_A[26];
acadoWorkspace.rk_A_traj[run * 64 + 27] = acadoWorkspace.rk_A[27];
acadoWorkspace.rk_A_traj[run * 64 + 28] = acadoWorkspace.rk_A[28];
acadoWorkspace.rk_A_traj[run * 64 + 29] = acadoWorkspace.rk_A[29];
acadoWorkspace.rk_A_traj[run * 64 + 30] = acadoWorkspace.rk_A[30];
acadoWorkspace.rk_A_traj[run * 64 + 31] = acadoWorkspace.rk_A[31];
acadoWorkspace.rk_A_traj[run * 64 + 32] = acadoWorkspace.rk_A[32];
acadoWorkspace.rk_A_traj[run * 64 + 33] = acadoWorkspace.rk_A[33];
acadoWorkspace.rk_A_traj[run * 64 + 34] = acadoWorkspace.rk_A[34];
acadoWorkspace.rk_A_traj[run * 64 + 35] = acadoWorkspace.rk_A[35];
acadoWorkspace.rk_A_traj[run * 64 + 36] = acadoWorkspace.rk_A[36];
acadoWorkspace.rk_A_traj[run * 64 + 37] = acadoWorkspace.rk_A[37];
acadoWorkspace.rk_A_traj[run * 64 + 38] = acadoWorkspace.rk_A[38];
acadoWorkspace.rk_A_traj[run * 64 + 39] = acadoWorkspace.rk_A[39];
acadoWorkspace.rk_A_traj[run * 64 + 40] = acadoWorkspace.rk_A[40];
acadoWorkspace.rk_A_traj[run * 64 + 41] = acadoWorkspace.rk_A[41];
acadoWorkspace.rk_A_traj[run * 64 + 42] = acadoWorkspace.rk_A[42];
acadoWorkspace.rk_A_traj[run * 64 + 43] = acadoWorkspace.rk_A[43];
acadoWorkspace.rk_A_traj[run * 64 + 44] = acadoWorkspace.rk_A[44];
acadoWorkspace.rk_A_traj[run * 64 + 45] = acadoWorkspace.rk_A[45];
acadoWorkspace.rk_A_traj[run * 64 + 46] = acadoWorkspace.rk_A[46];
acadoWorkspace.rk_A_traj[run * 64 + 47] = acadoWorkspace.rk_A[47];
acadoWorkspace.rk_A_traj[run * 64 + 48] = acadoWorkspace.rk_A[48];
acadoWorkspace.rk_A_traj[run * 64 + 49] = acadoWorkspace.rk_A[49];
acadoWorkspace.rk_A_traj[run * 64 + 50] = acadoWorkspace.rk_A[50];
acadoWorkspace.rk_A_traj[run * 64 + 51] = acadoWorkspace.rk_A[51];
acadoWorkspace.rk_A_traj[run * 64 + 52] = acadoWorkspace.rk_A[52];
acadoWorkspace.rk_A_traj[run * 64 + 53] = acadoWorkspace.rk_A[53];
acadoWorkspace.rk_A_traj[run * 64 + 54] = acadoWorkspace.rk_A[54];
acadoWorkspace.rk_A_traj[run * 64 + 55] = acadoWorkspace.rk_A[55];
acadoWorkspace.rk_A_traj[run * 64 + 56] = acadoWorkspace.rk_A[56];
acadoWorkspace.rk_A_traj[run * 64 + 57] = acadoWorkspace.rk_A[57];
acadoWorkspace.rk_A_traj[run * 64 + 58] = acadoWorkspace.rk_A[58];
acadoWorkspace.rk_A_traj[run * 64 + 59] = acadoWorkspace.rk_A[59];
acadoWorkspace.rk_A_traj[run * 64 + 60] = acadoWorkspace.rk_A[60];
acadoWorkspace.rk_A_traj[run * 64 + 61] = acadoWorkspace.rk_A[61];
acadoWorkspace.rk_A_traj[run * 64 + 62] = acadoWorkspace.rk_A[62];
acadoWorkspace.rk_A_traj[run * 64 + 63] = acadoWorkspace.rk_A[63];
acadoWorkspace.rk_aux_traj[run * 8] = acadoWorkspace.rk_dim8_perm[0];
acadoWorkspace.rk_aux_traj[run * 8 + 1] = acadoWorkspace.rk_dim8_perm[1];
acadoWorkspace.rk_aux_traj[run * 8 + 2] = acadoWorkspace.rk_dim8_perm[2];
acadoWorkspace.rk_aux_traj[run * 8 + 3] = acadoWorkspace.rk_dim8_perm[3];
acadoWorkspace.rk_aux_traj[run * 8 + 4] = acadoWorkspace.rk_dim8_perm[4];
acadoWorkspace.rk_aux_traj[run * 8 + 5] = acadoWorkspace.rk_dim8_perm[5];
acadoWorkspace.rk_aux_traj[run * 8 + 6] = acadoWorkspace.rk_dim8_perm[6];
acadoWorkspace.rk_aux_traj[run * 8 + 7] = acadoWorkspace.rk_dim8_perm[7];
rk_eta[0] += + acadoWorkspace.rk_Ktraj[run * 8]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_Ktraj[run * 8 + 1]*(real_t)2.5000000000000001e-02;
rk_eta[1] += + acadoWorkspace.rk_Ktraj[run * 8 + 2]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_Ktraj[run * 8 + 3]*(real_t)2.5000000000000001e-02;
rk_eta[2] += + acadoWorkspace.rk_Ktraj[run * 8 + 4]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_Ktraj[run * 8 + 5]*(real_t)2.5000000000000001e-02;
rk_eta[3] += + acadoWorkspace.rk_Ktraj[run * 8 + 6]*(real_t)2.5000000000000001e-02 + acadoWorkspace.rk_Ktraj[run * 8 + 7]*(real_t)2.5000000000000001e-02;
for (i = 0; i < 4; ++i)
{
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 8] = acadoWorkspace.rk_diffsNew2[(i * 7) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 24] = acadoWorkspace.rk_diffsNew2[(i * 7) + (j + 4)];
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
/* ------------ BACKWARD loop ------------: */
rk_eta[36] = 0.0000000000000000e+00;
rk_eta[37] = 0.0000000000000000e+00;
rk_eta[38] = 0.0000000000000000e+00;
rk_eta[39] = 0.0000000000000000e+00;
rk_eta[40] = 0.0000000000000000e+00;
rk_eta[41] = 0.0000000000000000e+00;
rk_eta[42] = 0.0000000000000000e+00;
rk_eta[43] = 0.0000000000000000e+00;
rk_eta[44] = 0.0000000000000000e+00;
rk_eta[45] = 0.0000000000000000e+00;
rk_eta[46] = 0.0000000000000000e+00;
rk_eta[47] = 0.0000000000000000e+00;
rk_eta[48] = 0.0000000000000000e+00;
rk_eta[49] = 0.0000000000000000e+00;
rk_eta[50] = 0.0000000000000000e+00;
rk_eta[51] = 0.0000000000000000e+00;
rk_eta[52] = 0.0000000000000000e+00;
rk_eta[53] = 0.0000000000000000e+00;
rk_eta[54] = 0.0000000000000000e+00;
rk_eta[55] = 0.0000000000000000e+00;
rk_eta[56] = 0.0000000000000000e+00;
rk_eta[57] = 0.0000000000000000e+00;
rk_eta[58] = 0.0000000000000000e+00;
rk_eta[59] = 0.0000000000000000e+00;
rk_eta[60] = 0.0000000000000000e+00;
rk_eta[61] = 0.0000000000000000e+00;
rk_eta[62] = 0.0000000000000000e+00;
rk_eta[63] = 0.0000000000000000e+00;
for (run = 0; -1 < run; --run)
{
acadoWorkspace.rk_b_trans[0] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[1] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[2] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[3] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[0] -= + (real_t)2.5000000000000001e-02*rk_eta[4];
acadoWorkspace.rk_b_trans[1] -= + (real_t)2.5000000000000001e-02*rk_eta[5];
acadoWorkspace.rk_b_trans[2] -= + (real_t)2.5000000000000001e-02*rk_eta[6];
acadoWorkspace.rk_b_trans[3] -= + (real_t)2.5000000000000001e-02*rk_eta[7];
acadoWorkspace.rk_b_trans[4] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[5] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[6] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[7] = 0.0000000000000000e+00;
acadoWorkspace.rk_b_trans[4] -= + (real_t)2.5000000000000001e-02*rk_eta[4];
acadoWorkspace.rk_b_trans[5] -= + (real_t)2.5000000000000001e-02*rk_eta[5];
acadoWorkspace.rk_b_trans[6] -= + (real_t)2.5000000000000001e-02*rk_eta[6];
acadoWorkspace.rk_b_trans[7] -= + (real_t)2.5000000000000001e-02*rk_eta[7];
acado_solve_dim8_transpose_reuse( &(acadoWorkspace.rk_A_traj[ run * 64 ]), acadoWorkspace.rk_b_trans, &(acadoWorkspace.rk_aux_traj[ run * 8 ]) );
acadoWorkspace.rk_seed[0] = acadoWorkspace.rk_stageV_traj[run * 8];
acadoWorkspace.rk_seed[1] = acadoWorkspace.rk_stageV_traj[run * 8 + 1];
acadoWorkspace.rk_seed[2] = acadoWorkspace.rk_stageV_traj[run * 8 + 2];
acadoWorkspace.rk_seed[3] = acadoWorkspace.rk_stageV_traj[run * 8 + 3];
acadoWorkspace.rk_diffsPrev2[0] = acadoWorkspace.rk_S_traj[run * 28];
acadoWorkspace.rk_diffsPrev2[1] = acadoWorkspace.rk_S_traj[run * 28 + 1];
acadoWorkspace.rk_diffsPrev2[2] = acadoWorkspace.rk_S_traj[run * 28 + 2];
acadoWorkspace.rk_diffsPrev2[3] = acadoWorkspace.rk_S_traj[run * 28 + 3];
acadoWorkspace.rk_diffsPrev2[4] = acadoWorkspace.rk_S_traj[run * 28 + 4];
acadoWorkspace.rk_diffsPrev2[5] = acadoWorkspace.rk_S_traj[run * 28 + 5];
acadoWorkspace.rk_diffsPrev2[6] = acadoWorkspace.rk_S_traj[run * 28 + 6];
acadoWorkspace.rk_diffsPrev2[7] = acadoWorkspace.rk_S_traj[run * 28 + 7];
acadoWorkspace.rk_diffsPrev2[8] = acadoWorkspace.rk_S_traj[run * 28 + 8];
acadoWorkspace.rk_diffsPrev2[9] = acadoWorkspace.rk_S_traj[run * 28 + 9];
acadoWorkspace.rk_diffsPrev2[10] = acadoWorkspace.rk_S_traj[run * 28 + 10];
acadoWorkspace.rk_diffsPrev2[11] = acadoWorkspace.rk_S_traj[run * 28 + 11];
acadoWorkspace.rk_diffsPrev2[12] = acadoWorkspace.rk_S_traj[run * 28 + 12];
acadoWorkspace.rk_diffsPrev2[13] = acadoWorkspace.rk_S_traj[run * 28 + 13];
acadoWorkspace.rk_diffsPrev2[14] = acadoWorkspace.rk_S_traj[run * 28 + 14];
acadoWorkspace.rk_diffsPrev2[15] = acadoWorkspace.rk_S_traj[run * 28 + 15];
acadoWorkspace.rk_diffsPrev2[16] = acadoWorkspace.rk_S_traj[run * 28 + 16];
acadoWorkspace.rk_diffsPrev2[17] = acadoWorkspace.rk_S_traj[run * 28 + 17];
acadoWorkspace.rk_diffsPrev2[18] = acadoWorkspace.rk_S_traj[run * 28 + 18];
acadoWorkspace.rk_diffsPrev2[19] = acadoWorkspace.rk_S_traj[run * 28 + 19];
acadoWorkspace.rk_diffsPrev2[20] = acadoWorkspace.rk_S_traj[run * 28 + 20];
acadoWorkspace.rk_diffsPrev2[21] = acadoWorkspace.rk_S_traj[run * 28 + 21];
acadoWorkspace.rk_diffsPrev2[22] = acadoWorkspace.rk_S_traj[run * 28 + 22];
acadoWorkspace.rk_diffsPrev2[23] = acadoWorkspace.rk_S_traj[run * 28 + 23];
acadoWorkspace.rk_diffsPrev2[24] = acadoWorkspace.rk_S_traj[run * 28 + 24];
acadoWorkspace.rk_diffsPrev2[25] = acadoWorkspace.rk_S_traj[run * 28 + 25];
acadoWorkspace.rk_diffsPrev2[26] = acadoWorkspace.rk_S_traj[run * 28 + 26];
acadoWorkspace.rk_diffsPrev2[27] = acadoWorkspace.rk_S_traj[run * 28 + 27];
for (i = 0; i < 4; ++i)
{
tmp_index1 = ((run * 4) + (i)) * (7);
for (j = 0; j < 7; ++j)
{
tmp_index2 = (tmp_index1) + (j);
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[0]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2];
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[1]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1];
}
}
acadoWorkspace.rk_seed[4] = acadoWorkspace.rk_diffsPrev2[0];
acadoWorkspace.rk_seed[5] = acadoWorkspace.rk_diffsPrev2[1];
acadoWorkspace.rk_seed[6] = acadoWorkspace.rk_diffsPrev2[2];
acadoWorkspace.rk_seed[7] = acadoWorkspace.rk_diffsPrev2[3];
acadoWorkspace.rk_seed[8] = acadoWorkspace.rk_diffsPrev2[4];
acadoWorkspace.rk_seed[9] = acadoWorkspace.rk_diffsPrev2[5];
acadoWorkspace.rk_seed[10] = acadoWorkspace.rk_diffsPrev2[6];
acadoWorkspace.rk_seed[11] = acadoWorkspace.rk_diffsPrev2[7];
acadoWorkspace.rk_seed[12] = acadoWorkspace.rk_diffsPrev2[8];
acadoWorkspace.rk_seed[13] = acadoWorkspace.rk_diffsPrev2[9];
acadoWorkspace.rk_seed[14] = acadoWorkspace.rk_diffsPrev2[10];
acadoWorkspace.rk_seed[15] = acadoWorkspace.rk_diffsPrev2[11];
acadoWorkspace.rk_seed[16] = acadoWorkspace.rk_diffsPrev2[12];
acadoWorkspace.rk_seed[17] = acadoWorkspace.rk_diffsPrev2[13];
acadoWorkspace.rk_seed[18] = acadoWorkspace.rk_diffsPrev2[14];
acadoWorkspace.rk_seed[19] = acadoWorkspace.rk_diffsPrev2[15];
acadoWorkspace.rk_seed[20] = acadoWorkspace.rk_diffsPrev2[16];
acadoWorkspace.rk_seed[21] = acadoWorkspace.rk_diffsPrev2[17];
acadoWorkspace.rk_seed[22] = acadoWorkspace.rk_diffsPrev2[18];
acadoWorkspace.rk_seed[23] = acadoWorkspace.rk_diffsPrev2[19];
acadoWorkspace.rk_seed[24] = acadoWorkspace.rk_diffsPrev2[20];
acadoWorkspace.rk_seed[25] = acadoWorkspace.rk_diffsPrev2[21];
acadoWorkspace.rk_seed[26] = acadoWorkspace.rk_diffsPrev2[22];
acadoWorkspace.rk_seed[27] = acadoWorkspace.rk_diffsPrev2[23];
acadoWorkspace.rk_seed[28] = acadoWorkspace.rk_diffsPrev2[24];
acadoWorkspace.rk_seed[29] = acadoWorkspace.rk_diffsPrev2[25];
acadoWorkspace.rk_seed[30] = acadoWorkspace.rk_diffsPrev2[26];
acadoWorkspace.rk_seed[31] = acadoWorkspace.rk_diffsPrev2[27];
acadoWorkspace.rk_seed[32] = acadoWorkspace.rk_b_trans[0];
acadoWorkspace.rk_seed[33] = acadoWorkspace.rk_b_trans[1];
acadoWorkspace.rk_seed[34] = acadoWorkspace.rk_b_trans[2];
acadoWorkspace.rk_seed[35] = acadoWorkspace.rk_b_trans[3];
acado_acado_backward( acadoWorkspace.rk_seed, acadoWorkspace.rk_adjoint );
rk_eta[4] += acadoWorkspace.rk_adjoint[0];
rk_eta[5] += acadoWorkspace.rk_adjoint[1];
rk_eta[6] += acadoWorkspace.rk_adjoint[2];
rk_eta[7] += acadoWorkspace.rk_adjoint[3];
rk_eta[36] += acadoWorkspace.rk_adjoint[4];
rk_eta[37] += acadoWorkspace.rk_adjoint[5];
rk_eta[38] += acadoWorkspace.rk_adjoint[6];
rk_eta[39] += acadoWorkspace.rk_adjoint[7];
rk_eta[40] += acadoWorkspace.rk_adjoint[8];
rk_eta[41] += acadoWorkspace.rk_adjoint[9];
rk_eta[42] += acadoWorkspace.rk_adjoint[10];
rk_eta[43] += acadoWorkspace.rk_adjoint[11];
rk_eta[44] += acadoWorkspace.rk_adjoint[12];
rk_eta[45] += acadoWorkspace.rk_adjoint[13];
rk_eta[46] += acadoWorkspace.rk_adjoint[14];
rk_eta[47] += acadoWorkspace.rk_adjoint[15];
rk_eta[48] += acadoWorkspace.rk_adjoint[16];
rk_eta[49] += acadoWorkspace.rk_adjoint[17];
rk_eta[50] += acadoWorkspace.rk_adjoint[18];
rk_eta[51] += acadoWorkspace.rk_adjoint[19];
rk_eta[52] += acadoWorkspace.rk_adjoint[20];
rk_eta[53] += acadoWorkspace.rk_adjoint[21];
rk_eta[54] += acadoWorkspace.rk_adjoint[22];
rk_eta[55] += acadoWorkspace.rk_adjoint[23];
rk_eta[56] += acadoWorkspace.rk_adjoint[24];
rk_eta[57] += acadoWorkspace.rk_adjoint[25];
rk_eta[58] += acadoWorkspace.rk_adjoint[26];
rk_eta[59] += acadoWorkspace.rk_adjoint[27];
rk_eta[60] += acadoWorkspace.rk_adjoint[28];
rk_eta[61] += acadoWorkspace.rk_adjoint[29];
rk_eta[62] += acadoWorkspace.rk_adjoint[30];
rk_eta[63] += acadoWorkspace.rk_adjoint[31];
acadoWorkspace.rk_seed[0] = acadoWorkspace.rk_stageV_traj[run * 8 + 4];
acadoWorkspace.rk_seed[1] = acadoWorkspace.rk_stageV_traj[run * 8 + 5];
acadoWorkspace.rk_seed[2] = acadoWorkspace.rk_stageV_traj[run * 8 + 6];
acadoWorkspace.rk_seed[3] = acadoWorkspace.rk_stageV_traj[run * 8 + 7];
acadoWorkspace.rk_diffsPrev2[0] = acadoWorkspace.rk_S_traj[run * 28];
acadoWorkspace.rk_diffsPrev2[1] = acadoWorkspace.rk_S_traj[run * 28 + 1];
acadoWorkspace.rk_diffsPrev2[2] = acadoWorkspace.rk_S_traj[run * 28 + 2];
acadoWorkspace.rk_diffsPrev2[3] = acadoWorkspace.rk_S_traj[run * 28 + 3];
acadoWorkspace.rk_diffsPrev2[4] = acadoWorkspace.rk_S_traj[run * 28 + 4];
acadoWorkspace.rk_diffsPrev2[5] = acadoWorkspace.rk_S_traj[run * 28 + 5];
acadoWorkspace.rk_diffsPrev2[6] = acadoWorkspace.rk_S_traj[run * 28 + 6];
acadoWorkspace.rk_diffsPrev2[7] = acadoWorkspace.rk_S_traj[run * 28 + 7];
acadoWorkspace.rk_diffsPrev2[8] = acadoWorkspace.rk_S_traj[run * 28 + 8];
acadoWorkspace.rk_diffsPrev2[9] = acadoWorkspace.rk_S_traj[run * 28 + 9];
acadoWorkspace.rk_diffsPrev2[10] = acadoWorkspace.rk_S_traj[run * 28 + 10];
acadoWorkspace.rk_diffsPrev2[11] = acadoWorkspace.rk_S_traj[run * 28 + 11];
acadoWorkspace.rk_diffsPrev2[12] = acadoWorkspace.rk_S_traj[run * 28 + 12];
acadoWorkspace.rk_diffsPrev2[13] = acadoWorkspace.rk_S_traj[run * 28 + 13];
acadoWorkspace.rk_diffsPrev2[14] = acadoWorkspace.rk_S_traj[run * 28 + 14];
acadoWorkspace.rk_diffsPrev2[15] = acadoWorkspace.rk_S_traj[run * 28 + 15];
acadoWorkspace.rk_diffsPrev2[16] = acadoWorkspace.rk_S_traj[run * 28 + 16];
acadoWorkspace.rk_diffsPrev2[17] = acadoWorkspace.rk_S_traj[run * 28 + 17];
acadoWorkspace.rk_diffsPrev2[18] = acadoWorkspace.rk_S_traj[run * 28 + 18];
acadoWorkspace.rk_diffsPrev2[19] = acadoWorkspace.rk_S_traj[run * 28 + 19];
acadoWorkspace.rk_diffsPrev2[20] = acadoWorkspace.rk_S_traj[run * 28 + 20];
acadoWorkspace.rk_diffsPrev2[21] = acadoWorkspace.rk_S_traj[run * 28 + 21];
acadoWorkspace.rk_diffsPrev2[22] = acadoWorkspace.rk_S_traj[run * 28 + 22];
acadoWorkspace.rk_diffsPrev2[23] = acadoWorkspace.rk_S_traj[run * 28 + 23];
acadoWorkspace.rk_diffsPrev2[24] = acadoWorkspace.rk_S_traj[run * 28 + 24];
acadoWorkspace.rk_diffsPrev2[25] = acadoWorkspace.rk_S_traj[run * 28 + 25];
acadoWorkspace.rk_diffsPrev2[26] = acadoWorkspace.rk_S_traj[run * 28 + 26];
acadoWorkspace.rk_diffsPrev2[27] = acadoWorkspace.rk_S_traj[run * 28 + 27];
for (i = 0; i < 4; ++i)
{
tmp_index1 = ((run * 4) + (i)) * (7);
for (j = 0; j < 7; ++j)
{
tmp_index2 = (tmp_index1) + (j);
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[2]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2];
acadoWorkspace.rk_diffsPrev2[(i * 7) + (j)] += + Ah_mat[3]*acadoWorkspace.rk_diffKtraj[tmp_index2 * 2 + 1];
}
}
acadoWorkspace.rk_seed[4] = acadoWorkspace.rk_diffsPrev2[0];
acadoWorkspace.rk_seed[5] = acadoWorkspace.rk_diffsPrev2[1];
acadoWorkspace.rk_seed[6] = acadoWorkspace.rk_diffsPrev2[2];
acadoWorkspace.rk_seed[7] = acadoWorkspace.rk_diffsPrev2[3];
acadoWorkspace.rk_seed[8] = acadoWorkspace.rk_diffsPrev2[4];
acadoWorkspace.rk_seed[9] = acadoWorkspace.rk_diffsPrev2[5];
acadoWorkspace.rk_seed[10] = acadoWorkspace.rk_diffsPrev2[6];
acadoWorkspace.rk_seed[11] = acadoWorkspace.rk_diffsPrev2[7];
acadoWorkspace.rk_seed[12] = acadoWorkspace.rk_diffsPrev2[8];
acadoWorkspace.rk_seed[13] = acadoWorkspace.rk_diffsPrev2[9];
acadoWorkspace.rk_seed[14] = acadoWorkspace.rk_diffsPrev2[10];
acadoWorkspace.rk_seed[15] = acadoWorkspace.rk_diffsPrev2[11];
acadoWorkspace.rk_seed[16] = acadoWorkspace.rk_diffsPrev2[12];
acadoWorkspace.rk_seed[17] = acadoWorkspace.rk_diffsPrev2[13];
acadoWorkspace.rk_seed[18] = acadoWorkspace.rk_diffsPrev2[14];
acadoWorkspace.rk_seed[19] = acadoWorkspace.rk_diffsPrev2[15];
acadoWorkspace.rk_seed[20] = acadoWorkspace.rk_diffsPrev2[16];
acadoWorkspace.rk_seed[21] = acadoWorkspace.rk_diffsPrev2[17];
acadoWorkspace.rk_seed[22] = acadoWorkspace.rk_diffsPrev2[18];
acadoWorkspace.rk_seed[23] = acadoWorkspace.rk_diffsPrev2[19];
acadoWorkspace.rk_seed[24] = acadoWorkspace.rk_diffsPrev2[20];
acadoWorkspace.rk_seed[25] = acadoWorkspace.rk_diffsPrev2[21];
acadoWorkspace.rk_seed[26] = acadoWorkspace.rk_diffsPrev2[22];
acadoWorkspace.rk_seed[27] = acadoWorkspace.rk_diffsPrev2[23];
acadoWorkspace.rk_seed[28] = acadoWorkspace.rk_diffsPrev2[24];
acadoWorkspace.rk_seed[29] = acadoWorkspace.rk_diffsPrev2[25];
acadoWorkspace.rk_seed[30] = acadoWorkspace.rk_diffsPrev2[26];
acadoWorkspace.rk_seed[31] = acadoWorkspace.rk_diffsPrev2[27];
acadoWorkspace.rk_seed[32] = acadoWorkspace.rk_b_trans[4];
acadoWorkspace.rk_seed[33] = acadoWorkspace.rk_b_trans[5];
acadoWorkspace.rk_seed[34] = acadoWorkspace.rk_b_trans[6];
acadoWorkspace.rk_seed[35] = acadoWorkspace.rk_b_trans[7];
acado_acado_backward( acadoWorkspace.rk_seed, acadoWorkspace.rk_adjoint );
rk_eta[4] += acadoWorkspace.rk_adjoint[0];
rk_eta[5] += acadoWorkspace.rk_adjoint[1];
rk_eta[6] += acadoWorkspace.rk_adjoint[2];
rk_eta[7] += acadoWorkspace.rk_adjoint[3];
rk_eta[36] += acadoWorkspace.rk_adjoint[4];
rk_eta[37] += acadoWorkspace.rk_adjoint[5];
rk_eta[38] += acadoWorkspace.rk_adjoint[6];
rk_eta[39] += acadoWorkspace.rk_adjoint[7];
rk_eta[40] += acadoWorkspace.rk_adjoint[8];
rk_eta[41] += acadoWorkspace.rk_adjoint[9];
rk_eta[42] += acadoWorkspace.rk_adjoint[10];
rk_eta[43] += acadoWorkspace.rk_adjoint[11];
rk_eta[44] += acadoWorkspace.rk_adjoint[12];
rk_eta[45] += acadoWorkspace.rk_adjoint[13];
rk_eta[46] += acadoWorkspace.rk_adjoint[14];
rk_eta[47] += acadoWorkspace.rk_adjoint[15];
rk_eta[48] += acadoWorkspace.rk_adjoint[16];
rk_eta[49] += acadoWorkspace.rk_adjoint[17];
rk_eta[50] += acadoWorkspace.rk_adjoint[18];
rk_eta[51] += acadoWorkspace.rk_adjoint[19];
rk_eta[52] += acadoWorkspace.rk_adjoint[20];
rk_eta[53] += acadoWorkspace.rk_adjoint[21];
rk_eta[54] += acadoWorkspace.rk_adjoint[22];
rk_eta[55] += acadoWorkspace.rk_adjoint[23];
rk_eta[56] += acadoWorkspace.rk_adjoint[24];
rk_eta[57] += acadoWorkspace.rk_adjoint[25];
rk_eta[58] += acadoWorkspace.rk_adjoint[26];
rk_eta[59] += acadoWorkspace.rk_adjoint[27];
rk_eta[60] += acadoWorkspace.rk_adjoint[28];
rk_eta[61] += acadoWorkspace.rk_adjoint[29];
rk_eta[62] += acadoWorkspace.rk_adjoint[30];
rk_eta[63] += acadoWorkspace.rk_adjoint[31];
acadoWorkspace.rk_ttt -= 1.0000000000000000e+00;
}
error = 0;
return error;
}



