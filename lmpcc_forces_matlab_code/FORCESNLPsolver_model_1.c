/* This function was automatically generated by CasADi */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef CODEGEN_PREFIX
#define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
#define _NAMESPACE_CONCAT(NS, ID) NS ## ID
#define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else /* CODEGEN_PREFIX */
#define CASADI_PREFIX(ID) FORCESNLPsolver_model_1_ ## ID
#endif /* CODEGEN_PREFIX */

#include <math.h>

#include "FORCESNLPsolver/include/FORCESNLPsolver.h"

#define PRINTF printf
FORCESNLPsolver_float CASADI_PREFIX(sq)(FORCESNLPsolver_float x) { return x*x;}
#define sq(x) CASADI_PREFIX(sq)(x)

FORCESNLPsolver_float CASADI_PREFIX(sign)(FORCESNLPsolver_float x) { return x<0 ? -1 : x>0 ? 1 : x;}
#define sign(x) CASADI_PREFIX(sign)(x)

static const solver_int32_default CASADI_PREFIX(s0)[] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
#define s0 CASADI_PREFIX(s0)
static const solver_int32_default CASADI_PREFIX(s1)[] = {340, 1, 0, 340, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339};
#define s1 CASADI_PREFIX(s1)
static const solver_int32_default CASADI_PREFIX(s2)[] = {1, 1, 0, 1, 0};
#define s2 CASADI_PREFIX(s2)
static const solver_int32_default CASADI_PREFIX(s3)[] = {1, 9, 0, 1, 2, 3, 4, 5, 5, 6, 7, 7, 0, 0, 0, 0, 0, 0, 0};
#define s3 CASADI_PREFIX(s3)
static const solver_int32_default CASADI_PREFIX(s4)[] = {2, 1, 0, 2, 0, 1};
#define s4 CASADI_PREFIX(s4)
static const solver_int32_default CASADI_PREFIX(s5)[] = {2, 9, 0, 0, 0, 2, 4, 6, 6, 6, 8, 8, 0, 1, 0, 1, 0, 1, 0, 1};
#define s5 CASADI_PREFIX(s5)
static const solver_int32_default CASADI_PREFIX(s6)[] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
#define s6 CASADI_PREFIX(s6)
static const solver_int32_default CASADI_PREFIX(s7)[] = {6, 9, 0, 5, 8, 9, 10, 11, 14, 19, 20, 21, 0, 1, 2, 3, 4, 0, 1, 2, 5, 0, 1, 0, 1, 2, 0, 1, 2, 3, 4, 4, 5};
#define s7 CASADI_PREFIX(s7)
/* evaluate_stages */
solver_int32_default FORCESNLPsolver_model_1(const FORCESNLPsolver_float **arg, FORCESNLPsolver_float **res) 
{
    FORCESNLPsolver_float a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36,a37,a38,a39,a40,a41,a42,a43,a44,a45,a46,a47,a48,a49,a50,a51,a52,a53,a54,a55,a56,a57,a58,a59,a60,a61,a62,a63,a64,a65,a66,a67,a68,a69,a70,a71,a72,a73,a74,a75,a76,a77,a78,a79,a80,a81,a82,a83,a84,a85,a86,a87,a88,a89,a90,a91,a92,a93,a94,a95,a96,a97,a98,a99,a100,a101,a102,a103,a104,a105,a106,a107,a108,a109;
    a0=3.;
    a1=arg[1] ? arg[1][4] : 0;
    a2=(a0*a1);
    a3=arg[0] ? arg[0][7] : 0;
    a4=arg[1] ? arg[1][16] : 0;
    a5=(a3-a4);
    a5=(a2*a5);
    a6=(a3-a4);
    a7=(a5*a6);
    a8=2.;
    a9=arg[1] ? arg[1][5] : 0;
    a10=(a8*a9);
    a11=(a3-a4);
    a11=(a10*a11);
    a7=(a7+a11);
    a11=arg[1] ? arg[1][6] : 0;
    a7=(a7+a11);
    a12=arg[1] ? arg[1][18] : 0;
    a13=(a3-a12);
    a14=1.0000000000000001e-01;
    a13=(a13/a14);
    a13=exp(a13);
    a15=1.;
    a16=(a15+a13);
    a7=(a7/a16);
    a17=(1./a16);
    a18=(a15-a17);
    a19=arg[1] ? arg[1][12] : 0;
    a20=(a0*a19);
    a21=arg[1] ? arg[1][17] : 0;
    a22=(a3-a21);
    a22=(a20*a22);
    a23=(a3-a21);
    a24=(a22*a23);
    a25=arg[1] ? arg[1][13] : 0;
    a26=(a8*a25);
    a27=(a3-a21);
    a27=(a26*a27);
    a24=(a24+a27);
    a27=arg[1] ? arg[1][14] : 0;
    a24=(a24+a27);
    a28=(a18*a24);
    a28=(a7+a28);
    a29=arg[1] ? arg[1][0] : 0;
    a30=(a0*a29);
    a31=(a3-a4);
    a31=(a30*a31);
    a32=(a3-a4);
    a33=(a31*a32);
    a34=arg[1] ? arg[1][1] : 0;
    a35=(a8*a34);
    a36=(a3-a4);
    a36=(a35*a36);
    a33=(a33+a36);
    a36=arg[1] ? arg[1][2] : 0;
    a33=(a33+a36);
    a33=(a33/a16);
    a37=(a15-a17);
    a38=arg[1] ? arg[1][8] : 0;
    a39=(a0*a38);
    a40=(a3-a21);
    a40=(a39*a40);
    a41=(a3-a21);
    a42=(a40*a41);
    a43=arg[1] ? arg[1][9] : 0;
    a44=(a8*a43);
    a45=(a3-a21);
    a45=(a44*a45);
    a42=(a42+a45);
    a45=arg[1] ? arg[1][10] : 0;
    a42=(a42+a45);
    a46=(a37*a42);
    a46=(a33+a46);
    a47=sq(a46);
    a48=sq(a28);
    a47=(a47+a48);
    a47=sqrt(a47);
    a48=(a28/a47);
    a49=(a3-a4);
    a49=(a29*a49);
    a50=(a3-a4);
    a51=(a49*a50);
    a52=(a3-a4);
    a53=(a51*a52);
    a54=(a3-a4);
    a54=(a34*a54);
    a55=(a3-a4);
    a56=(a54*a55);
    a53=(a53+a56);
    a56=(a3-a4);
    a56=(a36*a56);
    a53=(a53+a56);
    a56=arg[1] ? arg[1][3] : 0;
    a53=(a53+a56);
    a53=(a53/a16);
    a57=(a15-a17);
    a58=(a3-a21);
    a58=(a38*a58);
    a59=(a3-a21);
    a60=(a58*a59);
    a61=(a3-a21);
    a62=(a60*a61);
    a63=(a3-a21);
    a63=(a43*a63);
    a64=(a3-a21);
    a65=(a63*a64);
    a62=(a62+a65);
    a65=(a3-a21);
    a65=(a45*a65);
    a62=(a62+a65);
    a65=arg[1] ? arg[1][11] : 0;
    a62=(a62+a65);
    a66=(a57*a62);
    a66=(a53+a66);
    a67=arg[0] ? arg[0][3] : 0;
    a68=(a67-a66);
    a69=(a48*a68);
    a70=(a46/a47);
    a71=(a3-a4);
    a71=(a1*a71);
    a72=(a3-a4);
    a73=(a71*a72);
    a74=(a3-a4);
    a75=(a73*a74);
    a76=(a3-a4);
    a76=(a9*a76);
    a77=(a3-a4);
    a78=(a76*a77);
    a75=(a75+a78);
    a78=(a3-a4);
    a78=(a11*a78);
    a75=(a75+a78);
    a78=arg[1] ? arg[1][7] : 0;
    a75=(a75+a78);
    a75=(a75/a16);
    a79=(a15-a17);
    a80=(a3-a21);
    a80=(a19*a80);
    a81=(a3-a21);
    a82=(a80*a81);
    a83=(a3-a21);
    a84=(a82*a83);
    a85=(a3-a21);
    a85=(a25*a85);
    a86=(a3-a21);
    a87=(a85*a86);
    a84=(a84+a87);
    a87=(a3-a21);
    a87=(a27*a87);
    a84=(a84+a87);
    a87=arg[1] ? arg[1][15] : 0;
    a84=(a84+a87);
    a88=(a79*a84);
    a88=(a75+a88);
    a89=arg[0] ? arg[0][4] : 0;
    a90=(a89-a88);
    a91=(a70*a90);
    a69=(a69-a91);
    a91=arg[1] ? arg[1][19] : 0;
    a92=(a91*a69);
    a93=(a92*a69);
    a66=(a67-a66);
    a94=(a70*a66);
    a88=(a89-a88);
    a95=(a48*a88);
    a94=(a94+a95);
    a95=arg[1] ? arg[1][20] : 0;
    a96=(a95*a94);
    a97=(a96*a94);
    a93=(a93+a97);
    a97=arg[0] ? arg[0][6] : 0;
    a98=arg[1] ? arg[1][23] : 0;
    a99=(a97-a98);
    a100=arg[1] ? arg[1][38] : 0;
    a99=(a100*a99);
    a98=(a97-a98);
    a101=(a99*a98);
    a93=(a93+a101);
    a101=arg[1] ? arg[1][21] : 0;
    a102=arg[0] ? arg[0][0] : 0;
    a103=(a101*a102);
    a104=(a103*a102);
    a93=(a93+a104);
    a104=arg[1] ? arg[1][22] : 0;
    a105=arg[0] ? arg[0][1] : 0;
    a106=(a104*a105);
    a107=(a106*a105);
    a93=(a93+a107);
    a107=arg[1] ? arg[1][24] : 0;
    a108=arg[0] ? arg[0][2] : 0;
    a109=(a107*a108);
    a93=(a93+a109);
    if (res[0]!=0) res[0][0]=a93;
    a101=(a101*a102);
    a103=(a103+a101);
    if (res[1]!=0) res[1][0]=a103;
    a104=(a104*a105);
    a106=(a106+a104);
    if (res[1]!=0) res[1][1]=a106;
    if (res[1]!=0) res[1][2]=a107;
    a95=(a95*a94);
    a96=(a96+a95);
    a95=(a70*a96);
    a91=(a91*a69);
    a92=(a92+a91);
    a91=(a48*a92);
    a69=(a95+a91);
    if (res[1]!=0) res[1][3]=a69;
    a69=(a48*a96);
    a94=(a70*a92);
    a107=(a69-a94);
    if (res[1]!=0) res[1][4]=a107;
    a100=(a100*a98);
    a99=(a99+a100);
    if (res[1]!=0) res[1][5]=a99;
    a94=(a94-a69);
    a79=(a79*a94);
    a69=(a27*a79);
    a85=(a85*a79);
    a69=(a69+a85);
    a86=(a86*a79);
    a86=(a25*a86);
    a69=(a69+a86);
    a82=(a82*a79);
    a69=(a69+a82);
    a83=(a83*a79);
    a80=(a80*a83);
    a69=(a69+a80);
    a81=(a81*a83);
    a81=(a19*a81);
    a69=(a69+a81);
    a81=(a94/a16);
    a83=(a11*a81);
    a69=(a69+a83);
    a76=(a76*a81);
    a69=(a69+a76);
    a77=(a77*a81);
    a77=(a9*a77);
    a69=(a69+a77);
    a73=(a73*a81);
    a69=(a69+a73);
    a74=(a74*a81);
    a71=(a71*a74);
    a69=(a69+a71);
    a72=(a72*a74);
    a72=(a1*a72);
    a69=(a69+a72);
    a95=(a95+a91);
    a57=(a57*a95);
    a91=(a45*a57);
    a69=(a69-a91);
    a63=(a63*a57);
    a69=(a69-a63);
    a64=(a64*a57);
    a64=(a43*a64);
    a69=(a69-a64);
    a60=(a60*a57);
    a69=(a69-a60);
    a61=(a61*a57);
    a58=(a58*a61);
    a69=(a69-a58);
    a59=(a59*a61);
    a59=(a38*a59);
    a69=(a69-a59);
    a59=(a95/a16);
    a61=(a36*a59);
    a69=(a69-a61);
    a54=(a54*a59);
    a69=(a69-a54);
    a55=(a55*a59);
    a55=(a34*a55);
    a69=(a69-a55);
    a51=(a51*a59);
    a69=(a69-a51);
    a52=(a52*a59);
    a49=(a49*a52);
    a69=(a69-a49);
    a50=(a50*a52);
    a50=(a29*a50);
    a69=(a69-a50);
    a66=(a66*a96);
    a90=(a90*a92);
    a66=(a66-a90);
    a90=(a66/a47);
    a46=(a46+a46);
    a70=(a70/a47);
    a70=(a70*a66);
    a48=(a48/a47);
    a88=(a88*a96);
    a68=(a68*a92);
    a88=(a88+a68);
    a48=(a48*a88);
    a70=(a70+a48);
    a48=(a47+a47);
    a70=(a70/a48);
    a46=(a46*a70);
    a90=(a90-a46);
    a37=(a37*a90);
    a44=(a44*a37);
    a69=(a69+a44);
    a40=(a40*a37);
    a69=(a69+a40);
    a41=(a41*a37);
    a39=(a39*a41);
    a69=(a69+a39);
    a39=(a90/a16);
    a35=(a35*a39);
    a69=(a69+a35);
    a31=(a31*a39);
    a69=(a69+a31);
    a32=(a32*a39);
    a30=(a30*a32);
    a69=(a69+a30);
    a88=(a88/a47);
    a28=(a28+a28);
    a28=(a28*a70);
    a88=(a88-a28);
    a18=(a18*a88);
    a26=(a26*a18);
    a69=(a69+a26);
    a22=(a22*a18);
    a69=(a69+a22);
    a23=(a23*a18);
    a20=(a20*a23);
    a69=(a69+a20);
    a53=(a53/a16);
    a53=(a53*a95);
    a75=(a75/a16);
    a75=(a75*a94);
    a53=(a53-a75);
    a33=(a33/a16);
    a33=(a33*a90);
    a53=(a53-a33);
    a17=(a17/a16);
    a62=(a62*a95);
    a84=(a84*a94);
    a62=(a62-a84);
    a42=(a42*a90);
    a62=(a62-a42);
    a24=(a24*a88);
    a62=(a62-a24);
    a17=(a17*a62);
    a53=(a53-a17);
    a7=(a7/a16);
    a7=(a7*a88);
    a53=(a53-a7);
    a13=(a13*a53);
    a53=10.;
    a13=(a53*a13);
    a69=(a69+a13);
    a88=(a88/a16);
    a10=(a10*a88);
    a69=(a69+a10);
    a5=(a5*a88);
    a69=(a69+a5);
    a6=(a6*a88);
    a2=(a2*a6);
    a69=(a69+a2);
    if (res[1]!=0) res[1][6]=a69;
    a69=(a0*a29);
    a2=(a3-a4);
    a2=(a69*a2);
    a6=(a3-a4);
    a88=(a2*a6);
    a5=(a8*a34);
    a10=(a3-a4);
    a10=(a5*a10);
    a88=(a88+a10);
    a88=(a88+a36);
    a12=(a3-a12);
    a12=(a12/a14);
    a12=exp(a12);
    a10=(a15+a12);
    a88=(a88/a10);
    a16=(1./a10);
    a13=(a15-a16);
    a7=(a0*a38);
    a17=(a3-a21);
    a17=(a7*a17);
    a62=(a3-a21);
    a24=(a17*a62);
    a42=(a8*a43);
    a90=(a3-a21);
    a90=(a42*a90);
    a24=(a24+a90);
    a24=(a24+a45);
    a90=(a13*a24);
    a90=(a88+a90);
    a84=sq(a90);
    a94=(a0*a1);
    a95=(a3-a4);
    a95=(a94*a95);
    a33=(a3-a4);
    a75=(a95*a33);
    a20=(a8*a9);
    a23=(a3-a4);
    a23=(a20*a23);
    a75=(a75+a23);
    a75=(a75+a11);
    a75=(a75/a10);
    a23=(a15-a16);
    a0=(a0*a19);
    a18=(a3-a21);
    a18=(a0*a18);
    a22=(a3-a21);
    a26=(a18*a22);
    a28=(a8*a25);
    a70=(a3-a21);
    a70=(a28*a70);
    a26=(a26+a70);
    a26=(a26+a27);
    a70=(a23*a26);
    a70=(a75+a70);
    a47=sq(a70);
    a84=(a84+a47);
    a84=sqrt(a84);
    a47=(a90/a84);
    a30=(a3-a4);
    a30=(a1*a30);
    a32=(a3-a4);
    a39=(a30*a32);
    a31=(a3-a4);
    a35=(a39*a31);
    a41=(a3-a4);
    a41=(a9*a41);
    a37=(a3-a4);
    a40=(a41*a37);
    a35=(a35+a40);
    a40=(a3-a4);
    a40=(a11*a40);
    a35=(a35+a40);
    a35=(a35+a78);
    a35=(a35/a10);
    a78=(a15-a16);
    a40=(a3-a21);
    a40=(a19*a40);
    a44=(a3-a21);
    a46=(a40*a44);
    a48=(a3-a21);
    a68=(a46*a48);
    a92=(a3-a21);
    a92=(a25*a92);
    a96=(a3-a21);
    a66=(a92*a96);
    a68=(a68+a66);
    a66=(a3-a21);
    a66=(a27*a66);
    a68=(a68+a66);
    a68=(a68+a87);
    a87=(a78*a68);
    a87=(a35+a87);
    a87=(a89-a87);
    a66=(a47*a87);
    a50=(a70/a84);
    a52=(a3-a4);
    a52=(a29*a52);
    a49=(a3-a4);
    a59=(a52*a49);
    a51=(a3-a4);
    a55=(a59*a51);
    a54=(a3-a4);
    a54=(a34*a54);
    a61=(a3-a4);
    a58=(a54*a61);
    a55=(a55+a58);
    a4=(a3-a4);
    a4=(a36*a4);
    a55=(a55+a4);
    a55=(a55+a56);
    a55=(a55/a10);
    a56=(a15-a16);
    a4=(a3-a21);
    a4=(a38*a4);
    a58=(a3-a21);
    a57=(a4*a58);
    a60=(a3-a21);
    a64=(a57*a60);
    a63=(a3-a21);
    a63=(a43*a63);
    a91=(a3-a21);
    a72=(a63*a91);
    a64=(a64+a72);
    a21=(a3-a21);
    a21=(a45*a21);
    a64=(a64+a21);
    a64=(a64+a65);
    a65=(a56*a64);
    a65=(a55+a65);
    a65=(a67-a65);
    a21=(a50*a65);
    a66=(a66-a21);
    a21=(a66-a108);
    if (res[2]!=0) res[2][0]=a21;
    a66=(a66+a108);
    a66=(-a66);
    if (res[2]!=0) res[2][1]=a66;
    a66=-1.;
    if (res[3]!=0) res[3][0]=a66;
    if (res[3]!=0) res[3][1]=a66;
    a66=(-a50);
    if (res[3]!=0) res[3][2]=a66;
    if (res[3]!=0) res[3][3]=a50;
    if (res[3]!=0) res[3][4]=a47;
    a66=(-a47);
    if (res[3]!=0) res[3][5]=a66;
    a6=(a6*a69);
    a6=(a6+a2);
    a6=(a6+a5);
    a6=(a6/a10);
    a88=(a88/a10);
    a53=(a53*a12);
    a88=(a88*a53);
    a6=(a6-a88);
    a16=(a16/a10);
    a16=(a16*a53);
    a24=(a24*a16);
    a62=(a62*a7);
    a62=(a62+a17);
    a62=(a62+a42);
    a13=(a13*a62);
    a24=(a24+a13);
    a6=(a6+a24);
    a24=(a6/a84);
    a13=(a47/a84);
    a90=(a90+a90);
    a90=(a90*a6);
    a70=(a70+a70);
    a33=(a33*a94);
    a33=(a33+a95);
    a33=(a33+a20);
    a33=(a33/a10);
    a75=(a75/a10);
    a75=(a75*a53);
    a33=(a33-a75);
    a26=(a26*a16);
    a22=(a22*a0);
    a22=(a22+a18);
    a22=(a22+a28);
    a23=(a23*a22);
    a26=(a26+a23);
    a33=(a33+a26);
    a70=(a70*a33);
    a90=(a90+a70);
    a70=(a84+a84);
    a90=(a90/a70);
    a13=(a13*a90);
    a24=(a24-a13);
    a87=(a87*a24);
    a32=(a32*a1);
    a32=(a32+a30);
    a31=(a31*a32);
    a31=(a31+a39);
    a37=(a37*a9);
    a37=(a37+a41);
    a31=(a31+a37);
    a31=(a31+a11);
    a31=(a31/a10);
    a35=(a35/a10);
    a35=(a35*a53);
    a31=(a31-a35);
    a68=(a68*a16);
    a44=(a44*a19);
    a44=(a44+a40);
    a48=(a48*a44);
    a48=(a48+a46);
    a96=(a96*a25);
    a96=(a96+a92);
    a48=(a48+a96);
    a48=(a48+a27);
    a78=(a78*a48);
    a68=(a68+a78);
    a31=(a31+a68);
    a47=(a47*a31);
    a87=(a87-a47);
    a33=(a33/a84);
    a84=(a50/a84);
    a84=(a84*a90);
    a33=(a33-a84);
    a65=(a65*a33);
    a49=(a49*a29);
    a49=(a49+a52);
    a51=(a51*a49);
    a51=(a51+a59);
    a61=(a61*a34);
    a61=(a61+a54);
    a51=(a51+a61);
    a51=(a51+a36);
    a51=(a51/a10);
    a55=(a55/a10);
    a55=(a55*a53);
    a51=(a51-a55);
    a64=(a64*a16);
    a58=(a58*a38);
    a58=(a58+a4);
    a60=(a60*a58);
    a60=(a60+a57);
    a91=(a91*a43);
    a91=(a91+a63);
    a60=(a60+a91);
    a60=(a60+a45);
    a56=(a56*a60);
    a64=(a64+a56);
    a51=(a51+a64);
    a50=(a50*a51);
    a65=(a65-a50);
    a87=(a87-a65);
    if (res[3]!=0) res[3][6]=a87;
    a87=(-a87);
    if (res[3]!=0) res[3][7]=a87;
    a87=tan(a105);
    a65=5.0000000000000000e-01;
    a87=(a65*a87);
    a50=atan(a87);
    a51=arg[0] ? arg[0][5] : 0;
    a64=(a51+a50);
    a56=cos(a64);
    a60=(a97*a56);
    a45=5.0000000000000003e-02;
    a91=(a45*a102);
    a91=(a97+a91);
    a63=1.5000000000000000e+00;
    a43=(a97/a63);
    a57=sin(a50);
    a58=(a43*a57);
    a4=(a45*a58);
    a4=(a51+a4);
    a38=tan(a105);
    a38=(a65*a38);
    a16=atan(a38);
    a55=(a4+a16);
    a53=cos(a55);
    a10=(a91*a53);
    a10=(a8*a10);
    a60=(a60+a10);
    a10=(a45*a102);
    a10=(a97+a10);
    a36=(a91/a63);
    a61=sin(a16);
    a54=(a36*a61);
    a34=(a45*a54);
    a34=(a51+a34);
    a59=tan(a105);
    a59=(a65*a59);
    a49=atan(a59);
    a52=(a34+a49);
    a29=cos(a52);
    a33=(a10*a29);
    a33=(a8*a33);
    a60=(a60+a33);
    a33=(a14*a102);
    a33=(a97+a33);
    a84=(a10/a63);
    a90=sin(a49);
    a47=(a84*a90);
    a31=(a14*a47);
    a31=(a51+a31);
    a68=tan(a105);
    a68=(a65*a68);
    a78=atan(a68);
    a48=(a31+a78);
    a27=cos(a48);
    a96=(a33*a27);
    a60=(a60+a96);
    a96=1.6666666666666666e-02;
    a60=(a96*a60);
    a67=(a67+a60);
    if (res[4]!=0) res[4][0]=a67;
    a67=(a51+a50);
    a60=sin(a67);
    a92=(a97*a60);
    a4=(a4+a16);
    a25=sin(a4);
    a46=(a91*a25);
    a46=(a8*a46);
    a92=(a92+a46);
    a34=(a34+a49);
    a46=sin(a34);
    a44=(a10*a46);
    a44=(a8*a44);
    a92=(a92+a44);
    a31=(a31+a78);
    a44=sin(a31);
    a40=(a33*a44);
    a92=(a92+a40);
    a92=(a96*a92);
    a89=(a89+a92);
    if (res[4]!=0) res[4][1]=a89;
    a54=(a8*a54);
    a58=(a58+a54);
    a47=(a8*a47);
    a58=(a58+a47);
    a63=(a33/a63);
    a47=sin(a78);
    a54=(a63*a47);
    a58=(a58+a54);
    a58=(a96*a58);
    a51=(a51+a58);
    if (res[4]!=0) res[4][2]=a51;
    a51=(a8*a102);
    a51=(a102+a51);
    a58=(a8*a102);
    a51=(a51+a58);
    a51=(a51+a102);
    a51=(a96*a51);
    a51=(a97+a51);
    if (res[4]!=0) res[4][3]=a51;
    a51=(a8*a91);
    a51=(a97+a51);
    a102=(a8*a10);
    a51=(a51+a102);
    a51=(a51+a33);
    a51=(a96*a51);
    a3=(a3+a51);
    if (res[4]!=0) res[4][4]=a3;
    a3=(a8*a108);
    a3=(a108+a3);
    a51=(a8*a108);
    a3=(a3+a51);
    a3=(a3+a108);
    a3=(a96*a3);
    a108=arg[0] ? arg[0][8] : 0;
    a108=(a108+a3);
    if (res[4]!=0) res[4][5]=a108;
    a108=(a45*a53);
    a108=(a8*a108);
    a3=(a45*a29);
    a51=3.3333333333333333e-02;
    a102=(a51*a61);
    a58=(a45*a102);
    a52=sin(a52);
    a54=(a52*a58);
    a54=(a10*a54);
    a3=(a3-a54);
    a3=(a8*a3);
    a108=(a108+a3);
    a3=(a14*a27);
    a51=(a51*a90);
    a54=(a14*a51);
    a48=sin(a48);
    a89=(a48*a54);
    a89=(a33*a89);
    a3=(a3-a89);
    a108=(a108+a3);
    a108=(a96*a108);
    if (res[5]!=0) res[5][0]=a108;
    a108=(a45*a25);
    a108=(a8*a108);
    a3=(a45*a46);
    a34=cos(a34);
    a58=(a34*a58);
    a58=(a10*a58);
    a3=(a3+a58);
    a3=(a8*a3);
    a108=(a108+a3);
    a3=(a14*a44);
    a31=cos(a31);
    a54=(a31*a54);
    a54=(a33*a54);
    a3=(a3+a54);
    a108=(a108+a3);
    a108=(a96*a108);
    if (res[5]!=0) res[5][1]=a108;
    a102=(a8*a102);
    a51=(a8*a51);
    a102=(a102+a51);
    a51=6.6666666666666666e-02;
    a51=(a51*a47);
    a102=(a102+a51);
    a102=(a96*a102);
    if (res[5]!=0) res[5][2]=a102;
    if (res[5]!=0) res[5][3]=a14;
    a102=5.0000000000000010e-03;
    if (res[5]!=0) res[5][4]=a102;
    a102=cos(a105);
    a102=sq(a102);
    a102=(a65/a102);
    a87=sq(a87);
    a87=(a15+a87);
    a102=(a102/a87);
    a64=sin(a64);
    a87=(a64*a102);
    a87=(a97*a87);
    a50=cos(a50);
    a50=(a50*a102);
    a43=(a43*a50);
    a50=(a45*a43);
    a51=cos(a105);
    a51=sq(a51);
    a51=(a65/a51);
    a38=sq(a38);
    a38=(a15+a38);
    a51=(a51/a38);
    a38=(a50+a51);
    a55=sin(a55);
    a38=(a55*a38);
    a38=(a91*a38);
    a38=(a8*a38);
    a87=(a87+a38);
    a16=cos(a16);
    a16=(a16*a51);
    a36=(a36*a16);
    a16=(a45*a36);
    a38=cos(a105);
    a38=sq(a38);
    a38=(a65/a38);
    a59=sq(a59);
    a59=(a15+a59);
    a38=(a38/a59);
    a59=(a16+a38);
    a59=(a52*a59);
    a59=(a10*a59);
    a59=(a8*a59);
    a87=(a87+a59);
    a49=cos(a49);
    a49=(a49*a38);
    a84=(a84*a49);
    a49=(a14*a84);
    a105=cos(a105);
    a105=sq(a105);
    a65=(a65/a105);
    a68=sq(a68);
    a68=(a15+a68);
    a65=(a65/a68);
    a68=(a49+a65);
    a68=(a48*a68);
    a68=(a33*a68);
    a87=(a87+a68);
    a87=(a96*a87);
    a87=(-a87);
    if (res[5]!=0) res[5][5]=a87;
    a67=cos(a67);
    a102=(a67*a102);
    a102=(a97*a102);
    a50=(a50+a51);
    a4=cos(a4);
    a50=(a4*a50);
    a50=(a91*a50);
    a50=(a8*a50);
    a102=(a102+a50);
    a16=(a16+a38);
    a16=(a34*a16);
    a16=(a10*a16);
    a16=(a8*a16);
    a102=(a102+a16);
    a49=(a49+a65);
    a49=(a31*a49);
    a49=(a33*a49);
    a102=(a102+a49);
    a102=(a96*a102);
    if (res[5]!=0) res[5][6]=a102;
    a36=(a8*a36);
    a43=(a43+a36);
    a84=(a8*a84);
    a43=(a43+a84);
    a78=cos(a78);
    a78=(a78*a65);
    a63=(a63*a78);
    a43=(a43+a63);
    a43=(a96*a43);
    if (res[5]!=0) res[5][7]=a43;
    if (res[5]!=0) res[5][8]=a14;
    if (res[5]!=0) res[5][9]=a15;
    if (res[5]!=0) res[5][10]=a15;
    a64=(a97*a64);
    a43=(a91*a55);
    a43=(a8*a43);
    a64=(a64+a43);
    a43=(a10*a52);
    a43=(a8*a43);
    a64=(a64+a43);
    a43=(a33*a48);
    a64=(a64+a43);
    a64=(a96*a64);
    a64=(-a64);
    if (res[5]!=0) res[5][11]=a64;
    a97=(a97*a67);
    a67=(a91*a4);
    a67=(a8*a67);
    a97=(a97+a67);
    a67=(a10*a34);
    a67=(a8*a67);
    a97=(a97+a67);
    a67=(a33*a31);
    a97=(a97+a67);
    a97=(a96*a97);
    if (res[5]!=0) res[5][12]=a97;
    if (res[5]!=0) res[5][13]=a15;
    a97=6.6666666666666663e-01;
    a57=(a97*a57);
    a67=(a45*a57);
    a55=(a55*a67);
    a55=(a91*a55);
    a53=(a53-a55);
    a53=(a8*a53);
    a56=(a56+a53);
    a61=(a97*a61);
    a45=(a45*a61);
    a52=(a52*a45);
    a52=(a10*a52);
    a29=(a29-a52);
    a29=(a8*a29);
    a56=(a56+a29);
    a90=(a97*a90);
    a29=(a14*a90);
    a48=(a48*a29);
    a48=(a33*a48);
    a27=(a27-a48);
    a56=(a56+a27);
    a56=(a96*a56);
    if (res[5]!=0) res[5][14]=a56;
    a4=(a4*a67);
    a91=(a91*a4);
    a25=(a25+a91);
    a25=(a8*a25);
    a60=(a60+a25);
    a34=(a34*a45);
    a10=(a10*a34);
    a46=(a46+a10);
    a46=(a8*a46);
    a60=(a60+a46);
    a31=(a31*a29);
    a33=(a33*a31);
    a44=(a44+a33);
    a60=(a60+a44);
    a60=(a96*a60);
    if (res[5]!=0) res[5][15]=a60;
    a61=(a8*a61);
    a57=(a57+a61);
    a8=(a8*a90);
    a57=(a57+a8);
    a97=(a97*a47);
    a57=(a57+a97);
    a96=(a96*a57);
    if (res[5]!=0) res[5][16]=a96;
    if (res[5]!=0) res[5][17]=a15;
    if (res[5]!=0) res[5][18]=a14;
    if (res[5]!=0) res[5][19]=a15;
    if (res[5]!=0) res[5][20]=a15;
    return 0;
}

solver_int32_default FORCESNLPsolver_model_1_init(solver_int32_default *f_type, solver_int32_default *n_in, solver_int32_default *n_out, solver_int32_default *sz_arg, solver_int32_default *sz_res) 
{
    *f_type = 1;
    *n_in = 2;
    *n_out = 6;
    *sz_arg = 2;
    *sz_res = 6;
    return 0;
}

solver_int32_default FORCESNLPsolver_model_1_sparsity(solver_int32_default i, solver_int32_default *nrow, solver_int32_default *ncol, const solver_int32_default **colind, const solver_int32_default **row) 
{
    const solver_int32_default *s;
    switch (i) 
    {
      case 0:
        s = s0;
        break;
      case 1:
        s = s1;
        break;
      case 2:
        s = s2;
        break;
      case 3:
        s = s3;
        break;
      case 4:
        s = s4;
        break;
      case 5:
        s = s5;
        break;
      case 6:
        s = s6;
        break;
      case 7:
        s = s7;
        break;
      default:
        return 1;
    }
    
    *nrow = s[0];
    *ncol = s[1];
    *colind = s + 2;
    *row = s + 2 + (*ncol + 1);
    return 0;
}

solver_int32_default FORCESNLPsolver_model_1_work(solver_int32_default *sz_iw, solver_int32_default *sz_w) 
{
    if (sz_iw) *sz_iw = 0;
    if (sz_w) *sz_w = 110;
    return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
