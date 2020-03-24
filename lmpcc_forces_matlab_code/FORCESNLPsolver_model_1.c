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

static const solver_int32_default CASADI_PREFIX(s0)[] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
#define s0 CASADI_PREFIX(s0)
static const solver_int32_default CASADI_PREFIX(s1)[] = {40, 1, 0, 40, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
#define s1 CASADI_PREFIX(s1)
static const solver_int32_default CASADI_PREFIX(s2)[] = {1, 1, 0, 1, 0};
#define s2 CASADI_PREFIX(s2)
static const solver_int32_default CASADI_PREFIX(s3)[] = {1, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0};
#define s3 CASADI_PREFIX(s3)
static const solver_int32_default CASADI_PREFIX(s4)[] = {2, 1, 0, 2, 0, 1};
#define s4 CASADI_PREFIX(s4)
static const solver_int32_default CASADI_PREFIX(s5)[] = {2, 7, 0, 0, 0, 2, 4, 6, 8, 8, 0, 1, 0, 1, 0, 1, 0, 1};
#define s5 CASADI_PREFIX(s5)
static const solver_int32_default CASADI_PREFIX(s6)[] = {4, 1, 0, 4, 0, 1, 2, 3};
#define s6 CASADI_PREFIX(s6)
static const solver_int32_default CASADI_PREFIX(s7)[] = {4, 7, 0, 3, 6, 6, 7, 8, 11, 12, 0, 1, 3, 0, 1, 2, 0, 1, 0, 1, 2, 3};
#define s7 CASADI_PREFIX(s7)
/* evaluate_stages */
solver_int32_default FORCESNLPsolver_model_1(const FORCESNLPsolver_float **arg, FORCESNLPsolver_float **res) 
{
    FORCESNLPsolver_float a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27,a28,a29,a30,a31,a32,a33,a34,a35,a36,a37,a38,a39,a40,a41,a42,a43,a44,a45,a46,a47,a48,a49,a50,a51,a52,a53,a54,a55,a56,a57,a58,a59,a60,a61,a62,a63,a64,a65,a66,a67,a68,a69,a70,a71,a72,a73,a74,a75,a76,a77,a78,a79,a80,a81,a82,a83,a84,a85,a86,a87,a88,a89,a90,a91,a92,a93,a94,a95,a96,a97,a98,a99,a100,a101,a102,a103,a104,a105,a106,a107,a108,a109,a110,a111,a112,a113,a114,a115,a116,a117,a118,a119,a120,a121,a122,a123,a124,a125,a126,a127,a128,a129,a130,a131,a132,a133,a134,a135,a136,a137,a138;
    a0=3.;
    a1=arg[1] ? arg[1][4] : 0;
    a2=(a0*a1);
    a3=arg[0] ? arg[0][6] : 0;
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
    a12=(a3-a12);
    a13=1.0000000000000001e-01;
    a12=(a12/a13);
    a12=exp(a12);
    a13=1.;
    a14=(a13+a12);
    a7=(a7/a14);
    a15=(1./a14);
    a16=(a13-a15);
    a17=arg[1] ? arg[1][12] : 0;
    a18=(a0*a17);
    a19=arg[1] ? arg[1][17] : 0;
    a20=(a3-a19);
    a20=(a18*a20);
    a21=(a3-a19);
    a22=(a20*a21);
    a23=arg[1] ? arg[1][13] : 0;
    a24=(a8*a23);
    a25=(a3-a19);
    a25=(a24*a25);
    a22=(a22+a25);
    a25=arg[1] ? arg[1][14] : 0;
    a22=(a22+a25);
    a26=(a16*a22);
    a26=(a7+a26);
    a27=arg[1] ? arg[1][0] : 0;
    a28=(a0*a27);
    a29=(a3-a4);
    a29=(a28*a29);
    a30=(a3-a4);
    a31=(a29*a30);
    a32=arg[1] ? arg[1][1] : 0;
    a33=(a8*a32);
    a34=(a3-a4);
    a34=(a33*a34);
    a31=(a31+a34);
    a34=arg[1] ? arg[1][2] : 0;
    a31=(a31+a34);
    a31=(a31/a14);
    a35=(a13-a15);
    a36=arg[1] ? arg[1][8] : 0;
    a0=(a0*a36);
    a37=(a3-a19);
    a37=(a0*a37);
    a38=(a3-a19);
    a39=(a37*a38);
    a40=arg[1] ? arg[1][9] : 0;
    a41=(a8*a40);
    a42=(a3-a19);
    a42=(a41*a42);
    a39=(a39+a42);
    a42=arg[1] ? arg[1][10] : 0;
    a39=(a39+a42);
    a43=(a35*a39);
    a43=(a31+a43);
    a44=sq(a43);
    a45=sq(a26);
    a44=(a44+a45);
    a44=sqrt(a44);
    a45=(a26/a44);
    a46=(a3-a4);
    a46=(a27*a46);
    a47=(a3-a4);
    a48=(a46*a47);
    a49=(a3-a4);
    a50=(a48*a49);
    a51=(a3-a4);
    a51=(a32*a51);
    a52=(a3-a4);
    a53=(a51*a52);
    a50=(a50+a53);
    a53=(a3-a4);
    a53=(a34*a53);
    a50=(a50+a53);
    a53=arg[1] ? arg[1][3] : 0;
    a50=(a50+a53);
    a50=(a50/a14);
    a53=(a13-a15);
    a54=(a3-a19);
    a54=(a36*a54);
    a55=(a3-a19);
    a56=(a54*a55);
    a57=(a3-a19);
    a58=(a56*a57);
    a59=(a3-a19);
    a59=(a40*a59);
    a60=(a3-a19);
    a61=(a59*a60);
    a58=(a58+a61);
    a61=(a3-a19);
    a61=(a42*a61);
    a58=(a58+a61);
    a61=arg[1] ? arg[1][11] : 0;
    a58=(a58+a61);
    a61=(a53*a58);
    a61=(a50+a61);
    a62=arg[0] ? arg[0][3] : 0;
    a63=(a62-a61);
    a64=(a45*a63);
    a65=(a43/a44);
    a66=(a3-a4);
    a66=(a1*a66);
    a67=(a3-a4);
    a68=(a66*a67);
    a69=(a3-a4);
    a70=(a68*a69);
    a71=(a3-a4);
    a71=(a9*a71);
    a72=(a3-a4);
    a73=(a71*a72);
    a70=(a70+a73);
    a4=(a3-a4);
    a4=(a11*a4);
    a70=(a70+a4);
    a4=arg[1] ? arg[1][7] : 0;
    a70=(a70+a4);
    a70=(a70/a14);
    a4=(a13-a15);
    a73=(a3-a19);
    a73=(a17*a73);
    a74=(a3-a19);
    a75=(a73*a74);
    a76=(a3-a19);
    a77=(a75*a76);
    a78=(a3-a19);
    a78=(a23*a78);
    a79=(a3-a19);
    a80=(a78*a79);
    a77=(a77+a80);
    a19=(a3-a19);
    a19=(a25*a19);
    a77=(a77+a19);
    a19=arg[1] ? arg[1][15] : 0;
    a77=(a77+a19);
    a19=(a4*a77);
    a19=(a70+a19);
    a80=arg[0] ? arg[0][4] : 0;
    a81=(a80-a19);
    a82=(a65*a81);
    a64=(a64-a82);
    a82=arg[1] ? arg[1][19] : 0;
    a83=(a82*a64);
    a84=(a83*a64);
    a61=(a62-a61);
    a85=(a65*a61);
    a19=(a80-a19);
    a86=(a45*a19);
    a85=(a85+a86);
    a86=arg[1] ? arg[1][20] : 0;
    a87=(a86*a85);
    a88=(a87*a85);
    a84=(a84+a88);
    a88=arg[0] ? arg[0][0] : 0;
    a89=arg[1] ? arg[1][22] : 0;
    a90=(a88-a89);
    a91=arg[1] ? arg[1][25] : 0;
    a90=(a91*a90);
    a89=(a88-a89);
    a92=(a90*a89);
    a84=(a84+a92);
    a92=arg[1] ? arg[1][21] : 0;
    a93=arg[0] ? arg[0][1] : 0;
    a94=(a92*a93);
    a95=(a94*a93);
    a84=(a84+a95);
    a95=arg[1] ? arg[1][23] : 0;
    a96=arg[0] ? arg[0][2] : 0;
    a97=(a95*a96);
    a98=(a97*a96);
    a84=(a84+a98);
    a98=arg[0] ? arg[0][5] : 0;
    a99=cos(a98);
    a100=arg[1] ? arg[1][27] : 0;
    a99=(a99*a100);
    a99=(a62+a99);
    a101=arg[1] ? arg[1][28] : 0;
    a102=(a99-a101);
    a103=arg[1] ? arg[1][30] : 0;
    a104=cos(a103);
    a105=(a102*a104);
    a106=sin(a98);
    a106=(a106*a100);
    a106=(a80+a106);
    a107=arg[1] ? arg[1][29] : 0;
    a108=(a106-a107);
    a109=sin(a103);
    a110=(a108*a109);
    a105=(a105-a110);
    a110=arg[1] ? arg[1][31] : 0;
    a111=arg[1] ? arg[1][26] : 0;
    a112=(a110+a111);
    a112=sq(a112);
    a105=(a105/a112);
    a113=(a105*a104);
    a114=sin(a103);
    a115=(a102*a114);
    a116=cos(a103);
    a117=(a108*a116);
    a115=(a115+a117);
    a117=arg[1] ? arg[1][32] : 0;
    a118=(a117+a111);
    a118=sq(a118);
    a115=(a115/a118);
    a119=(a115*a114);
    a113=(a113+a119);
    a119=(a113*a102);
    a115=(a115*a116);
    a105=(a105*a109);
    a115=(a115-a105);
    a105=(a115*a108);
    a119=(a119+a105);
    a119=(a13-a119);
    a105=sq(a119);
    a120=1.0000000000000000e-03;
    a105=(a105+a120);
    a121=(1./a105);
    a122=arg[1] ? arg[1][33] : 0;
    a99=(a99-a122);
    a123=arg[1] ? arg[1][35] : 0;
    a124=cos(a123);
    a125=(a99*a124);
    a126=arg[1] ? arg[1][34] : 0;
    a106=(a106-a126);
    a127=sin(a123);
    a128=(a106*a127);
    a125=(a125-a128);
    a128=arg[1] ? arg[1][36] : 0;
    a129=(a128+a111);
    a129=sq(a129);
    a125=(a125/a129);
    a130=(a125*a124);
    a131=sin(a123);
    a132=(a99*a131);
    a133=cos(a123);
    a134=(a106*a133);
    a132=(a132+a134);
    a134=arg[1] ? arg[1][37] : 0;
    a135=(a134+a111);
    a135=sq(a135);
    a132=(a132/a135);
    a136=(a132*a131);
    a130=(a130+a136);
    a136=(a130*a99);
    a132=(a132*a133);
    a125=(a125*a127);
    a132=(a132-a125);
    a125=(a132*a106);
    a136=(a136+a125);
    a136=(a13-a136);
    a125=sq(a136);
    a125=(a125+a120);
    a120=(1./a125);
    a137=(a121+a120);
    a138=arg[1] ? arg[1][24] : 0;
    a137=(a138*a137);
    a84=(a84+a137);
    if (res[0]!=0) res[0][0]=a84;
    a91=(a91*a89);
    a90=(a90+a91);
    if (res[1]!=0) res[1][0]=a90;
    a92=(a92*a93);
    a94=(a94+a92);
    if (res[1]!=0) res[1][1]=a94;
    a95=(a95*a96);
    a97=(a97+a95);
    if (res[1]!=0) res[1][2]=a97;
    a136=(a136+a136);
    a120=(a120/a125);
    a120=(a120*a138);
    a136=(a136*a120);
    a130=(a130*a136);
    a106=(a106*a136);
    a120=(a133*a106);
    a99=(a99*a136);
    a125=(a131*a99);
    a120=(a120+a125);
    a120=(a120/a135);
    a131=(a131*a120);
    a130=(a130+a131);
    a99=(a124*a99);
    a106=(a127*a106);
    a99=(a99-a106);
    a99=(a99/a129);
    a124=(a124*a99);
    a130=(a130+a124);
    a119=(a119+a119);
    a121=(a121/a105);
    a121=(a121*a138);
    a119=(a119*a121);
    a113=(a113*a119);
    a108=(a108*a119);
    a121=(a116*a108);
    a102=(a102*a119);
    a138=(a114*a102);
    a121=(a121+a138);
    a121=(a121/a118);
    a114=(a114*a121);
    a113=(a113+a114);
    a102=(a104*a102);
    a108=(a109*a108);
    a102=(a102-a108);
    a102=(a102/a112);
    a104=(a104*a102);
    a113=(a113+a104);
    a130=(a130+a113);
    a86=(a86*a85);
    a87=(a87+a86);
    a86=(a65*a87);
    a85=(a130+a86);
    a82=(a82*a64);
    a83=(a83+a82);
    a82=(a45*a83);
    a85=(a85+a82);
    if (res[1]!=0) res[1][3]=a85;
    a132=(a132*a136);
    a133=(a133*a120);
    a132=(a132+a133);
    a127=(a127*a99);
    a132=(a132-a127);
    a115=(a115*a119);
    a116=(a116*a121);
    a115=(a115+a116);
    a109=(a109*a102);
    a115=(a115-a109);
    a132=(a132+a115);
    a115=(a45*a87);
    a109=(a132+a115);
    a102=(a65*a83);
    a109=(a109-a102);
    if (res[1]!=0) res[1][4]=a109;
    a132=(a100*a132);
    a109=cos(a98);
    a109=(a109*a132);
    a130=(a100*a130);
    a132=sin(a98);
    a132=(a132*a130);
    a109=(a109-a132);
    if (res[1]!=0) res[1][5]=a109;
    a102=(a102-a115);
    a4=(a4*a102);
    a25=(a25*a4);
    a78=(a78*a4);
    a25=(a25+a78);
    a79=(a79*a4);
    a23=(a23*a79);
    a25=(a25+a23);
    a75=(a75*a4);
    a25=(a25+a75);
    a76=(a76*a4);
    a73=(a73*a76);
    a25=(a25+a73);
    a74=(a74*a76);
    a17=(a17*a74);
    a25=(a25+a17);
    a17=(a102/a14);
    a11=(a11*a17);
    a25=(a25+a11);
    a71=(a71*a17);
    a25=(a25+a71);
    a72=(a72*a17);
    a9=(a9*a72);
    a25=(a25+a9);
    a68=(a68*a17);
    a25=(a25+a68);
    a69=(a69*a17);
    a66=(a66*a69);
    a25=(a25+a66);
    a67=(a67*a69);
    a1=(a1*a67);
    a25=(a25+a1);
    a86=(a86+a82);
    a53=(a53*a86);
    a42=(a42*a53);
    a25=(a25-a42);
    a59=(a59*a53);
    a25=(a25-a59);
    a60=(a60*a53);
    a40=(a40*a60);
    a25=(a25-a40);
    a56=(a56*a53);
    a25=(a25-a56);
    a57=(a57*a53);
    a54=(a54*a57);
    a25=(a25-a54);
    a55=(a55*a57);
    a36=(a36*a55);
    a25=(a25-a36);
    a36=(a86/a14);
    a34=(a34*a36);
    a25=(a25-a34);
    a51=(a51*a36);
    a25=(a25-a51);
    a52=(a52*a36);
    a32=(a32*a52);
    a25=(a25-a32);
    a48=(a48*a36);
    a25=(a25-a48);
    a49=(a49*a36);
    a46=(a46*a49);
    a25=(a25-a46);
    a47=(a47*a49);
    a27=(a27*a47);
    a25=(a25-a27);
    a61=(a61*a87);
    a81=(a81*a83);
    a61=(a61-a81);
    a81=(a61/a44);
    a43=(a43+a43);
    a65=(a65/a44);
    a65=(a65*a61);
    a45=(a45/a44);
    a19=(a19*a87);
    a63=(a63*a83);
    a19=(a19+a63);
    a45=(a45*a19);
    a65=(a65+a45);
    a45=(a44+a44);
    a65=(a65/a45);
    a43=(a43*a65);
    a81=(a81-a43);
    a35=(a35*a81);
    a41=(a41*a35);
    a25=(a25+a41);
    a37=(a37*a35);
    a25=(a25+a37);
    a38=(a38*a35);
    a0=(a0*a38);
    a25=(a25+a0);
    a0=(a81/a14);
    a33=(a33*a0);
    a25=(a25+a33);
    a29=(a29*a0);
    a25=(a25+a29);
    a30=(a30*a0);
    a28=(a28*a30);
    a25=(a25+a28);
    a19=(a19/a44);
    a26=(a26+a26);
    a26=(a26*a65);
    a19=(a19-a26);
    a16=(a16*a19);
    a24=(a24*a16);
    a25=(a25+a24);
    a20=(a20*a16);
    a25=(a25+a20);
    a21=(a21*a16);
    a18=(a18*a21);
    a25=(a25+a18);
    a50=(a50/a14);
    a50=(a50*a86);
    a70=(a70/a14);
    a70=(a70*a102);
    a50=(a50-a70);
    a31=(a31/a14);
    a31=(a31*a81);
    a50=(a50-a31);
    a15=(a15/a14);
    a58=(a58*a86);
    a77=(a77*a102);
    a58=(a58-a77);
    a39=(a39*a81);
    a58=(a58-a39);
    a22=(a22*a19);
    a58=(a58-a22);
    a15=(a15*a58);
    a50=(a50-a15);
    a7=(a7/a14);
    a7=(a7*a19);
    a50=(a50-a7);
    a12=(a12*a50);
    a50=10.;
    a50=(a50*a12);
    a25=(a25+a50);
    a19=(a19/a14);
    a10=(a10*a19);
    a25=(a25+a10);
    a5=(a5*a19);
    a25=(a25+a5);
    a6=(a6*a19);
    a2=(a2*a6);
    a25=(a25+a2);
    if (res[1]!=0) res[1][6]=a25;
    a25=cos(a98);
    a25=(a25*a100);
    a25=(a62+a25);
    a101=(a25-a101);
    a2=cos(a103);
    a6=(a101*a2);
    a19=sin(a98);
    a19=(a19*a100);
    a19=(a80+a19);
    a107=(a19-a107);
    a5=sin(a103);
    a10=(a107*a5);
    a6=(a6-a10);
    a110=(a110+a111);
    a110=sq(a110);
    a6=(a6/a110);
    a10=(a6*a2);
    a14=sin(a103);
    a50=(a101*a14);
    a103=cos(a103);
    a12=(a107*a103);
    a50=(a50+a12);
    a117=(a117+a111);
    a117=sq(a117);
    a50=(a50/a117);
    a12=(a50*a14);
    a10=(a10+a12);
    a12=(a10*a101);
    a50=(a50*a103);
    a6=(a6*a5);
    a50=(a50-a6);
    a6=(a50*a107);
    a12=(a12+a6);
    a12=(a12+a96);
    if (res[2]!=0) res[2][0]=a12;
    a25=(a25-a122);
    a122=cos(a123);
    a12=(a25*a122);
    a19=(a19-a126);
    a126=sin(a123);
    a6=(a19*a126);
    a12=(a12-a6);
    a128=(a128+a111);
    a128=sq(a128);
    a12=(a12/a128);
    a6=(a12*a122);
    a7=sin(a123);
    a15=(a25*a7);
    a123=cos(a123);
    a58=(a19*a123);
    a15=(a15+a58);
    a134=(a134+a111);
    a134=sq(a134);
    a15=(a15/a134);
    a111=(a15*a7);
    a6=(a6+a111);
    a111=(a6*a25);
    a15=(a15*a123);
    a12=(a12*a126);
    a15=(a15-a12);
    a12=(a15*a19);
    a111=(a111+a12);
    a111=(a111+a96);
    if (res[2]!=0) res[2][1]=a111;
    if (res[3]!=0) res[3][0]=a13;
    if (res[3]!=0) res[3][1]=a13;
    a111=(a2/a110);
    a96=(a2*a111);
    a12=(a14/a117);
    a58=(a14*a12);
    a96=(a96+a58);
    a96=(a101*a96);
    a96=(a96+a10);
    a12=(a103*a12);
    a111=(a5*a111);
    a12=(a12-a111);
    a12=(a107*a12);
    a96=(a96+a12);
    if (res[3]!=0) res[3][2]=a96;
    a96=(a122/a128);
    a12=(a122*a96);
    a111=(a7/a134);
    a58=(a7*a111);
    a12=(a12+a58);
    a12=(a25*a12);
    a12=(a12+a6);
    a111=(a123*a111);
    a96=(a126*a96);
    a111=(a111-a96);
    a111=(a19*a111);
    a12=(a12+a111);
    if (res[3]!=0) res[3][3]=a12;
    a12=(a103/a117);
    a111=(a14*a12);
    a96=(a5/a110);
    a58=(a2*a96);
    a111=(a111-a58);
    a111=(a101*a111);
    a12=(a103*a12);
    a96=(a5*a96);
    a12=(a12+a96);
    a12=(a107*a12);
    a12=(a12+a50);
    a111=(a111+a12);
    if (res[3]!=0) res[3][4]=a111;
    a111=(a123/a134);
    a12=(a7*a111);
    a96=(a126/a128);
    a58=(a122*a96);
    a12=(a12-a58);
    a12=(a25*a12);
    a111=(a123*a111);
    a96=(a126*a96);
    a111=(a111+a96);
    a111=(a19*a111);
    a111=(a111+a15);
    a12=(a12+a111);
    if (res[3]!=0) res[3][5]=a12;
    a12=cos(a98);
    a12=(a100*a12);
    a111=(a103*a12);
    a96=sin(a98);
    a100=(a100*a96);
    a96=(a14*a100);
    a111=(a111-a96);
    a111=(a111/a117);
    a14=(a14*a111);
    a117=(a2*a100);
    a96=(a5*a12);
    a117=(a117+a96);
    a117=(a117/a110);
    a2=(a2*a117);
    a14=(a14-a2);
    a101=(a101*a14);
    a10=(a10*a100);
    a101=(a101-a10);
    a103=(a103*a111);
    a5=(a5*a117);
    a103=(a103+a5);
    a107=(a107*a103);
    a50=(a50*a12);
    a107=(a107+a50);
    a101=(a101+a107);
    if (res[3]!=0) res[3][6]=a101;
    a101=(a123*a12);
    a107=(a7*a100);
    a101=(a101-a107);
    a101=(a101/a134);
    a7=(a7*a101);
    a134=(a122*a100);
    a107=(a126*a12);
    a134=(a134+a107);
    a134=(a134/a128);
    a122=(a122*a134);
    a7=(a7-a122);
    a25=(a25*a7);
    a6=(a6*a100);
    a25=(a25-a6);
    a123=(a123*a101);
    a126=(a126*a134);
    a123=(a123+a126);
    a19=(a19*a123);
    a15=(a15*a12);
    a19=(a19+a15);
    a25=(a25+a19);
    if (res[3]!=0) res[3][7]=a25;
    a25=cos(a98);
    a19=(a88*a25);
    a15=2.0000000000000001e-01;
    a12=(a15*a93);
    a12=(a98+a12);
    a123=cos(a12);
    a126=(a88*a123);
    a126=(a8*a126);
    a19=(a19+a126);
    a126=(a15*a93);
    a126=(a98+a126);
    a134=cos(a126);
    a101=(a88*a134);
    a101=(a8*a101);
    a19=(a19+a101);
    a101=4.0000000000000002e-01;
    a6=(a101*a93);
    a6=(a98+a6);
    a100=cos(a6);
    a7=(a88*a100);
    a19=(a19+a7);
    a7=6.6666666666666666e-02;
    a19=(a7*a19);
    a62=(a62+a19);
    if (res[4]!=0) res[4][0]=a62;
    a62=sin(a98);
    a19=(a88*a62);
    a122=sin(a12);
    a128=(a88*a122);
    a128=(a8*a128);
    a19=(a19+a128);
    a128=sin(a126);
    a107=(a88*a128);
    a107=(a8*a107);
    a19=(a19+a107);
    a107=sin(a6);
    a50=(a88*a107);
    a19=(a19+a50);
    a19=(a7*a19);
    a80=(a80+a19);
    if (res[4]!=0) res[4][1]=a80;
    a80=(a8*a93);
    a80=(a93+a80);
    a19=(a8*a93);
    a80=(a80+a19);
    a80=(a80+a93);
    a80=(a7*a80);
    a80=(a98+a80);
    if (res[4]!=0) res[4][2]=a80;
    a80=(a8*a88);
    a80=(a88+a80);
    a93=(a8*a88);
    a80=(a80+a93);
    a80=(a80+a88);
    a80=(a7*a80);
    a3=(a3+a80);
    if (res[4]!=0) res[4][3]=a3;
    a123=(a8*a123);
    a25=(a25+a123);
    a134=(a8*a134);
    a25=(a25+a134);
    a25=(a25+a100);
    a25=(a7*a25);
    if (res[5]!=0) res[5][0]=a25;
    a122=(a8*a122);
    a62=(a62+a122);
    a128=(a8*a128);
    a62=(a62+a128);
    a62=(a62+a107);
    a62=(a7*a62);
    if (res[5]!=0) res[5][1]=a62;
    if (res[5]!=0) res[5][2]=a101;
    a62=sin(a12);
    a107=(a15*a62);
    a107=(a88*a107);
    a107=(a8*a107);
    a128=sin(a126);
    a122=(a15*a128);
    a122=(a88*a122);
    a122=(a8*a122);
    a107=(a107+a122);
    a122=sin(a6);
    a25=(a101*a122);
    a25=(a88*a25);
    a107=(a107+a25);
    a107=(a7*a107);
    a107=(-a107);
    if (res[5]!=0) res[5][3]=a107;
    a12=cos(a12);
    a107=(a15*a12);
    a107=(a88*a107);
    a107=(a8*a107);
    a126=cos(a126);
    a15=(a15*a126);
    a15=(a88*a15);
    a15=(a8*a15);
    a107=(a107+a15);
    a6=cos(a6);
    a15=(a101*a6);
    a15=(a88*a15);
    a107=(a107+a15);
    a107=(a7*a107);
    if (res[5]!=0) res[5][4]=a107;
    if (res[5]!=0) res[5][5]=a101;
    if (res[5]!=0) res[5][6]=a13;
    if (res[5]!=0) res[5][7]=a13;
    a101=sin(a98);
    a101=(a88*a101);
    a62=(a88*a62);
    a62=(a8*a62);
    a101=(a101+a62);
    a128=(a88*a128);
    a128=(a8*a128);
    a101=(a101+a128);
    a122=(a88*a122);
    a101=(a101+a122);
    a101=(a7*a101);
    a101=(-a101);
    if (res[5]!=0) res[5][8]=a101;
    a98=cos(a98);
    a98=(a88*a98);
    a12=(a88*a12);
    a12=(a8*a12);
    a98=(a98+a12);
    a126=(a88*a126);
    a8=(a8*a126);
    a98=(a98+a8);
    a88=(a88*a6);
    a98=(a98+a88);
    a7=(a7*a98);
    if (res[5]!=0) res[5][9]=a7;
    if (res[5]!=0) res[5][10]=a13;
    if (res[5]!=0) res[5][11]=a13;
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
    if (sz_w) *sz_w = 139;
    return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
