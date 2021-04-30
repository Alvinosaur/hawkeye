/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) FORCESNLPsolver_model_ ## ID
#endif

#include <math.h> 
#include "FORCESNLPsolver_model.h"

#ifndef casadi_real
#define casadi_real FORCESNLPsolver_float
#endif

#ifndef casadi_int
#define casadi_int solver_int32_default
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#if 0
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[15] = {11, 1, 0, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s3[22] = {1, 11, 0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0};
static const casadi_int casadi_s4[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s5[31] = {7, 11, 0, 2, 4, 6, 7, 8, 9, 10, 12, 14, 16, 17, 0, 3, 1, 4, 2, 5, 6, 0, 1, 2, 0, 3, 1, 4, 2, 5, 6};

/* FORCESNLPsolver_objective_0:(i0[11],i1[6])->(o0,o1[1x11,8nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a4, a5, a6, a7, a8, a9;
  a0=100.;
  a1=1.;
  a2=arg[1]? arg[1][3] : 0;
  a3=casadi_sq(a2);
  a4=arg[0]? arg[0][10] : 0;
  a5=cos(a4);
  a6=arg[1]? arg[1][4] : 0;
  a5=(a5*a6);
  a7=sin(a4);
  a8=arg[1]? arg[1][5] : 0;
  a7=(a7*a8);
  a5=(a5-a7);
  a7=casadi_sq(a5);
  a3=(a3+a7);
  a7=sin(a4);
  a7=(a7*a6);
  a9=cos(a4);
  a9=(a9*a8);
  a7=(a7+a9);
  a9=casadi_sq(a7);
  a3=(a3+a9);
  a3=sqrt(a3);
  a2=(a2/a3);
  a9=arg[1]? arg[1][0] : 0;
  a10=arg[0]? arg[0][4] : 0;
  a11=(a9-a10);
  a12=casadi_sq(a11);
  a13=arg[1]? arg[1][1] : 0;
  a14=arg[0]? arg[0][5] : 0;
  a15=(a13-a14);
  a16=casadi_sq(a15);
  a12=(a12+a16);
  a16=arg[1]? arg[1][2] : 0;
  a17=arg[0]? arg[0][6] : 0;
  a16=(a16-a17);
  a17=casadi_sq(a16);
  a12=(a12+a17);
  a12=sqrt(a12);
  a17=(a11/a12);
  a18=(a2*a17);
  a19=(a5/a3);
  a20=(a15/a12);
  a21=(a19*a20);
  a18=(a18+a21);
  a21=(a7/a3);
  a22=(a16/a12);
  a23=(a21*a22);
  a18=(a18+a23);
  a1=(a1-a18);
  a18=casadi_sq(a1);
  a18=(a0*a18);
  a23=10.;
  a24=8.;
  a24=(a10-a24);
  a25=casadi_sq(a24);
  a25=(a23*a25);
  a18=(a18+a25);
  a25=22.;
  a10=(a10-a9);
  a9=casadi_sq(a10);
  a14=(a14-a13);
  a13=casadi_sq(a14);
  a9=(a9+a13);
  a25=(a25-a9);
  a9=casadi_sq(a25);
  a9=(a23*a9);
  a18=(a18+a9);
  a9=1.0000000000000000e-02;
  a13=arg[0]? arg[0][0] : 0;
  a26=casadi_sq(a13);
  a27=arg[0]? arg[0][1] : 0;
  a28=casadi_sq(a27);
  a26=(a26+a28);
  a28=arg[0]? arg[0][2] : 0;
  a29=casadi_sq(a28);
  a26=(a26+a29);
  a29=1.0000000000000000e-03;
  a30=arg[0]? arg[0][3] : 0;
  a31=(a29*a30);
  a32=(a31*a30);
  a26=(a26+a32);
  a26=(a9*a26);
  a18=(a18+a26);
  if (res[0]!=0) res[0][0]=a18;
  a13=(a13+a13);
  a13=(a9*a13);
  if (res[1]!=0) res[1][0]=a13;
  a27=(a27+a27);
  a27=(a9*a27);
  if (res[1]!=0) res[1][1]=a27;
  a28=(a28+a28);
  a28=(a9*a28);
  if (res[1]!=0) res[1][2]=a28;
  a31=(a9*a31);
  a9=(a9*a30);
  a29=(a29*a9);
  a31=(a31+a29);
  if (res[1]!=0) res[1][3]=a31;
  a24=(a24+a24);
  a24=(a23*a24);
  a10=(a10+a10);
  a25=(a25+a25);
  a23=(a23*a25);
  a10=(a10*a23);
  a24=(a24-a10);
  a11=(a11+a11);
  a10=(a22/a12);
  a1=(a1+a1);
  a0=(a0*a1);
  a1=(a21*a0);
  a10=(a10*a1);
  a25=(a20/a12);
  a31=(a19*a0);
  a25=(a25*a31);
  a10=(a10+a25);
  a25=(a17/a12);
  a29=(a2*a0);
  a25=(a25*a29);
  a10=(a10+a25);
  a25=(a12+a12);
  a10=(a10/a25);
  a11=(a11*a10);
  a29=(a29/a12);
  a11=(a11-a29);
  a24=(a24-a11);
  if (res[1]!=0) res[1][4]=a24;
  a14=(a14+a14);
  a14=(a14*a23);
  a15=(a15+a15);
  a15=(a15*a10);
  a31=(a31/a12);
  a15=(a15-a31);
  a14=(a14+a15);
  a14=(-a14);
  if (res[1]!=0) res[1][5]=a14;
  a16=(a16+a16);
  a16=(a16*a10);
  a1=(a1/a12);
  a16=(a16-a1);
  a16=(-a16);
  if (res[1]!=0) res[1][6]=a16;
  a16=cos(a4);
  a7=(a7+a7);
  a21=(a21/a3);
  a22=(a22*a0);
  a21=(a21*a22);
  a19=(a19/a3);
  a20=(a20*a0);
  a19=(a19*a20);
  a21=(a21+a19);
  a2=(a2/a3);
  a17=(a17*a0);
  a2=(a2*a17);
  a21=(a21+a2);
  a2=(a3+a3);
  a21=(a21/a2);
  a7=(a7*a21);
  a22=(a22/a3);
  a7=(a7-a22);
  a22=(a6*a7);
  a16=(a16*a22);
  a22=sin(a4);
  a7=(a8*a7);
  a22=(a22*a7);
  a16=(a16-a22);
  a22=cos(a4);
  a5=(a5+a5);
  a5=(a5*a21);
  a20=(a20/a3);
  a5=(a5-a20);
  a8=(a8*a5);
  a22=(a22*a8);
  a16=(a16-a22);
  a4=sin(a4);
  a6=(a6*a5);
  a4=(a4*a6);
  a16=(a16-a4);
  if (res[1]!=0) res[1][7]=a16;
  return 0;
}

int FORCESNLPsolver_objective_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

int FORCESNLPsolver_objective_0_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_objective_0_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_objective_0_free_mem(int mem) {
}

int FORCESNLPsolver_objective_0_checkout(void) {
  return 0;
}

void FORCESNLPsolver_objective_0_release(int mem) {
}

void FORCESNLPsolver_objective_0_incref(void) {
}

void FORCESNLPsolver_objective_0_decref(void) {
}

casadi_int FORCESNLPsolver_objective_0_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_objective_0_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_objective_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    default: return 0;
  }
}

int FORCESNLPsolver_objective_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_dynamics_0:(i0[11],i1[6])->(o0[7],o1[7x11,17nz]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][4] : 0;
  a1=1.0000000000000001e-01;
  a2=arg[0]? arg[0][7] : 0;
  a3=(a1*a2);
  a0=(a0+a3);
  a3=5.0000000000000010e-03;
  a4=arg[0]? arg[0][0] : 0;
  a5=(a3*a4);
  a0=(a0+a5);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][5] : 0;
  a5=arg[0]? arg[0][8] : 0;
  a6=(a1*a5);
  a0=(a0+a6);
  a6=arg[0]? arg[0][1] : 0;
  a7=(a3*a6);
  a0=(a0+a7);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][6] : 0;
  a7=arg[0]? arg[0][9] : 0;
  a8=(a1*a7);
  a0=(a0+a8);
  a8=arg[0]? arg[0][2] : 0;
  a9=(a3*a8);
  a0=(a0+a9);
  if (res[0]!=0) res[0][2]=a0;
  a4=(a1*a4);
  a2=(a2+a4);
  if (res[0]!=0) res[0][3]=a2;
  a6=(a1*a6);
  a5=(a5+a6);
  if (res[0]!=0) res[0][4]=a5;
  a8=(a1*a8);
  a7=(a7+a8);
  if (res[0]!=0) res[0][5]=a7;
  a7=arg[0]? arg[0][10] : 0;
  a8=arg[0]? arg[0][3] : 0;
  a8=(a1*a8);
  a7=(a7+a8);
  if (res[0]!=0) res[0][6]=a7;
  if (res[1]!=0) res[1][0]=a3;
  if (res[1]!=0) res[1][1]=a1;
  if (res[1]!=0) res[1][2]=a3;
  if (res[1]!=0) res[1][3]=a1;
  if (res[1]!=0) res[1][4]=a3;
  if (res[1]!=0) res[1][5]=a1;
  if (res[1]!=0) res[1][6]=a1;
  a3=1.;
  if (res[1]!=0) res[1][7]=a3;
  if (res[1]!=0) res[1][8]=a3;
  if (res[1]!=0) res[1][9]=a3;
  if (res[1]!=0) res[1][10]=a1;
  if (res[1]!=0) res[1][11]=a3;
  if (res[1]!=0) res[1][12]=a1;
  if (res[1]!=0) res[1][13]=a3;
  if (res[1]!=0) res[1][14]=a1;
  if (res[1]!=0) res[1][15]=a3;
  if (res[1]!=0) res[1][16]=a3;
  return 0;
}

int FORCESNLPsolver_dynamics_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

int FORCESNLPsolver_dynamics_0_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_dynamics_0_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_dynamics_0_free_mem(int mem) {
}

int FORCESNLPsolver_dynamics_0_checkout(void) {
  return 0;
}

void FORCESNLPsolver_dynamics_0_release(int mem) {
}

void FORCESNLPsolver_dynamics_0_incref(void) {
}

void FORCESNLPsolver_dynamics_0_decref(void) {
}

casadi_int FORCESNLPsolver_dynamics_0_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_dynamics_0_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_dynamics_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_dynamics_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_dynamics_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dynamics_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_dynamics_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    default: return 0;
  }
}

int FORCESNLPsolver_dynamics_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_objective_1:(i0[11],i1[6])->(o0,o1[1x11,8nz]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a4, a5, a6, a7, a8, a9;
  a0=100.;
  a1=1.;
  a2=arg[1]? arg[1][3] : 0;
  a3=casadi_sq(a2);
  a4=arg[0]? arg[0][10] : 0;
  a5=cos(a4);
  a6=arg[1]? arg[1][4] : 0;
  a5=(a5*a6);
  a7=sin(a4);
  a8=arg[1]? arg[1][5] : 0;
  a7=(a7*a8);
  a5=(a5-a7);
  a7=casadi_sq(a5);
  a3=(a3+a7);
  a7=sin(a4);
  a7=(a7*a6);
  a9=cos(a4);
  a9=(a9*a8);
  a7=(a7+a9);
  a9=casadi_sq(a7);
  a3=(a3+a9);
  a3=sqrt(a3);
  a2=(a2/a3);
  a9=arg[1]? arg[1][0] : 0;
  a10=arg[0]? arg[0][4] : 0;
  a11=(a9-a10);
  a12=casadi_sq(a11);
  a13=arg[1]? arg[1][1] : 0;
  a14=arg[0]? arg[0][5] : 0;
  a15=(a13-a14);
  a16=casadi_sq(a15);
  a12=(a12+a16);
  a16=arg[1]? arg[1][2] : 0;
  a17=arg[0]? arg[0][6] : 0;
  a16=(a16-a17);
  a17=casadi_sq(a16);
  a12=(a12+a17);
  a12=sqrt(a12);
  a17=(a11/a12);
  a18=(a2*a17);
  a19=(a5/a3);
  a20=(a15/a12);
  a21=(a19*a20);
  a18=(a18+a21);
  a21=(a7/a3);
  a22=(a16/a12);
  a23=(a21*a22);
  a18=(a18+a23);
  a1=(a1-a18);
  a18=casadi_sq(a1);
  a18=(a0*a18);
  a23=10.;
  a24=8.;
  a24=(a10-a24);
  a25=casadi_sq(a24);
  a25=(a23*a25);
  a18=(a18+a25);
  a25=22.;
  a10=(a10-a9);
  a9=casadi_sq(a10);
  a14=(a14-a13);
  a13=casadi_sq(a14);
  a9=(a9+a13);
  a25=(a25-a9);
  a9=casadi_sq(a25);
  a9=(a23*a9);
  a18=(a18+a9);
  a9=1.0000000000000000e-02;
  a13=arg[0]? arg[0][0] : 0;
  a26=casadi_sq(a13);
  a27=arg[0]? arg[0][1] : 0;
  a28=casadi_sq(a27);
  a26=(a26+a28);
  a28=arg[0]? arg[0][2] : 0;
  a29=casadi_sq(a28);
  a26=(a26+a29);
  a29=1.0000000000000000e-03;
  a30=arg[0]? arg[0][3] : 0;
  a31=(a29*a30);
  a32=(a31*a30);
  a26=(a26+a32);
  a26=(a9*a26);
  a18=(a18+a26);
  if (res[0]!=0) res[0][0]=a18;
  a13=(a13+a13);
  a13=(a9*a13);
  if (res[1]!=0) res[1][0]=a13;
  a27=(a27+a27);
  a27=(a9*a27);
  if (res[1]!=0) res[1][1]=a27;
  a28=(a28+a28);
  a28=(a9*a28);
  if (res[1]!=0) res[1][2]=a28;
  a31=(a9*a31);
  a9=(a9*a30);
  a29=(a29*a9);
  a31=(a31+a29);
  if (res[1]!=0) res[1][3]=a31;
  a24=(a24+a24);
  a24=(a23*a24);
  a10=(a10+a10);
  a25=(a25+a25);
  a23=(a23*a25);
  a10=(a10*a23);
  a24=(a24-a10);
  a11=(a11+a11);
  a10=(a22/a12);
  a1=(a1+a1);
  a0=(a0*a1);
  a1=(a21*a0);
  a10=(a10*a1);
  a25=(a20/a12);
  a31=(a19*a0);
  a25=(a25*a31);
  a10=(a10+a25);
  a25=(a17/a12);
  a29=(a2*a0);
  a25=(a25*a29);
  a10=(a10+a25);
  a25=(a12+a12);
  a10=(a10/a25);
  a11=(a11*a10);
  a29=(a29/a12);
  a11=(a11-a29);
  a24=(a24-a11);
  if (res[1]!=0) res[1][4]=a24;
  a14=(a14+a14);
  a14=(a14*a23);
  a15=(a15+a15);
  a15=(a15*a10);
  a31=(a31/a12);
  a15=(a15-a31);
  a14=(a14+a15);
  a14=(-a14);
  if (res[1]!=0) res[1][5]=a14;
  a16=(a16+a16);
  a16=(a16*a10);
  a1=(a1/a12);
  a16=(a16-a1);
  a16=(-a16);
  if (res[1]!=0) res[1][6]=a16;
  a16=cos(a4);
  a7=(a7+a7);
  a21=(a21/a3);
  a22=(a22*a0);
  a21=(a21*a22);
  a19=(a19/a3);
  a20=(a20*a0);
  a19=(a19*a20);
  a21=(a21+a19);
  a2=(a2/a3);
  a17=(a17*a0);
  a2=(a2*a17);
  a21=(a21+a2);
  a2=(a3+a3);
  a21=(a21/a2);
  a7=(a7*a21);
  a22=(a22/a3);
  a7=(a7-a22);
  a22=(a6*a7);
  a16=(a16*a22);
  a22=sin(a4);
  a7=(a8*a7);
  a22=(a22*a7);
  a16=(a16-a22);
  a22=cos(a4);
  a5=(a5+a5);
  a5=(a5*a21);
  a20=(a20/a3);
  a5=(a5-a20);
  a8=(a8*a5);
  a22=(a22*a8);
  a16=(a16-a22);
  a4=sin(a4);
  a6=(a6*a5);
  a4=(a4*a6);
  a16=(a16-a4);
  if (res[1]!=0) res[1][7]=a16;
  return 0;
}

int FORCESNLPsolver_objective_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

int FORCESNLPsolver_objective_1_alloc_mem(void) {
  return 0;
}

int FORCESNLPsolver_objective_1_init_mem(int mem) {
  return 0;
}

void FORCESNLPsolver_objective_1_free_mem(int mem) {
}

int FORCESNLPsolver_objective_1_checkout(void) {
  return 0;
}

void FORCESNLPsolver_objective_1_release(int mem) {
}

void FORCESNLPsolver_objective_1_incref(void) {
}

void FORCESNLPsolver_objective_1_decref(void) {
}

casadi_int FORCESNLPsolver_objective_1_n_in(void) { return 2;}

casadi_int FORCESNLPsolver_objective_1_n_out(void) { return 2;}

casadi_real FORCESNLPsolver_objective_1_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_1_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

const char* FORCESNLPsolver_objective_1_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_1_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* FORCESNLPsolver_objective_1_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    default: return 0;
  }
}

int FORCESNLPsolver_objective_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
