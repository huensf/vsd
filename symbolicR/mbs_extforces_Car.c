//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Mon Nov 30 14:42:11 2020
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 44
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 1708
//
//	==> Generation Time :  0.030 seconds
//	==> Post-Processing :  0.030 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_extforces(double **frc,double **trq,
MbsData *s, double tsim)

// double frc[3][44];
// double trq[3][44];
{ 
double PxF1[4]; 
double RxF1[4][4]; 
double VxF1[4]; 
double OMxF1[4]; 
double AxF1[4]; 
double OMPxF1[4]; 
double *SWr1; 
double PxF2[4]; 
double RxF2[4][4]; 
double VxF2[4]; 
double OMxF2[4]; 
double AxF2[4]; 
double OMPxF2[4]; 
double *SWr2; 
double PxF3[4]; 
double RxF3[4][4]; 
double VxF3[4]; 
double OMxF3[4]; 
double AxF3[4]; 
double OMPxF3[4]; 
double *SWr3; 
double PxF4[4]; 
double RxF4[4][4]; 
double VxF4[4]; 
double OMxF4[4]; 
double AxF4[4]; 
double OMPxF4[4]; 
double *SWr4; 
 
#include "mbs_extforces_Car.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);
  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);
  C17 = cos(q[17]);
  S17 = sin(q[17]);

// = = Block_0_0_0_0_0_15 = = 
 
// Trigonometric Variables  

  C33 = cos(q[33]);
  S33 = sin(q[33]);
  C34 = cos(q[34]);
  S34 = sin(q[34]);
  C35 = cos(q[35]);
  S35 = sin(q[35]);

// = = Block_0_0_0_0_0_16 = = 
 
// Trigonometric Variables  

  C36 = cos(q[36]);
  S36 = sin(q[36]);
  C37 = cos(q[37]);
  S37 = sin(q[37]);
  C38 = cos(q[38]);
  S38 = sin(q[38]);

// = = Block_0_0_1_1_0_1 = = 
 
// Sensor Kinematics 


  ROcp1_45 = -S4*C5;
  ROcp1_55 = C4*C5;
  ROcp1_75 = S4*S5;
  ROcp1_85 = -C4*S5;
  ROcp1_16 = -(ROcp1_75*S6-C4*C6);
  ROcp1_26 = -(ROcp1_85*S6-S4*C6);
  ROcp1_36 = -C5*S6;
  ROcp1_76 = ROcp1_75*C6+C4*S6;
  ROcp1_86 = ROcp1_85*C6+S4*S6;
  ROcp1_96 = C5*C6;
  OMcp1_15 = qd[5]*C4;
  OMcp1_25 = qd[5]*S4;
  OMcp1_16 = OMcp1_15+qd[6]*ROcp1_45;
  OMcp1_26 = OMcp1_25+qd[6]*ROcp1_55;
  OMcp1_36 = qd[4]+qd[6]*S5;
  OPcp1_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp1_55-OMcp1_25*S5)-qdd[5]*C4-qdd[6]*ROcp1_45);
  OPcp1_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp1_45-OMcp1_15*S5)+qdd[5]*S4+qdd[6]*ROcp1_55;
  OPcp1_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_1_0_3 = = 
 
// Sensor Kinematics 


  ROcp1_48 = ROcp1_45*C8+ROcp1_76*S8;
  ROcp1_58 = ROcp1_55*C8+ROcp1_86*S8;
  ROcp1_68 = ROcp1_96*S8+S5*C8;
  ROcp1_78 = -(ROcp1_45*S8-ROcp1_76*C8);
  ROcp1_88 = -(ROcp1_55*S8-ROcp1_86*C8);
  ROcp1_98 = ROcp1_96*C8-S5*S8;
  ROcp1_19 = ROcp1_16*C9-ROcp1_78*S9;
  ROcp1_29 = ROcp1_26*C9-ROcp1_88*S9;
  ROcp1_39 = ROcp1_36*C9-ROcp1_98*S9;
  ROcp1_79 = ROcp1_16*S9+ROcp1_78*C9;
  ROcp1_89 = ROcp1_26*S9+ROcp1_88*C9;
  ROcp1_99 = ROcp1_36*S9+ROcp1_98*C9;
  ROcp1_410 = ROcp1_48*C10+ROcp1_79*S10;
  ROcp1_510 = ROcp1_58*C10+ROcp1_89*S10;
  ROcp1_610 = ROcp1_68*C10+ROcp1_99*S10;
  ROcp1_710 = -(ROcp1_48*S10-ROcp1_79*C10);
  ROcp1_810 = -(ROcp1_58*S10-ROcp1_89*C10);
  ROcp1_910 = -(ROcp1_68*S10-ROcp1_99*C10);
  ROcp1_111 = ROcp1_19*C11+ROcp1_410*S11;
  ROcp1_211 = ROcp1_29*C11+ROcp1_510*S11;
  ROcp1_311 = ROcp1_39*C11+ROcp1_610*S11;
  ROcp1_411 = -(ROcp1_19*S11-ROcp1_410*C11);
  ROcp1_511 = -(ROcp1_29*S11-ROcp1_510*C11);
  ROcp1_611 = -(ROcp1_39*S11-ROcp1_610*C11);
  ROcp1_112 = ROcp1_111*C12-ROcp1_710*S12;
  ROcp1_212 = ROcp1_211*C12-ROcp1_810*S12;
  ROcp1_312 = ROcp1_311*C12-ROcp1_910*S12;
  ROcp1_712 = ROcp1_111*S12+ROcp1_710*C12;
  ROcp1_812 = ROcp1_211*S12+ROcp1_810*C12;
  ROcp1_912 = ROcp1_311*S12+ROcp1_910*C12;
  RLcp1_18 = ROcp1_16*s->dpt[1][1]+ROcp1_45*s->dpt[2][1]+ROcp1_76*s->dpt[3][1];
  RLcp1_28 = ROcp1_26*s->dpt[1][1]+ROcp1_55*s->dpt[2][1]+ROcp1_86*s->dpt[3][1];
  RLcp1_38 = ROcp1_36*s->dpt[1][1]+ROcp1_96*s->dpt[3][1]+s->dpt[2][1]*S5;
  OMcp1_18 = OMcp1_16+qd[8]*ROcp1_16;
  OMcp1_28 = OMcp1_26+qd[8]*ROcp1_26;
  OMcp1_38 = OMcp1_36+qd[8]*ROcp1_36;
  ORcp1_18 = OMcp1_26*RLcp1_38-OMcp1_36*RLcp1_28;
  ORcp1_28 = -(OMcp1_16*RLcp1_38-OMcp1_36*RLcp1_18);
  ORcp1_38 = OMcp1_16*RLcp1_28-OMcp1_26*RLcp1_18;
  OPcp1_18 = OPcp1_16+qd[8]*(OMcp1_26*ROcp1_36-OMcp1_36*ROcp1_26)+qdd[8]*ROcp1_16;
  OPcp1_28 = OPcp1_26-qd[8]*(OMcp1_16*ROcp1_36-OMcp1_36*ROcp1_16)+qdd[8]*ROcp1_26;
  OPcp1_38 = OPcp1_36+qd[8]*(OMcp1_16*ROcp1_26-OMcp1_26*ROcp1_16)+qdd[8]*ROcp1_36;
  RLcp1_19 = ROcp1_48*s->dpt[2][17];
  RLcp1_29 = ROcp1_58*s->dpt[2][17];
  RLcp1_39 = ROcp1_68*s->dpt[2][17];
  OMcp1_19 = OMcp1_18+qd[9]*ROcp1_48;
  OMcp1_29 = OMcp1_28+qd[9]*ROcp1_58;
  OMcp1_39 = OMcp1_38+qd[9]*ROcp1_68;
  ORcp1_19 = OMcp1_28*RLcp1_39-OMcp1_38*RLcp1_29;
  ORcp1_29 = -(OMcp1_18*RLcp1_39-OMcp1_38*RLcp1_19);
  ORcp1_39 = OMcp1_18*RLcp1_29-OMcp1_28*RLcp1_19;
  OMcp1_110 = OMcp1_19+qd[10]*ROcp1_19;
  OMcp1_210 = OMcp1_29+qd[10]*ROcp1_29;
  OMcp1_310 = OMcp1_39+qd[10]*ROcp1_39;
  OMcp1_111 = OMcp1_110+qd[11]*ROcp1_710;
  OMcp1_211 = OMcp1_210+qd[11]*ROcp1_810;
  OMcp1_311 = OMcp1_310+qd[11]*ROcp1_910;
  OPcp1_111 = OPcp1_18+qd[10]*(OMcp1_29*ROcp1_39-OMcp1_39*ROcp1_29)+qd[11]*(OMcp1_210*ROcp1_910-OMcp1_310*ROcp1_810)+
 qd[9]*(OMcp1_28*ROcp1_68-OMcp1_38*ROcp1_58)+qdd[10]*ROcp1_19+qdd[11]*ROcp1_710+qdd[9]*ROcp1_48;
  OPcp1_211 = OPcp1_28-qd[10]*(OMcp1_19*ROcp1_39-OMcp1_39*ROcp1_19)-qd[11]*(OMcp1_110*ROcp1_910-OMcp1_310*ROcp1_710)-
 qd[9]*(OMcp1_18*ROcp1_68-OMcp1_38*ROcp1_48)+qdd[10]*ROcp1_29+qdd[11]*ROcp1_810+qdd[9]*ROcp1_58;
  OPcp1_311 = OPcp1_38+qd[10]*(OMcp1_19*ROcp1_29-OMcp1_29*ROcp1_19)+qd[11]*(OMcp1_110*ROcp1_810-OMcp1_210*ROcp1_710)+
 qd[9]*(OMcp1_18*ROcp1_58-OMcp1_28*ROcp1_48)+qdd[10]*ROcp1_39+qdd[11]*ROcp1_910+qdd[9]*ROcp1_68;
  RLcp1_112 = ROcp1_710*s->dpt[3][21];
  RLcp1_212 = ROcp1_810*s->dpt[3][21];
  RLcp1_312 = ROcp1_910*s->dpt[3][21];
  ORcp1_112 = OMcp1_211*RLcp1_312-OMcp1_311*RLcp1_212;
  ORcp1_212 = -(OMcp1_111*RLcp1_312-OMcp1_311*RLcp1_112);
  ORcp1_312 = OMcp1_111*RLcp1_212-OMcp1_211*RLcp1_112;
  PxF1[1] = q[1]+RLcp1_112+RLcp1_18+RLcp1_19;
  PxF1[2] = q[2]+RLcp1_212+RLcp1_28+RLcp1_29;
  PxF1[3] = q[3]+RLcp1_312+RLcp1_38+RLcp1_39;
  RxF1[1][1] = ROcp1_112;
  RxF1[1][2] = ROcp1_212;
  RxF1[1][3] = ROcp1_312;
  RxF1[2][1] = ROcp1_411;
  RxF1[2][2] = ROcp1_511;
  RxF1[2][3] = ROcp1_611;
  RxF1[3][1] = ROcp1_712;
  RxF1[3][2] = ROcp1_812;
  RxF1[3][3] = ROcp1_912;
  VxF1[1] = qd[1]+ORcp1_112+ORcp1_18+ORcp1_19;
  VxF1[2] = qd[2]+ORcp1_212+ORcp1_28+ORcp1_29;
  VxF1[3] = qd[3]+ORcp1_312+ORcp1_38+ORcp1_39;
  OMxF1[1] = OMcp1_111+qd[12]*ROcp1_411;
  OMxF1[2] = OMcp1_211+qd[12]*ROcp1_511;
  OMxF1[3] = OMcp1_311+qd[12]*ROcp1_611;
  AxF1[1] = qdd[1]+OMcp1_211*ORcp1_312+OMcp1_26*ORcp1_38+OMcp1_28*ORcp1_39-OMcp1_311*ORcp1_212-OMcp1_36*ORcp1_28-
 OMcp1_38*ORcp1_29+OPcp1_211*RLcp1_312+OPcp1_26*RLcp1_38+OPcp1_28*RLcp1_39-OPcp1_311*RLcp1_212-OPcp1_36*RLcp1_28-OPcp1_38*
 RLcp1_29;
  AxF1[2] = qdd[2]-OMcp1_111*ORcp1_312-OMcp1_16*ORcp1_38-OMcp1_18*ORcp1_39+OMcp1_311*ORcp1_112+OMcp1_36*ORcp1_18+
 OMcp1_38*ORcp1_19-OPcp1_111*RLcp1_312-OPcp1_16*RLcp1_38-OPcp1_18*RLcp1_39+OPcp1_311*RLcp1_112+OPcp1_36*RLcp1_18+OPcp1_38*
 RLcp1_19;
  AxF1[3] = qdd[3]+OMcp1_111*ORcp1_212+OMcp1_16*ORcp1_28+OMcp1_18*ORcp1_29-OMcp1_211*ORcp1_112-OMcp1_26*ORcp1_18-
 OMcp1_28*ORcp1_19+OPcp1_111*RLcp1_212+OPcp1_16*RLcp1_28+OPcp1_18*RLcp1_29-OPcp1_211*RLcp1_112-OPcp1_26*RLcp1_18-OPcp1_28*
 RLcp1_19;
  OMPxF1[1] = OPcp1_111+qd[12]*(OMcp1_211*ROcp1_611-OMcp1_311*ROcp1_511)+qdd[12]*ROcp1_411;
  OMPxF1[2] = OPcp1_211-qd[12]*(OMcp1_111*ROcp1_611-OMcp1_311*ROcp1_411)+qdd[12]*ROcp1_511;
  OMPxF1[3] = OPcp1_311+qd[12]*(OMcp1_111*ROcp1_511-OMcp1_211*ROcp1_411)+qdd[12]*ROcp1_611;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc12 = ROcp1_112*SWr1[1]+ROcp1_212*SWr1[2]+ROcp1_312*SWr1[3];
  xfrc22 = ROcp1_411*SWr1[1]+ROcp1_511*SWr1[2]+ROcp1_611*SWr1[3];
  xfrc32 = ROcp1_712*SWr1[1]+ROcp1_812*SWr1[2]+ROcp1_912*SWr1[3];
  frc[1][12] = s->frc[1][12]+xfrc12;
  frc[2][12] = s->frc[2][12]+xfrc22;
  frc[3][12] = s->frc[3][12]+xfrc32;
  xtrq12 = ROcp1_112*SWr1[4]+ROcp1_212*SWr1[5]+ROcp1_312*SWr1[6];
  xtrq22 = ROcp1_411*SWr1[4]+ROcp1_511*SWr1[5]+ROcp1_611*SWr1[6];
  xtrq32 = ROcp1_712*SWr1[4]+ROcp1_812*SWr1[5]+ROcp1_912*SWr1[6];
  trq[1][12] = s->trq[1][12]+xtrq12-xfrc22*SWr1[9]+xfrc32*SWr1[8];
  trq[2][12] = s->trq[2][12]+xtrq22+xfrc12*SWr1[9]-xfrc32*SWr1[7];
  trq[3][12] = s->trq[3][12]+xtrq32-xfrc12*SWr1[8]+xfrc22*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
// Sensor Kinematics 


  ROcp2_45 = -S4*C5;
  ROcp2_55 = C4*C5;
  ROcp2_75 = S4*S5;
  ROcp2_85 = -C4*S5;
  ROcp2_16 = -(ROcp2_75*S6-C4*C6);
  ROcp2_26 = -(ROcp2_85*S6-S4*C6);
  ROcp2_36 = -C5*S6;
  ROcp2_76 = ROcp2_75*C6+C4*S6;
  ROcp2_86 = ROcp2_85*C6+S4*S6;
  ROcp2_96 = C5*C6;
  OMcp2_15 = qd[5]*C4;
  OMcp2_25 = qd[5]*S4;
  OMcp2_16 = OMcp2_15+qd[6]*ROcp2_45;
  OMcp2_26 = OMcp2_25+qd[6]*ROcp2_55;
  OMcp2_36 = qd[4]+qd[6]*S5;
  OPcp2_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp2_55-OMcp2_25*S5)-qdd[5]*C4-qdd[6]*ROcp2_45);
  OPcp2_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp2_45-OMcp2_15*S5)+qdd[5]*S4+qdd[6]*ROcp2_55;
  OPcp2_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_2_0_4 = = 
 
// Sensor Kinematics 


  ROcp2_413 = ROcp2_45*C13+ROcp2_76*S13;
  ROcp2_513 = ROcp2_55*C13+ROcp2_86*S13;
  ROcp2_613 = ROcp2_96*S13+C13*S5;
  ROcp2_713 = -(ROcp2_45*S13-ROcp2_76*C13);
  ROcp2_813 = -(ROcp2_55*S13-ROcp2_86*C13);
  ROcp2_913 = ROcp2_96*C13-S13*S5;
  ROcp2_114 = ROcp2_16*C14-ROcp2_713*S14;
  ROcp2_214 = ROcp2_26*C14-ROcp2_813*S14;
  ROcp2_314 = ROcp2_36*C14-ROcp2_913*S14;
  ROcp2_714 = ROcp2_16*S14+ROcp2_713*C14;
  ROcp2_814 = ROcp2_26*S14+ROcp2_813*C14;
  ROcp2_914 = ROcp2_36*S14+ROcp2_913*C14;
  ROcp2_415 = ROcp2_413*C15+ROcp2_714*S15;
  ROcp2_515 = ROcp2_513*C15+ROcp2_814*S15;
  ROcp2_615 = ROcp2_613*C15+ROcp2_914*S15;
  ROcp2_715 = -(ROcp2_413*S15-ROcp2_714*C15);
  ROcp2_815 = -(ROcp2_513*S15-ROcp2_814*C15);
  ROcp2_915 = -(ROcp2_613*S15-ROcp2_914*C15);
  ROcp2_116 = ROcp2_114*C16+ROcp2_415*S16;
  ROcp2_216 = ROcp2_214*C16+ROcp2_515*S16;
  ROcp2_316 = ROcp2_314*C16+ROcp2_615*S16;
  ROcp2_416 = -(ROcp2_114*S16-ROcp2_415*C16);
  ROcp2_516 = -(ROcp2_214*S16-ROcp2_515*C16);
  ROcp2_616 = -(ROcp2_314*S16-ROcp2_615*C16);
  ROcp2_117 = ROcp2_116*C17-ROcp2_715*S17;
  ROcp2_217 = ROcp2_216*C17-ROcp2_815*S17;
  ROcp2_317 = ROcp2_316*C17-ROcp2_915*S17;
  ROcp2_717 = ROcp2_116*S17+ROcp2_715*C17;
  ROcp2_817 = ROcp2_216*S17+ROcp2_815*C17;
  ROcp2_917 = ROcp2_316*S17+ROcp2_915*C17;
  RLcp2_113 = ROcp2_16*s->dpt[1][2]+ROcp2_45*s->dpt[2][2]+ROcp2_76*s->dpt[3][2];
  RLcp2_213 = ROcp2_26*s->dpt[1][2]+ROcp2_55*s->dpt[2][2]+ROcp2_86*s->dpt[3][2];
  RLcp2_313 = ROcp2_36*s->dpt[1][2]+ROcp2_96*s->dpt[3][2]+s->dpt[2][2]*S5;
  OMcp2_113 = OMcp2_16+qd[13]*ROcp2_16;
  OMcp2_213 = OMcp2_26+qd[13]*ROcp2_26;
  OMcp2_313 = OMcp2_36+qd[13]*ROcp2_36;
  ORcp2_113 = OMcp2_26*RLcp2_313-OMcp2_36*RLcp2_213;
  ORcp2_213 = -(OMcp2_16*RLcp2_313-OMcp2_36*RLcp2_113);
  ORcp2_313 = OMcp2_16*RLcp2_213-OMcp2_26*RLcp2_113;
  OPcp2_113 = OPcp2_16+qd[13]*(OMcp2_26*ROcp2_36-OMcp2_36*ROcp2_26)+qdd[13]*ROcp2_16;
  OPcp2_213 = OPcp2_26-qd[13]*(OMcp2_16*ROcp2_36-OMcp2_36*ROcp2_16)+qdd[13]*ROcp2_26;
  OPcp2_313 = OPcp2_36+qd[13]*(OMcp2_16*ROcp2_26-OMcp2_26*ROcp2_16)+qdd[13]*ROcp2_36;
  RLcp2_114 = ROcp2_413*s->dpt[2][23];
  RLcp2_214 = ROcp2_513*s->dpt[2][23];
  RLcp2_314 = ROcp2_613*s->dpt[2][23];
  OMcp2_114 = OMcp2_113+qd[14]*ROcp2_413;
  OMcp2_214 = OMcp2_213+qd[14]*ROcp2_513;
  OMcp2_314 = OMcp2_313+qd[14]*ROcp2_613;
  ORcp2_114 = OMcp2_213*RLcp2_314-OMcp2_313*RLcp2_214;
  ORcp2_214 = -(OMcp2_113*RLcp2_314-OMcp2_313*RLcp2_114);
  ORcp2_314 = OMcp2_113*RLcp2_214-OMcp2_213*RLcp2_114;
  OMcp2_115 = OMcp2_114+qd[15]*ROcp2_114;
  OMcp2_215 = OMcp2_214+qd[15]*ROcp2_214;
  OMcp2_315 = OMcp2_314+qd[15]*ROcp2_314;
  OMcp2_116 = OMcp2_115+qd[16]*ROcp2_715;
  OMcp2_216 = OMcp2_215+qd[16]*ROcp2_815;
  OMcp2_316 = OMcp2_315+qd[16]*ROcp2_915;
  OPcp2_116 = OPcp2_113+qd[14]*(OMcp2_213*ROcp2_613-OMcp2_313*ROcp2_513)+qd[15]*(OMcp2_214*ROcp2_314-OMcp2_314*ROcp2_214
 )+qd[16]*(OMcp2_215*ROcp2_915-OMcp2_315*ROcp2_815)+qdd[14]*ROcp2_413+qdd[15]*ROcp2_114+qdd[16]*ROcp2_715;
  OPcp2_216 = OPcp2_213-qd[14]*(OMcp2_113*ROcp2_613-OMcp2_313*ROcp2_413)-qd[15]*(OMcp2_114*ROcp2_314-OMcp2_314*ROcp2_114
 )-qd[16]*(OMcp2_115*ROcp2_915-OMcp2_315*ROcp2_715)+qdd[14]*ROcp2_513+qdd[15]*ROcp2_214+qdd[16]*ROcp2_815;
  OPcp2_316 = OPcp2_313+qd[14]*(OMcp2_113*ROcp2_513-OMcp2_213*ROcp2_413)+qd[15]*(OMcp2_114*ROcp2_214-OMcp2_214*ROcp2_114
 )+qd[16]*(OMcp2_115*ROcp2_815-OMcp2_215*ROcp2_715)+qdd[14]*ROcp2_613+qdd[15]*ROcp2_314+qdd[16]*ROcp2_915;
  RLcp2_117 = ROcp2_715*s->dpt[3][26];
  RLcp2_217 = ROcp2_815*s->dpt[3][26];
  RLcp2_317 = ROcp2_915*s->dpt[3][26];
  ORcp2_117 = OMcp2_216*RLcp2_317-OMcp2_316*RLcp2_217;
  ORcp2_217 = -(OMcp2_116*RLcp2_317-OMcp2_316*RLcp2_117);
  ORcp2_317 = OMcp2_116*RLcp2_217-OMcp2_216*RLcp2_117;
  PxF2[1] = q[1]+RLcp2_113+RLcp2_114+RLcp2_117;
  PxF2[2] = q[2]+RLcp2_213+RLcp2_214+RLcp2_217;
  PxF2[3] = q[3]+RLcp2_313+RLcp2_314+RLcp2_317;
  RxF2[1][1] = ROcp2_117;
  RxF2[1][2] = ROcp2_217;
  RxF2[1][3] = ROcp2_317;
  RxF2[2][1] = ROcp2_416;
  RxF2[2][2] = ROcp2_516;
  RxF2[2][3] = ROcp2_616;
  RxF2[3][1] = ROcp2_717;
  RxF2[3][2] = ROcp2_817;
  RxF2[3][3] = ROcp2_917;
  VxF2[1] = qd[1]+ORcp2_113+ORcp2_114+ORcp2_117;
  VxF2[2] = qd[2]+ORcp2_213+ORcp2_214+ORcp2_217;
  VxF2[3] = qd[3]+ORcp2_313+ORcp2_314+ORcp2_317;
  OMxF2[1] = OMcp2_116+qd[17]*ROcp2_416;
  OMxF2[2] = OMcp2_216+qd[17]*ROcp2_516;
  OMxF2[3] = OMcp2_316+qd[17]*ROcp2_616;
  AxF2[1] = qdd[1]+OMcp2_213*ORcp2_314+OMcp2_216*ORcp2_317+OMcp2_26*ORcp2_313-OMcp2_313*ORcp2_214-OMcp2_316*ORcp2_217-
 OMcp2_36*ORcp2_213+OPcp2_213*RLcp2_314+OPcp2_216*RLcp2_317+OPcp2_26*RLcp2_313-OPcp2_313*RLcp2_214-OPcp2_316*RLcp2_217-
 OPcp2_36*RLcp2_213;
  AxF2[2] = qdd[2]-OMcp2_113*ORcp2_314-OMcp2_116*ORcp2_317-OMcp2_16*ORcp2_313+OMcp2_313*ORcp2_114+OMcp2_316*ORcp2_117+
 OMcp2_36*ORcp2_113-OPcp2_113*RLcp2_314-OPcp2_116*RLcp2_317-OPcp2_16*RLcp2_313+OPcp2_313*RLcp2_114+OPcp2_316*RLcp2_117+
 OPcp2_36*RLcp2_113;
  AxF2[3] = qdd[3]+OMcp2_113*ORcp2_214+OMcp2_116*ORcp2_217+OMcp2_16*ORcp2_213-OMcp2_213*ORcp2_114-OMcp2_216*ORcp2_117-
 OMcp2_26*ORcp2_113+OPcp2_113*RLcp2_214+OPcp2_116*RLcp2_217+OPcp2_16*RLcp2_213-OPcp2_213*RLcp2_114-OPcp2_216*RLcp2_117-
 OPcp2_26*RLcp2_113;
  OMPxF2[1] = OPcp2_116+qd[17]*(OMcp2_216*ROcp2_616-OMcp2_316*ROcp2_516)+qdd[17]*ROcp2_416;
  OMPxF2[2] = OPcp2_216-qd[17]*(OMcp2_116*ROcp2_616-OMcp2_316*ROcp2_416)+qdd[17]*ROcp2_516;
  OMPxF2[3] = OPcp2_316+qd[17]*(OMcp2_116*ROcp2_516-OMcp2_216*ROcp2_416)+qdd[17]*ROcp2_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc13 = ROcp2_117*SWr2[1]+ROcp2_217*SWr2[2]+ROcp2_317*SWr2[3];
  xfrc23 = ROcp2_416*SWr2[1]+ROcp2_516*SWr2[2]+ROcp2_616*SWr2[3];
  xfrc33 = ROcp2_717*SWr2[1]+ROcp2_817*SWr2[2]+ROcp2_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc13;
  frc[2][17] = s->frc[2][17]+xfrc23;
  frc[3][17] = s->frc[3][17]+xfrc33;
  xtrq13 = ROcp2_117*SWr2[4]+ROcp2_217*SWr2[5]+ROcp2_317*SWr2[6];
  xtrq23 = ROcp2_416*SWr2[4]+ROcp2_516*SWr2[5]+ROcp2_616*SWr2[6];
  xtrq33 = ROcp2_717*SWr2[4]+ROcp2_817*SWr2[5]+ROcp2_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq13-xfrc23*SWr2[9]+xfrc33*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq23+xfrc13*SWr2[9]-xfrc33*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq33-xfrc13*SWr2[8]+xfrc23*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
// Sensor Kinematics 


  ROcp3_45 = -S4*C5;
  ROcp3_55 = C4*C5;
  ROcp3_75 = S4*S5;
  ROcp3_85 = -C4*S5;
  ROcp3_16 = -(ROcp3_75*S6-C4*C6);
  ROcp3_26 = -(ROcp3_85*S6-S4*C6);
  ROcp3_36 = -C5*S6;
  ROcp3_76 = ROcp3_75*C6+C4*S6;
  ROcp3_86 = ROcp3_85*C6+S4*S6;
  ROcp3_96 = C5*C6;
  OMcp3_15 = qd[5]*C4;
  OMcp3_25 = qd[5]*S4;
  OMcp3_16 = OMcp3_15+qd[6]*ROcp3_45;
  OMcp3_26 = OMcp3_25+qd[6]*ROcp3_55;
  OMcp3_36 = qd[4]+qd[6]*S5;
  OPcp3_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp3_55-OMcp3_25*S5)-qdd[5]*C4-qdd[6]*ROcp3_45);
  OPcp3_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp3_45-OMcp3_15*S5)+qdd[5]*S4+qdd[6]*ROcp3_55;
  OPcp3_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_3_0_15 = = 
 
// Sensor Kinematics 


  ROcp3_433 = ROcp3_45*C33+ROcp3_76*S33;
  ROcp3_533 = ROcp3_55*C33+ROcp3_86*S33;
  ROcp3_633 = ROcp3_96*S33+C33*S5;
  ROcp3_733 = -(ROcp3_45*S33-ROcp3_76*C33);
  ROcp3_833 = -(ROcp3_55*S33-ROcp3_86*C33);
  ROcp3_933 = ROcp3_96*C33-S33*S5;
  ROcp3_434 = ROcp3_433*C34+ROcp3_733*S34;
  ROcp3_534 = ROcp3_533*C34+ROcp3_833*S34;
  ROcp3_634 = ROcp3_633*C34+ROcp3_933*S34;
  ROcp3_734 = -(ROcp3_433*S34-ROcp3_733*C34);
  ROcp3_834 = -(ROcp3_533*S34-ROcp3_833*C34);
  ROcp3_934 = -(ROcp3_633*S34-ROcp3_933*C34);
  ROcp3_135 = ROcp3_16*C35-ROcp3_734*S35;
  ROcp3_235 = ROcp3_26*C35-ROcp3_834*S35;
  ROcp3_335 = ROcp3_36*C35-ROcp3_934*S35;
  ROcp3_735 = ROcp3_16*S35+ROcp3_734*C35;
  ROcp3_835 = ROcp3_26*S35+ROcp3_834*C35;
  ROcp3_935 = ROcp3_36*S35+ROcp3_934*C35;
  RLcp3_133 = ROcp3_16*s->dpt[1][12]+ROcp3_45*s->dpt[2][12]+ROcp3_76*s->dpt[3][12];
  RLcp3_233 = ROcp3_26*s->dpt[1][12]+ROcp3_55*s->dpt[2][12]+ROcp3_86*s->dpt[3][12];
  RLcp3_333 = ROcp3_36*s->dpt[1][12]+ROcp3_96*s->dpt[3][12]+s->dpt[2][12]*S5;
  OMcp3_133 = OMcp3_16+qd[33]*ROcp3_16;
  OMcp3_233 = OMcp3_26+qd[33]*ROcp3_26;
  OMcp3_333 = OMcp3_36+qd[33]*ROcp3_36;
  ORcp3_133 = OMcp3_26*RLcp3_333-OMcp3_36*RLcp3_233;
  ORcp3_233 = -(OMcp3_16*RLcp3_333-OMcp3_36*RLcp3_133);
  ORcp3_333 = OMcp3_16*RLcp3_233-OMcp3_26*RLcp3_133;
  OPcp3_133 = OPcp3_16+qd[33]*(OMcp3_26*ROcp3_36-OMcp3_36*ROcp3_26)+qdd[33]*ROcp3_16;
  OPcp3_233 = OPcp3_26-qd[33]*(OMcp3_16*ROcp3_36-OMcp3_36*ROcp3_16)+qdd[33]*ROcp3_26;
  OPcp3_333 = OPcp3_36+qd[33]*(OMcp3_16*ROcp3_26-OMcp3_26*ROcp3_16)+qdd[33]*ROcp3_36;
  RLcp3_134 = ROcp3_433*s->dpt[2][46];
  RLcp3_234 = ROcp3_533*s->dpt[2][46];
  RLcp3_334 = ROcp3_633*s->dpt[2][46];
  OMcp3_134 = OMcp3_133+qd[34]*ROcp3_16;
  OMcp3_234 = OMcp3_233+qd[34]*ROcp3_26;
  OMcp3_334 = OMcp3_333+qd[34]*ROcp3_36;
  ORcp3_134 = OMcp3_233*RLcp3_334-OMcp3_333*RLcp3_234;
  ORcp3_234 = -(OMcp3_133*RLcp3_334-OMcp3_333*RLcp3_134);
  ORcp3_334 = OMcp3_133*RLcp3_234-OMcp3_233*RLcp3_134;
  OPcp3_134 = OPcp3_133+qd[34]*(OMcp3_233*ROcp3_36-OMcp3_333*ROcp3_26)+qdd[34]*ROcp3_16;
  OPcp3_234 = OPcp3_233-qd[34]*(OMcp3_133*ROcp3_36-OMcp3_333*ROcp3_16)+qdd[34]*ROcp3_26;
  OPcp3_334 = OPcp3_333+qd[34]*(OMcp3_133*ROcp3_26-OMcp3_233*ROcp3_16)+qdd[34]*ROcp3_36;
  RLcp3_135 = ROcp3_734*s->dpt[3][49];
  RLcp3_235 = ROcp3_834*s->dpt[3][49];
  RLcp3_335 = ROcp3_934*s->dpt[3][49];
  ORcp3_135 = OMcp3_234*RLcp3_335-OMcp3_334*RLcp3_235;
  ORcp3_235 = -(OMcp3_134*RLcp3_335-OMcp3_334*RLcp3_135);
  ORcp3_335 = OMcp3_134*RLcp3_235-OMcp3_234*RLcp3_135;
  PxF3[1] = q[1]+RLcp3_133+RLcp3_134+RLcp3_135;
  PxF3[2] = q[2]+RLcp3_233+RLcp3_234+RLcp3_235;
  PxF3[3] = q[3]+RLcp3_333+RLcp3_334+RLcp3_335;
  RxF3[1][1] = ROcp3_135;
  RxF3[1][2] = ROcp3_235;
  RxF3[1][3] = ROcp3_335;
  RxF3[2][1] = ROcp3_434;
  RxF3[2][2] = ROcp3_534;
  RxF3[2][3] = ROcp3_634;
  RxF3[3][1] = ROcp3_735;
  RxF3[3][2] = ROcp3_835;
  RxF3[3][3] = ROcp3_935;
  VxF3[1] = qd[1]+ORcp3_133+ORcp3_134+ORcp3_135;
  VxF3[2] = qd[2]+ORcp3_233+ORcp3_234+ORcp3_235;
  VxF3[3] = qd[3]+ORcp3_333+ORcp3_334+ORcp3_335;
  OMxF3[1] = OMcp3_134+qd[35]*ROcp3_434;
  OMxF3[2] = OMcp3_234+qd[35]*ROcp3_534;
  OMxF3[3] = OMcp3_334+qd[35]*ROcp3_634;
  AxF3[1] = qdd[1]+OMcp3_233*ORcp3_334+OMcp3_234*ORcp3_335+OMcp3_26*ORcp3_333-OMcp3_333*ORcp3_234-OMcp3_334*ORcp3_235-
 OMcp3_36*ORcp3_233+OPcp3_233*RLcp3_334+OPcp3_234*RLcp3_335+OPcp3_26*RLcp3_333-OPcp3_333*RLcp3_234-OPcp3_334*RLcp3_235-
 OPcp3_36*RLcp3_233;
  AxF3[2] = qdd[2]-OMcp3_133*ORcp3_334-OMcp3_134*ORcp3_335-OMcp3_16*ORcp3_333+OMcp3_333*ORcp3_134+OMcp3_334*ORcp3_135+
 OMcp3_36*ORcp3_133-OPcp3_133*RLcp3_334-OPcp3_134*RLcp3_335-OPcp3_16*RLcp3_333+OPcp3_333*RLcp3_134+OPcp3_334*RLcp3_135+
 OPcp3_36*RLcp3_133;
  AxF3[3] = qdd[3]+OMcp3_133*ORcp3_234+OMcp3_134*ORcp3_235+OMcp3_16*ORcp3_233-OMcp3_233*ORcp3_134-OMcp3_234*ORcp3_135-
 OMcp3_26*ORcp3_133+OPcp3_133*RLcp3_234+OPcp3_134*RLcp3_235+OPcp3_16*RLcp3_233-OPcp3_233*RLcp3_134-OPcp3_234*RLcp3_135-
 OPcp3_26*RLcp3_133;
  OMPxF3[1] = OPcp3_134+qd[35]*(OMcp3_234*ROcp3_634-OMcp3_334*ROcp3_534)+qdd[35]*ROcp3_434;
  OMPxF3[2] = OPcp3_234-qd[35]*(OMcp3_134*ROcp3_634-OMcp3_334*ROcp3_434)+qdd[35]*ROcp3_534;
  OMPxF3[3] = OPcp3_334+qd[35]*(OMcp3_134*ROcp3_534-OMcp3_234*ROcp3_434)+qdd[35]*ROcp3_634;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_135*SWr3[1]+ROcp3_235*SWr3[2]+ROcp3_335*SWr3[3];
  xfrc24 = ROcp3_434*SWr3[1]+ROcp3_534*SWr3[2]+ROcp3_634*SWr3[3];
  xfrc34 = ROcp3_735*SWr3[1]+ROcp3_835*SWr3[2]+ROcp3_935*SWr3[3];
  frc[1][35] = s->frc[1][35]+xfrc14;
  frc[2][35] = s->frc[2][35]+xfrc24;
  frc[3][35] = s->frc[3][35]+xfrc34;
  xtrq14 = ROcp3_135*SWr3[4]+ROcp3_235*SWr3[5]+ROcp3_335*SWr3[6];
  xtrq24 = ROcp3_434*SWr3[4]+ROcp3_534*SWr3[5]+ROcp3_634*SWr3[6];
  xtrq34 = ROcp3_735*SWr3[4]+ROcp3_835*SWr3[5]+ROcp3_935*SWr3[6];
  trq[1][35] = s->trq[1][35]+xtrq14-xfrc24*SWr3[9]+xfrc34*SWr3[8];
  trq[2][35] = s->trq[2][35]+xtrq24+xfrc14*SWr3[9]-xfrc34*SWr3[7];
  trq[3][35] = s->trq[3][35]+xtrq34-xfrc14*SWr3[8]+xfrc24*SWr3[7];

// = = Block_0_0_1_4_0_1 = = 
 
// Sensor Kinematics 


  ROcp4_45 = -S4*C5;
  ROcp4_55 = C4*C5;
  ROcp4_75 = S4*S5;
  ROcp4_85 = -C4*S5;
  ROcp4_16 = -(ROcp4_75*S6-C4*C6);
  ROcp4_26 = -(ROcp4_85*S6-S4*C6);
  ROcp4_36 = -C5*S6;
  ROcp4_76 = ROcp4_75*C6+C4*S6;
  ROcp4_86 = ROcp4_85*C6+S4*S6;
  ROcp4_96 = C5*C6;
  OMcp4_15 = qd[5]*C4;
  OMcp4_25 = qd[5]*S4;
  OMcp4_16 = OMcp4_15+qd[6]*ROcp4_45;
  OMcp4_26 = OMcp4_25+qd[6]*ROcp4_55;
  OMcp4_36 = qd[4]+qd[6]*S5;
  OPcp4_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp4_55-OMcp4_25*S5)-qdd[5]*C4-qdd[6]*ROcp4_45);
  OPcp4_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp4_45-OMcp4_15*S5)+qdd[5]*S4+qdd[6]*ROcp4_55;
  OPcp4_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_4_0_16 = = 
 
// Sensor Kinematics 


  ROcp4_436 = ROcp4_45*C36+ROcp4_76*S36;
  ROcp4_536 = ROcp4_55*C36+ROcp4_86*S36;
  ROcp4_636 = ROcp4_96*S36+C36*S5;
  ROcp4_736 = -(ROcp4_45*S36-ROcp4_76*C36);
  ROcp4_836 = -(ROcp4_55*S36-ROcp4_86*C36);
  ROcp4_936 = ROcp4_96*C36-S36*S5;
  ROcp4_437 = ROcp4_436*C37+ROcp4_736*S37;
  ROcp4_537 = ROcp4_536*C37+ROcp4_836*S37;
  ROcp4_637 = ROcp4_636*C37+ROcp4_936*S37;
  ROcp4_737 = -(ROcp4_436*S37-ROcp4_736*C37);
  ROcp4_837 = -(ROcp4_536*S37-ROcp4_836*C37);
  ROcp4_937 = -(ROcp4_636*S37-ROcp4_936*C37);
  ROcp4_138 = ROcp4_16*C38-ROcp4_737*S38;
  ROcp4_238 = ROcp4_26*C38-ROcp4_837*S38;
  ROcp4_338 = ROcp4_36*C38-ROcp4_937*S38;
  ROcp4_738 = ROcp4_16*S38+ROcp4_737*C38;
  ROcp4_838 = ROcp4_26*S38+ROcp4_837*C38;
  ROcp4_938 = ROcp4_36*S38+ROcp4_937*C38;
  RLcp4_136 = ROcp4_16*s->dpt[1][13]+ROcp4_45*s->dpt[2][13]+ROcp4_76*s->dpt[3][13];
  RLcp4_236 = ROcp4_26*s->dpt[1][13]+ROcp4_55*s->dpt[2][13]+ROcp4_86*s->dpt[3][13];
  RLcp4_336 = ROcp4_36*s->dpt[1][13]+ROcp4_96*s->dpt[3][13]+s->dpt[2][13]*S5;
  OMcp4_136 = OMcp4_16+qd[36]*ROcp4_16;
  OMcp4_236 = OMcp4_26+qd[36]*ROcp4_26;
  OMcp4_336 = OMcp4_36+qd[36]*ROcp4_36;
  ORcp4_136 = OMcp4_26*RLcp4_336-OMcp4_36*RLcp4_236;
  ORcp4_236 = -(OMcp4_16*RLcp4_336-OMcp4_36*RLcp4_136);
  ORcp4_336 = OMcp4_16*RLcp4_236-OMcp4_26*RLcp4_136;
  OPcp4_136 = OPcp4_16+qd[36]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26)+qdd[36]*ROcp4_16;
  OPcp4_236 = OPcp4_26-qd[36]*(OMcp4_16*ROcp4_36-OMcp4_36*ROcp4_16)+qdd[36]*ROcp4_26;
  OPcp4_336 = OPcp4_36+qd[36]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16)+qdd[36]*ROcp4_36;
  RLcp4_137 = ROcp4_436*s->dpt[2][51];
  RLcp4_237 = ROcp4_536*s->dpt[2][51];
  RLcp4_337 = ROcp4_636*s->dpt[2][51];
  OMcp4_137 = OMcp4_136+qd[37]*ROcp4_16;
  OMcp4_237 = OMcp4_236+qd[37]*ROcp4_26;
  OMcp4_337 = OMcp4_336+qd[37]*ROcp4_36;
  ORcp4_137 = OMcp4_236*RLcp4_337-OMcp4_336*RLcp4_237;
  ORcp4_237 = -(OMcp4_136*RLcp4_337-OMcp4_336*RLcp4_137);
  ORcp4_337 = OMcp4_136*RLcp4_237-OMcp4_236*RLcp4_137;
  OPcp4_137 = OPcp4_136+qd[37]*(OMcp4_236*ROcp4_36-OMcp4_336*ROcp4_26)+qdd[37]*ROcp4_16;
  OPcp4_237 = OPcp4_236-qd[37]*(OMcp4_136*ROcp4_36-OMcp4_336*ROcp4_16)+qdd[37]*ROcp4_26;
  OPcp4_337 = OPcp4_336+qd[37]*(OMcp4_136*ROcp4_26-OMcp4_236*ROcp4_16)+qdd[37]*ROcp4_36;
  RLcp4_138 = ROcp4_737*s->dpt[3][53];
  RLcp4_238 = ROcp4_837*s->dpt[3][53];
  RLcp4_338 = ROcp4_937*s->dpt[3][53];
  ORcp4_138 = OMcp4_237*RLcp4_338-OMcp4_337*RLcp4_238;
  ORcp4_238 = -(OMcp4_137*RLcp4_338-OMcp4_337*RLcp4_138);
  ORcp4_338 = OMcp4_137*RLcp4_238-OMcp4_237*RLcp4_138;
  PxF4[1] = q[1]+RLcp4_136+RLcp4_137+RLcp4_138;
  PxF4[2] = q[2]+RLcp4_236+RLcp4_237+RLcp4_238;
  PxF4[3] = q[3]+RLcp4_336+RLcp4_337+RLcp4_338;
  RxF4[1][1] = ROcp4_138;
  RxF4[1][2] = ROcp4_238;
  RxF4[1][3] = ROcp4_338;
  RxF4[2][1] = ROcp4_437;
  RxF4[2][2] = ROcp4_537;
  RxF4[2][3] = ROcp4_637;
  RxF4[3][1] = ROcp4_738;
  RxF4[3][2] = ROcp4_838;
  RxF4[3][3] = ROcp4_938;
  VxF4[1] = qd[1]+ORcp4_136+ORcp4_137+ORcp4_138;
  VxF4[2] = qd[2]+ORcp4_236+ORcp4_237+ORcp4_238;
  VxF4[3] = qd[3]+ORcp4_336+ORcp4_337+ORcp4_338;
  OMxF4[1] = OMcp4_137+qd[38]*ROcp4_437;
  OMxF4[2] = OMcp4_237+qd[38]*ROcp4_537;
  OMxF4[3] = OMcp4_337+qd[38]*ROcp4_637;
  AxF4[1] = qdd[1]+OMcp4_236*ORcp4_337+OMcp4_237*ORcp4_338+OMcp4_26*ORcp4_336-OMcp4_336*ORcp4_237-OMcp4_337*ORcp4_238-
 OMcp4_36*ORcp4_236+OPcp4_236*RLcp4_337+OPcp4_237*RLcp4_338+OPcp4_26*RLcp4_336-OPcp4_336*RLcp4_237-OPcp4_337*RLcp4_238-
 OPcp4_36*RLcp4_236;
  AxF4[2] = qdd[2]-OMcp4_136*ORcp4_337-OMcp4_137*ORcp4_338-OMcp4_16*ORcp4_336+OMcp4_336*ORcp4_137+OMcp4_337*ORcp4_138+
 OMcp4_36*ORcp4_136-OPcp4_136*RLcp4_337-OPcp4_137*RLcp4_338-OPcp4_16*RLcp4_336+OPcp4_336*RLcp4_137+OPcp4_337*RLcp4_138+
 OPcp4_36*RLcp4_136;
  AxF4[3] = qdd[3]+OMcp4_136*ORcp4_237+OMcp4_137*ORcp4_238+OMcp4_16*ORcp4_236-OMcp4_236*ORcp4_137-OMcp4_237*ORcp4_138-
 OMcp4_26*ORcp4_136+OPcp4_136*RLcp4_237+OPcp4_137*RLcp4_238+OPcp4_16*RLcp4_236-OPcp4_236*RLcp4_137-OPcp4_237*RLcp4_138-
 OPcp4_26*RLcp4_136;
  OMPxF4[1] = OPcp4_137+qd[38]*(OMcp4_237*ROcp4_637-OMcp4_337*ROcp4_537)+qdd[38]*ROcp4_437;
  OMPxF4[2] = OPcp4_237-qd[38]*(OMcp4_137*ROcp4_637-OMcp4_337*ROcp4_437)+qdd[38]*ROcp4_537;
  OMPxF4[3] = OPcp4_337+qd[38]*(OMcp4_137*ROcp4_537-OMcp4_237*ROcp4_437)+qdd[38]*ROcp4_637;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_138*SWr4[1]+ROcp4_238*SWr4[2]+ROcp4_338*SWr4[3];
  xfrc25 = ROcp4_437*SWr4[1]+ROcp4_537*SWr4[2]+ROcp4_637*SWr4[3];
  xfrc35 = ROcp4_738*SWr4[1]+ROcp4_838*SWr4[2]+ROcp4_938*SWr4[3];
  frc[1][38] = s->frc[1][38]+xfrc15;
  frc[2][38] = s->frc[2][38]+xfrc25;
  frc[3][38] = s->frc[3][38]+xfrc35;
  xtrq15 = ROcp4_138*SWr4[4]+ROcp4_238*SWr4[5]+ROcp4_338*SWr4[6];
  xtrq25 = ROcp4_437*SWr4[4]+ROcp4_537*SWr4[5]+ROcp4_637*SWr4[6];
  xtrq35 = ROcp4_738*SWr4[4]+ROcp4_838*SWr4[5]+ROcp4_938*SWr4[6];
  trq[1][38] = s->trq[1][38]+xtrq15-xfrc25*SWr4[9]+xfrc35*SWr4[8];
  trq[2][38] = s->trq[2][38]+xtrq25+xfrc15*SWr4[9]-xfrc35*SWr4[7];
  trq[3][38] = s->trq[3][38]+xtrq35-xfrc15*SWr4[8]+xfrc25*SWr4[7];

// = = Block_0_0_1_4_1_0 = = 
 
// Symbolic Outputs  

  frc[1][6] = s->frc[1][6];
  frc[2][6] = s->frc[2][6];
  frc[3][6] = s->frc[3][6];
  frc[1][7] = s->frc[1][7];
  frc[2][7] = s->frc[2][7];
  frc[3][7] = s->frc[3][7];
  frc[1][8] = s->frc[1][8];
  frc[2][8] = s->frc[2][8];
  frc[3][8] = s->frc[3][8];
  frc[1][11] = s->frc[1][11];
  frc[2][11] = s->frc[2][11];
  frc[3][11] = s->frc[3][11];
  frc[1][13] = s->frc[1][13];
  frc[2][13] = s->frc[2][13];
  frc[3][13] = s->frc[3][13];
  frc[1][16] = s->frc[1][16];
  frc[2][16] = s->frc[2][16];
  frc[3][16] = s->frc[3][16];
  frc[1][18] = s->frc[1][18];
  frc[2][18] = s->frc[2][18];
  frc[3][18] = s->frc[3][18];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][20] = s->frc[1][20];
  frc[2][20] = s->frc[2][20];
  frc[3][20] = s->frc[3][20];
  frc[1][21] = s->frc[1][21];
  frc[2][21] = s->frc[2][21];
  frc[3][21] = s->frc[3][21];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
  frc[1][24] = s->frc[1][24];
  frc[2][24] = s->frc[2][24];
  frc[3][24] = s->frc[3][24];
  frc[1][25] = s->frc[1][25];
  frc[2][25] = s->frc[2][25];
  frc[3][25] = s->frc[3][25];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  frc[1][28] = s->frc[1][28];
  frc[2][28] = s->frc[2][28];
  frc[3][28] = s->frc[3][28];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
  frc[1][32] = s->frc[1][32];
  frc[2][32] = s->frc[2][32];
  frc[3][32] = s->frc[3][32];
  frc[1][33] = s->frc[1][33];
  frc[2][33] = s->frc[2][33];
  frc[3][33] = s->frc[3][33];
  frc[1][34] = s->frc[1][34];
  frc[2][34] = s->frc[2][34];
  frc[3][34] = s->frc[3][34];
  frc[1][36] = s->frc[1][36];
  frc[2][36] = s->frc[2][36];
  frc[3][36] = s->frc[3][36];
  frc[1][37] = s->frc[1][37];
  frc[2][37] = s->frc[2][37];
  frc[3][37] = s->frc[3][37];
  frc[1][39] = s->frc[1][39];
  frc[2][39] = s->frc[2][39];
  frc[3][39] = s->frc[3][39];
  frc[1][40] = s->frc[1][40];
  frc[2][40] = s->frc[2][40];
  frc[3][40] = s->frc[3][40];
  frc[1][42] = s->frc[1][42];
  frc[2][42] = s->frc[2][42];
  frc[3][42] = s->frc[3][42];
  frc[1][44] = s->frc[1][44];
  frc[2][44] = s->frc[2][44];
  frc[3][44] = s->frc[3][44];
  trq[1][6] = s->trq[1][6];
  trq[2][6] = s->trq[2][6];
  trq[3][6] = s->trq[3][6];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][13] = s->trq[1][13];
  trq[2][13] = s->trq[2][13];
  trq[3][13] = s->trq[3][13];
  trq[1][16] = s->trq[1][16];
  trq[2][16] = s->trq[2][16];
  trq[3][16] = s->trq[3][16];
  trq[1][18] = s->trq[1][18];
  trq[2][18] = s->trq[2][18];
  trq[3][18] = s->trq[3][18];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][20] = s->trq[1][20];
  trq[2][20] = s->trq[2][20];
  trq[3][20] = s->trq[3][20];
  trq[1][21] = s->trq[1][21];
  trq[2][21] = s->trq[2][21];
  trq[3][21] = s->trq[3][21];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][24] = s->trq[1][24];
  trq[2][24] = s->trq[2][24];
  trq[3][24] = s->trq[3][24];
  trq[1][25] = s->trq[1][25];
  trq[2][25] = s->trq[2][25];
  trq[3][25] = s->trq[3][25];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];
  trq[1][28] = s->trq[1][28];
  trq[2][28] = s->trq[2][28];
  trq[3][28] = s->trq[3][28];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];
  trq[1][32] = s->trq[1][32];
  trq[2][32] = s->trq[2][32];
  trq[3][32] = s->trq[3][32];
  trq[1][33] = s->trq[1][33];
  trq[2][33] = s->trq[2][33];
  trq[3][33] = s->trq[3][33];
  trq[1][34] = s->trq[1][34];
  trq[2][34] = s->trq[2][34];
  trq[3][34] = s->trq[3][34];
  trq[1][36] = s->trq[1][36];
  trq[2][36] = s->trq[2][36];
  trq[3][36] = s->trq[3][36];
  trq[1][37] = s->trq[1][37];
  trq[2][37] = s->trq[2][37];
  trq[3][37] = s->trq[3][37];
  trq[1][39] = s->trq[1][39];
  trq[2][39] = s->trq[2][39];
  trq[3][39] = s->trq[3][39];
  trq[1][40] = s->trq[1][40];
  trq[2][40] = s->trq[2][40];
  trq[3][40] = s->trq[3][40];
  trq[1][42] = s->trq[1][42];
  trq[2][42] = s->trq[2][42];
  trq[3][42] = s->trq[3][42];
  trq[1][44] = s->trq[1][44];
  trq[2][44] = s->trq[2][44];
  trq[3][44] = s->trq[3][44];

// ====== END Task 0 ====== 


}
 

