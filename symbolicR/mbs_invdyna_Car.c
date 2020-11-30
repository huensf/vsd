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
//	==> Generation Date : Mon Nov 30 14:42:10 2020
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 44
//
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 2172
//
//	==> Generation Time :  0.050 seconds
//	==> Post-Processing :  0.020 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)

// double Qq[44];
{ 
 
#include "mbs_invdyna_Car.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

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

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C20 = cos(q[20]);
  S20 = sin(q[20]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C22 = cos(q[22]);
  S22 = sin(q[22]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);

// = = Block_0_0_0_0_0_11 = = 
 
// Trigonometric Variables  

  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_13 = = 
 
// Trigonometric Variables  

  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);

// = = Block_0_0_0_0_0_14 = = 
 
// Trigonometric Variables  

  C31 = cos(q[31]);
  S31 = sin(q[31]);
  C32 = cos(q[32]);
  S32 = sin(q[32]);

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

// = = Block_0_0_0_0_0_17 = = 
 
// Trigonometric Variables  

  C39 = cos(q[39]);
  S39 = sin(q[39]);

// = = Block_0_0_0_0_0_18 = = 
 
// Trigonometric Variables  

  C40 = cos(q[40]);
  S40 = sin(q[40]);
  C41 = cos(q[41]);
  S41 = sin(q[41]);
  C42 = cos(q[42]);
  S42 = sin(q[42]);

// = = Block_0_0_0_0_0_19 = = 
 
// Trigonometric Variables  

  C43 = cos(q[43]);
  S43 = sin(q[43]);
  C44 = cos(q[44]);
  S44 = sin(q[44]);

// = = Block_0_1_0_0_0_0 = = 
 
// Forward Kinematics 

  ALPHA33 = qdd[3]-s->g[3];
  ALPHA14 = qdd[1]*C4+qdd[2]*S4;
  ALPHA24 = -(qdd[1]*S4-qdd[2]*C4);
  OM35 = qd[4]*C5;
  OMp35 = -(qd[4]*qd[5]*S5-qdd[4]*C5);
  ALPHA25 = ALPHA24*C5+ALPHA33*S5;
  ALPHA35 = -(ALPHA24*S5-ALPHA33*C5);
  OM16 = qd[5]*C6-OM35*S6;
  OM26 = qd[6]+qd[4]*S5;
  OM36 = qd[5]*S6+OM35*C6;
  OMp16 = C6*(qdd[5]-qd[6]*OM35)-S6*(OMp35+qd[5]*qd[6]);
  OMp26 = qdd[6]+qd[4]*qd[5]*C5+qdd[4]*S5;
  OMp36 = C6*(OMp35+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM35);
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BETA26 = BS26-OMp36;
  BETA36 = BS36+OMp26;
  BETA46 = BS26+OMp36;
  BETA66 = BS66-OMp16;
  BETA76 = BS36-OMp26;
  BETA86 = BS66+OMp16;
  ALPHA16 = ALPHA14*C6-ALPHA35*S6;
  ALPHA36 = ALPHA14*S6+ALPHA35*C6;
  OM18 = qd[8]+OM16;
  OM28 = OM26*C8+OM36*S8;
  OM38 = -(OM26*S8-OM36*C8);
  OMp18 = qdd[8]+OMp16;
  OMp38 = C8*(OMp36-qd[8]*OM26)-S8*(OMp26+qd[8]*OM36);
  BETA28 = OM18*OM28-OMp38;
  BETA88 = OMp18+OM28*OM38;
  ALPHA18 = ALPHA16+BETA26*s->dpt[2][1]+BETA36*s->dpt[3][1]+BS16*s->dpt[1][1];
  ALPHA38 = C8*(ALPHA36+BETA76*s->dpt[1][1]+BETA86*s->dpt[2][1]+BS96*s->dpt[3][1])-S8*(ALPHA25+BETA46*s->dpt[1][1]+
 BETA66*s->dpt[3][1]+BS56*s->dpt[2][1]);
  OM29 = qd[9]+OM28;
  OM39 = OM18*S9+OM38*C9;
  OMp29 = qdd[9]+C8*(OMp26+qd[8]*OM36)+S8*(OMp36-qd[8]*OM26);
  OMp39 = C9*(OMp38+qd[9]*OM18)+S9*(OMp18-qd[9]*OM38);
  ALPHA19 = C9*(ALPHA18+BETA28*s->dpt[2][17])-S9*(ALPHA38+BETA88*s->dpt[2][17]);
  ALPHA29 = C8*(ALPHA25+BETA46*s->dpt[1][1]+BETA66*s->dpt[3][1]+BS56*s->dpt[2][1])+S8*(ALPHA36+BETA76*s->dpt[1][1]+
 BETA86*s->dpt[2][1]+BS96*s->dpt[3][1])-s->dpt[2][17]*(OM18*OM18+OM38*OM38);
  ALPHA39 = C9*(ALPHA38+BETA88*s->dpt[2][17])+S9*(ALPHA18+BETA28*s->dpt[2][17]);
  OM110 = qd[10]+OM18*C9-OM38*S9;
  OM210 = OM29*C10+OM39*S10;
  OMp110 = qdd[10]+C9*(OMp18-qd[9]*OM38)-S9*(OMp38+qd[9]*OM18);
  OMp210 = C10*(OMp29+qd[10]*OM39)+S10*(OMp39-qd[10]*OM29);
  ALPHA210 = ALPHA29*C10+ALPHA39*S10;
  ALPHA310 = -(ALPHA29*S10-ALPHA39*C10);
  OM111 = OM110*C11+OM210*S11;
  OM211 = -(OM110*S11-OM210*C11);
  OM311 = qd[11]-OM29*S10+OM39*C10;
  OMp111 = C11*(OMp110+qd[11]*OM210)+S11*(OMp210-qd[11]*OM110);
  OMp211 = C11*(OMp210-qd[11]*OM110)-S11*(OMp110+qd[11]*OM210);
  OMp311 = qdd[11]+C10*(OMp39-qd[10]*OM29)-S10*(OMp29+qd[10]*OM39);
  BS911 = -(OM111*OM111+OM211*OM211);
  BETA311 = OMp211+OM111*OM311;
  ALPHA111 = ALPHA19*C11+ALPHA210*S11;
  ALPHA211 = -(ALPHA19*S11-ALPHA210*C11);
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[12]+OM211;
  OM312 = OM111*S12+OM311*C12;
  OM113 = qd[13]+OM16;
  OM213 = OM26*C13+OM36*S13;
  OM313 = -(OM26*S13-OM36*C13);
  OMp113 = qdd[13]+OMp16;
  OMp313 = C13*(OMp36-qd[13]*OM26)-S13*(OMp26+qd[13]*OM36);
  BETA213 = OM113*OM213-OMp313;
  BETA813 = OMp113+OM213*OM313;
  ALPHA113 = ALPHA16+BETA26*s->dpt[2][2]+BETA36*s->dpt[3][2]+BS16*s->dpt[1][2];
  ALPHA313 = C13*(ALPHA36+BETA76*s->dpt[1][2]+BETA86*s->dpt[2][2]+BS96*s->dpt[3][2])-S13*(ALPHA25+BETA46*s->dpt[1][2]+
 BETA66*s->dpt[3][2]+BS56*s->dpt[2][2]);
  OM214 = qd[14]+OM213;
  OM314 = OM113*S14+OM313*C14;
  OMp214 = qdd[14]+C13*(OMp26+qd[13]*OM36)+S13*(OMp36-qd[13]*OM26);
  OMp314 = C14*(OMp313+qd[14]*OM113)+S14*(OMp113-qd[14]*OM313);
  ALPHA114 = C14*(ALPHA113+BETA213*s->dpt[2][23])-S14*(ALPHA313+BETA813*s->dpt[2][23]);
  ALPHA214 = C13*(ALPHA25+BETA46*s->dpt[1][2]+BETA66*s->dpt[3][2]+BS56*s->dpt[2][2])+S13*(ALPHA36+BETA76*s->dpt[1][2]+
 BETA86*s->dpt[2][2]+BS96*s->dpt[3][2])-s->dpt[2][23]*(OM113*OM113+OM313*OM313);
  ALPHA314 = C14*(ALPHA313+BETA813*s->dpt[2][23])+S14*(ALPHA113+BETA213*s->dpt[2][23]);
  OM115 = qd[15]+OM113*C14-OM313*S14;
  OM215 = OM214*C15+OM314*S15;
  OMp115 = qdd[15]+C14*(OMp113-qd[14]*OM313)-S14*(OMp313+qd[14]*OM113);
  OMp215 = C15*(OMp214+qd[15]*OM314)+S15*(OMp314-qd[15]*OM214);
  ALPHA215 = ALPHA214*C15+ALPHA314*S15;
  ALPHA315 = -(ALPHA214*S15-ALPHA314*C15);
  OM116 = OM115*C16+OM215*S16;
  OM216 = -(OM115*S16-OM215*C16);
  OM316 = qd[16]-OM214*S15+OM314*C15;
  OMp116 = C16*(OMp115+qd[16]*OM215)+S16*(OMp215-qd[16]*OM115);
  OMp216 = C16*(OMp215-qd[16]*OM115)-S16*(OMp115+qd[16]*OM215);
  OMp316 = qdd[16]+C15*(OMp314-qd[15]*OM214)-S15*(OMp214+qd[15]*OM314);
  BS916 = -(OM116*OM116+OM216*OM216);
  BETA316 = OMp216+OM116*OM316;
  ALPHA116 = ALPHA114*C16+ALPHA215*S16;
  ALPHA216 = -(ALPHA114*S16-ALPHA215*C16);
  OM117 = OM116*C17-OM316*S17;
  OM217 = qd[17]+OM216;
  OM317 = OM116*S17+OM316*C17;
  OM133 = qd[33]+OM16;
  OM233 = OM26*C33+OM36*S33;
  OM333 = -(OM26*S33-OM36*C33);
  OMp133 = qdd[33]+OMp16;
  OMp233 = C33*(OMp26+qd[33]*OM36)+S33*(OMp36-qd[33]*OM26);
  OMp333 = C33*(OMp36-qd[33]*OM26)-S33*(OMp26+qd[33]*OM36);
  BS533 = -(OM133*OM133+OM333*OM333);
  BETA833 = OMp133+OM233*OM333;
  ALPHA233 = C33*(ALPHA25+BETA46*s->dpt[1][12]+BETA66*s->dpt[3][12]+BS56*s->dpt[2][12])+S33*(ALPHA36+BETA76*
 s->dpt[1][12]+BETA86*s->dpt[2][12]+BS96*s->dpt[3][12]);
  ALPHA333 = C33*(ALPHA36+BETA76*s->dpt[1][12]+BETA86*s->dpt[2][12]+BS96*s->dpt[3][12])-S33*(ALPHA25+BETA46*
 s->dpt[1][12]+BETA66*s->dpt[3][12]+BS56*s->dpt[2][12]);
  OM134 = qd[34]+OM133;
  OM234 = OM233*C34+OM333*S34;
  OM334 = -(OM233*S34-OM333*C34);
  OMp134 = qdd[34]+OMp133;
  OMp234 = C34*(OMp233+qd[34]*OM333)+S34*(OMp333-qd[34]*OM233);
  OMp334 = C34*(OMp333-qd[34]*OM233)-S34*(OMp233+qd[34]*OM333);
  BS934 = -(OM134*OM134+OM234*OM234);
  BETA334 = OMp234+OM134*OM334;
  ALPHA134 = ALPHA16+BETA26*s->dpt[2][12]+BETA36*s->dpt[3][12]+BS16*s->dpt[1][12]-s->dpt[2][46]*(OMp333-OM133*OM233);
  ALPHA234 = C34*(ALPHA233+BS533*s->dpt[2][46])+S34*(ALPHA333+BETA833*s->dpt[2][46]);
  ALPHA334 = C34*(ALPHA333+BETA833*s->dpt[2][46])-S34*(ALPHA233+BS533*s->dpt[2][46]);
  OM135 = OM134*C35-OM334*S35;
  OM235 = qd[35]+OM234;
  OM335 = OM134*S35+OM334*C35;
  OM136 = qd[36]+OM16;
  OM236 = OM26*C36+OM36*S36;
  OM336 = -(OM26*S36-OM36*C36);
  OMp136 = qdd[36]+OMp16;
  OMp236 = C36*(OMp26+qd[36]*OM36)+S36*(OMp36-qd[36]*OM26);
  OMp336 = C36*(OMp36-qd[36]*OM26)-S36*(OMp26+qd[36]*OM36);
  BS536 = -(OM136*OM136+OM336*OM336);
  BETA836 = OMp136+OM236*OM336;
  ALPHA236 = C36*(ALPHA25+BETA46*s->dpt[1][13]+BETA66*s->dpt[3][13]+BS56*s->dpt[2][13])+S36*(ALPHA36+BETA76*
 s->dpt[1][13]+BETA86*s->dpt[2][13]+BS96*s->dpt[3][13]);
  ALPHA336 = C36*(ALPHA36+BETA76*s->dpt[1][13]+BETA86*s->dpt[2][13]+BS96*s->dpt[3][13])-S36*(ALPHA25+BETA46*
 s->dpt[1][13]+BETA66*s->dpt[3][13]+BS56*s->dpt[2][13]);
  OM137 = qd[37]+OM136;
  OM237 = OM236*C37+OM336*S37;
  OM337 = -(OM236*S37-OM336*C37);
  OMp137 = qdd[37]+OMp136;
  OMp237 = C37*(OMp236+qd[37]*OM336)+S37*(OMp336-qd[37]*OM236);
  OMp337 = C37*(OMp336-qd[37]*OM236)-S37*(OMp236+qd[37]*OM336);
  BS937 = -(OM137*OM137+OM237*OM237);
  BETA337 = OMp237+OM137*OM337;
  ALPHA137 = ALPHA16+BETA26*s->dpt[2][13]+BETA36*s->dpt[3][13]+BS16*s->dpt[1][13]-s->dpt[2][51]*(OMp336-OM136*OM236);
  ALPHA237 = C37*(ALPHA236+BS536*s->dpt[2][51])+S37*(ALPHA336+BETA836*s->dpt[2][51]);
  ALPHA337 = C37*(ALPHA336+BETA836*s->dpt[2][51])-S37*(ALPHA236+BS536*s->dpt[2][51]);
  OM138 = OM137*C38-OM337*S38;
  OM238 = qd[38]+OM237;
  OM338 = OM137*S38+OM337*C38;
 
// Backward Dynamics 

  Fq143 = -(s->frc[1][44]*C44+s->frc[3][44]*S44);
  Fq343 = s->frc[1][44]*S44-s->frc[3][44]*C44;
  Cq143 = -(s->trq[1][44]*C44+s->trq[3][44]*S44);
  Cq343 = s->trq[1][44]*S44-s->trq[3][44]*C44;
  Fq141 = -(s->frc[1][42]*C42+s->frc[3][42]*S42);
  Fq341 = s->frc[1][42]*S42-s->frc[3][42]*C42;
  Cq141 = -(s->trq[1][42]*C42+s->trq[3][42]*S42);
  Cq341 = s->trq[1][42]*S42-s->trq[3][42]*C42;
  Fq140 = -(s->frc[1][40]-Fq141);
  Fq340 = -(s->frc[3][40]+s->frc[2][42]*S41-Fq341*C41);
  Cq140 = -(s->trq[1][40]-Cq141+s->dpt[2][58]*(s->frc[2][42]*S41-Fq341*C41));
  Cq240 = -(s->trq[2][40]+s->trq[2][42]*C41+Cq341*S41-s->dpt[1][58]*(s->frc[2][42]*S41-Fq341*C41));
  Cq340 = -(s->trq[3][40]+s->trq[2][42]*S41-Cq341*C41+Fq141*s->dpt[2][58]+s->dpt[1][58]*(s->frc[2][42]*C41+Fq341*S41));
  Fq139 = -(s->frc[1][39]-Fq143-Fq140*C40-Fq340*S40);
  Fq239 = -(s->frc[2][39]+s->frc[2][40]+s->frc[2][42]*C41+s->frc[2][44]*C43+Fq341*S41+Fq343*S43);
  Fq339 = -(s->frc[3][39]+s->frc[2][44]*S43+Fq140*S40-Fq340*C40-Fq343*C43);
  Cq139 = -(s->trq[1][39]-Cq143-Cq140*C40-Cq340*S40+s->dpt[2][57]*(s->frc[2][44]*S43-Fq343*C43));
  Cq239 = -(s->trq[2][39]-Cq240+s->trq[2][44]*C43+Cq343*S43-s->dpt[1][57]*(s->frc[2][44]*S43-Fq343*C43));
  Cq339 = -(s->trq[3][39]+s->trq[2][44]*S43+Cq140*S40-Cq340*C40-Cq343*C43+Fq143*s->dpt[2][57]+s->dpt[1][57]*(
 s->frc[2][44]*C43+Fq343*S43));
  Fs138 = -(s->frc[1][38]-s->m[38]*(C38*(ALPHA137+BETA337*s->dpt[3][53])-S38*(ALPHA337+BS937*s->dpt[3][53])));
  Fs238 = -(s->frc[2][38]-s->m[38]*(ALPHA237-s->dpt[3][53]*(OMp137-OM237*OM337)));
  Fs338 = -(s->frc[3][38]-s->m[38]*(C38*(ALPHA337+BS937*s->dpt[3][53])+S38*(ALPHA137+BETA337*s->dpt[3][53])));
  Cq138 = -(s->trq[1][38]-s->In[1][38]*(C38*(OMp137-qd[38]*OM337)-S38*(OMp337+qd[38]*OM137))+OM238*OM338*(s->In[5][38]-
 s->In[9][38]));
  Cq238 = -(s->trq[2][38]-s->In[5][38]*(qdd[38]+OMp237)-OM138*OM338*(s->In[1][38]-s->In[9][38]));
  Cq338 = -(s->trq[3][38]-s->In[9][38]*(C38*(OMp337+qd[38]*OM137)+S38*(OMp137-qd[38]*OM337))+OM138*OM238*(s->In[1][38]-
 s->In[5][38]));
  Fq137 = -(s->frc[1][37]-s->m[37]*ALPHA137-Fs138*C38-Fs338*S38);
  Fq237 = -(s->frc[2][37]-Fs238-s->m[37]*ALPHA237);
  Fq337 = -(s->frc[3][37]-s->m[37]*ALPHA337+Fs138*S38-Fs338*C38);
  Cq137 = -(s->trq[1][37]-Cq138*C38-Cq338*S38+Fs238*s->dpt[3][53]);
  Cq237 = -(s->trq[2][37]-Cq238-s->dpt[3][53]*(Fs138*C38+Fs338*S38));
  Cq337 = -(s->trq[3][37]+Cq138*S38-Cq338*C38);
  Fq136 = -(s->frc[1][36]-Fq137);
  Fq236 = -(s->frc[2][36]-Fq237*C37+Fq337*S37);
  Fq336 = -(s->frc[3][36]-Fq237*S37-Fq337*C37);
  Cq136 = -(s->trq[1][36]-Cq137-s->dpt[2][51]*(Fq237*S37+Fq337*C37));
  Cq236 = -(s->trq[2][36]-Cq237*C37+Cq337*S37);
  Cq336 = -(s->trq[3][36]-Cq237*S37-Cq337*C37+Fq137*s->dpt[2][51]);
  Fs135 = -(s->frc[1][35]-s->m[35]*(C35*(ALPHA134+BETA334*s->dpt[3][49])-S35*(ALPHA334+BS934*s->dpt[3][49])));
  Fs235 = -(s->frc[2][35]-s->m[35]*(ALPHA234-s->dpt[3][49]*(OMp134-OM234*OM334)));
  Fs335 = -(s->frc[3][35]-s->m[35]*(C35*(ALPHA334+BS934*s->dpt[3][49])+S35*(ALPHA134+BETA334*s->dpt[3][49])));
  Cq135 = -(s->trq[1][35]-s->In[1][35]*(C35*(OMp134-qd[35]*OM334)-S35*(OMp334+qd[35]*OM134))+OM235*OM335*(s->In[5][35]-
 s->In[9][35]));
  Cq235 = -(s->trq[2][35]-s->In[5][35]*(qdd[35]+OMp234)-OM135*OM335*(s->In[1][35]-s->In[9][35]));
  Cq335 = -(s->trq[3][35]-s->In[9][35]*(C35*(OMp334+qd[35]*OM134)+S35*(OMp134-qd[35]*OM334))+OM135*OM235*(s->In[1][35]-
 s->In[5][35]));
  Fq134 = -(s->frc[1][34]-s->m[34]*ALPHA134-Fs135*C35-Fs335*S35);
  Fq234 = -(s->frc[2][34]-Fs235-s->m[34]*ALPHA234);
  Fq334 = -(s->frc[3][34]-s->m[34]*ALPHA334+Fs135*S35-Fs335*C35);
  Cq134 = -(s->trq[1][34]-Cq135*C35-Cq335*S35+Fs235*s->dpt[3][49]);
  Cq234 = -(s->trq[2][34]-Cq235-s->dpt[3][49]*(Fs135*C35+Fs335*S35));
  Cq334 = -(s->trq[3][34]+Cq135*S35-Cq335*C35);
  Fq133 = -(s->frc[1][33]-Fq134);
  Fq233 = -(s->frc[2][33]-Fq234*C34+Fq334*S34);
  Fq333 = -(s->frc[3][33]-Fq234*S34-Fq334*C34);
  Cq133 = -(s->trq[1][33]-Cq134-s->dpt[2][46]*(Fq234*S34+Fq334*C34));
  Cq233 = -(s->trq[2][33]-Cq234*C34+Cq334*S34);
  Cq333 = -(s->trq[3][33]-Cq234*S34-Cq334*C34+Fq134*s->dpt[2][46]);
  Fq131 = -(s->frc[1][32]*C32-s->frc[2][32]*S32);
  Fq231 = -(s->frc[1][32]*S32+s->frc[2][32]*C32);
  Cq131 = -(s->trq[1][32]*C32-s->trq[2][32]*S32);
  Cq231 = -(s->trq[1][32]*S32+s->trq[2][32]*C32);
  Fq129 = -(s->frc[1][30]*C30-s->frc[2][30]*S30);
  Fq229 = -(s->frc[1][30]*S30+s->frc[2][30]*C30);
  Cq129 = -(s->trq[1][30]*C30-s->trq[2][30]*S30);
  Cq229 = -(s->trq[1][30]*S30+s->trq[2][30]*C30);
  Fq128 = -(s->frc[1][28]-Fq129-Fq131-s->m[28]*(ALPHA16+q[28]*BETA26-(2.0)*qd[28]*OM36+BETA36*s->dpt[3][9]+BS16*s->dpt[1][9])
 );
  Fq228 = -(s->frc[2][28]-s->frc[3][30]*S29-s->frc[3][32]*S31-s->m[28]*(qdd[28]+ALPHA25+q[28]*BS56+BETA46*s->dpt[1][9]+
 BETA66*s->dpt[3][9])-Fq229*C29-Fq231*C31);
  Fq328 = -(s->frc[3][28]+s->frc[3][30]*C29+s->frc[3][32]*C31-s->m[28]*(ALPHA36+q[28]*BETA86+(2.0)*qd[28]*OM16+BETA76*
 s->dpt[1][9]+BS96*s->dpt[3][9])-Fq229*S29-Fq231*S31);
  Fq126 = -(s->frc[1][27]*C27+s->frc[3][27]*S27);
  Fq326 = s->frc[1][27]*S27-s->frc[3][27]*C27;
  Cq126 = -(s->trq[1][27]*C27+s->trq[3][27]*S27);
  Cq326 = s->trq[1][27]*S27-s->trq[3][27]*C27;
  Fq125 = -(s->frc[1][25]-Fq126);
  Fq325 = -(s->frc[3][25]+s->frc[2][27]*S26-Fq326*C26);
  Cq125 = -(s->trq[1][25]-Cq126+s->dpt[2][40]*(s->frc[2][27]*S26-Fq326*C26));
  Cq225 = -(s->trq[2][25]+s->trq[2][27]*C26+Cq326*S26-s->dpt[1][40]*(s->frc[2][27]*S26-Fq326*C26));
  Cq325 = -(s->trq[3][25]+s->trq[2][27]*S26-Cq326*C26+Fq126*s->dpt[2][40]+s->dpt[1][40]*(s->frc[2][27]*C26+Fq326*S26));
  Fq123 = -(s->frc[1][24]*C24+s->frc[3][24]*S24);
  Fq323 = s->frc[1][24]*S24-s->frc[3][24]*C24;
  Cq123 = -(s->trq[1][24]*C24+s->trq[3][24]*S24);
  Cq323 = s->trq[1][24]*S24-s->trq[3][24]*C24;
  Fq122 = -(s->frc[1][22]-Fq123-Fq125*C25-Fq325*S25);
  Fq222 = -(s->frc[2][22]+s->frc[2][25]+s->frc[2][24]*C23+s->frc[2][27]*C26+Fq323*S23+Fq326*S26);
  Fq322 = -(s->frc[3][22]+s->frc[2][24]*S23+Fq125*S25-Fq323*C23-Fq325*C25);
  Cq122 = -(s->trq[1][22]-Cq123-Cq125*C25-Cq325*S25+s->dpt[2][37]*(s->frc[2][24]*S23-Fq323*C23));
  Cq222 = -(s->trq[2][22]-Cq225+s->trq[2][24]*C23+Cq323*S23-s->dpt[1][37]*(s->frc[2][24]*S23-Fq323*C23));
  Cq322 = -(s->trq[3][22]+s->trq[2][24]*S23+Cq125*S25-Cq323*C23-Cq325*C25+Fq123*s->dpt[2][37]+s->dpt[1][37]*(
 s->frc[2][24]*C23+Fq323*S23));
  Fs117 = -(s->frc[1][17]-s->m[17]*(C17*(ALPHA116+BETA316*s->dpt[3][26])-S17*(ALPHA315+BS916*s->dpt[3][26])));
  Fs217 = -(s->frc[2][17]-s->m[17]*(ALPHA216-s->dpt[3][26]*(OMp116-OM216*OM316)));
  Fs317 = -(s->frc[3][17]-s->m[17]*(C17*(ALPHA315+BS916*s->dpt[3][26])+S17*(ALPHA116+BETA316*s->dpt[3][26])));
  Cq117 = -(s->trq[1][17]-s->In[1][17]*(C17*(OMp116-qd[17]*OM316)-S17*(OMp316+qd[17]*OM116))+OM217*OM317*(s->In[5][17]-
 s->In[9][17]));
  Cq217 = -(s->trq[2][17]-s->In[5][17]*(qdd[17]+OMp216)-OM117*OM317*(s->In[1][17]-s->In[9][17]));
  Cq317 = -(s->trq[3][17]-s->In[9][17]*(C17*(OMp316+qd[17]*OM116)+S17*(OMp116-qd[17]*OM316))+OM117*OM217*(s->In[1][17]-
 s->In[5][17]));
  Fq116 = -(s->frc[1][16]-s->m[16]*ALPHA116-Fs117*C17-Fs317*S17);
  Fq216 = -(s->frc[2][16]-Fs217-s->m[16]*ALPHA216);
  Fq316 = -(s->frc[3][16]-s->m[16]*ALPHA315+Fs117*S17-Fs317*C17);
  Cq116 = -(s->trq[1][16]-Cq117*C17-Cq317*S17+Fs217*s->dpt[3][26]);
  Cq216 = -(s->trq[2][16]-Cq217-s->dpt[3][26]*(Fs117*C17+Fs317*S17));
  Cq316 = -(s->trq[3][16]+Cq117*S17-Cq317*C17);
  Fq115 = Fq116*C16-Fq216*S16;
  Fq215 = Fq116*S16+Fq216*C16;
  Cq115 = Cq116*C16-Cq216*S16;
  Cq215 = Cq116*S16+Cq216*C16;
  Fq314 = Fq215*S15+Fq316*C15;
  Cq214 = Cq215*C15-Cq316*S15;
  Cq314 = Cq215*S15+Cq316*C15;
  Fq113 = -(s->frc[1][13]-Fq115*C14-Fq314*S14);
  Fq213 = -(s->frc[2][13]-Fq215*C15+Fq316*S15);
  Fq313 = -(s->frc[3][13]+Fq115*S14-Fq314*C14);
  Cq113 = -(s->trq[1][13]-Cq115*C14-Cq314*S14+s->dpt[2][23]*(Fq115*S14-Fq314*C14));
  Cq213 = -(s->trq[2][13]-Cq214);
  Cq313 = -(s->trq[3][13]+Cq115*S14-Cq314*C14+s->dpt[2][23]*(Fq115*C14+Fq314*S14));
  Fs112 = -(s->frc[1][12]-s->m[12]*(C12*(ALPHA111+BETA311*s->dpt[3][21])-S12*(ALPHA310+BS911*s->dpt[3][21])));
  Fs212 = -(s->frc[2][12]-s->m[12]*(ALPHA211-s->dpt[3][21]*(OMp111-OM211*OM311)));
  Fs312 = -(s->frc[3][12]-s->m[12]*(C12*(ALPHA310+BS911*s->dpt[3][21])+S12*(ALPHA111+BETA311*s->dpt[3][21])));
  Cq112 = -(s->trq[1][12]-s->In[1][12]*(C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111))+OM212*OM312*(s->In[5][12]-
 s->In[9][12]));
  Cq212 = -(s->trq[2][12]-s->In[5][12]*(qdd[12]+OMp211)-OM112*OM312*(s->In[1][12]-s->In[9][12]));
  Cq312 = -(s->trq[3][12]-s->In[9][12]*(C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311))+OM112*OM212*(s->In[1][12]-
 s->In[5][12]));
  Fq111 = -(s->frc[1][11]-s->m[11]*ALPHA111-Fs112*C12-Fs312*S12);
  Fq211 = -(s->frc[2][11]-Fs212-s->m[11]*ALPHA211);
  Fq311 = -(s->frc[3][11]-s->m[11]*ALPHA310+Fs112*S12-Fs312*C12);
  Cq111 = -(s->trq[1][11]-Cq112*C12-Cq312*S12+Fs212*s->dpt[3][21]);
  Cq211 = -(s->trq[2][11]-Cq212-s->dpt[3][21]*(Fs112*C12+Fs312*S12));
  Cq311 = -(s->trq[3][11]+Cq112*S12-Cq312*C12);
  Fq110 = Fq111*C11-Fq211*S11;
  Fq210 = Fq111*S11+Fq211*C11;
  Cq110 = Cq111*C11-Cq211*S11;
  Cq210 = Cq111*S11+Cq211*C11;
  Fq39 = Fq210*S10+Fq311*C10;
  Cq29 = Cq210*C10-Cq311*S10;
  Cq39 = Cq210*S10+Cq311*C10;
  Fq18 = -(s->frc[1][8]-Fq110*C9-Fq39*S9);
  Fq28 = -(s->frc[2][8]-Fq210*C10+Fq311*S10);
  Fq38 = -(s->frc[3][8]+Fq110*S9-Fq39*C9);
  Cq18 = -(s->trq[1][8]-Cq110*C9-Cq39*S9+s->dpt[2][17]*(Fq110*S9-Fq39*C9));
  Cq28 = -(s->trq[2][8]-Cq29);
  Cq38 = -(s->trq[3][8]+Cq110*S9-Cq39*C9+s->dpt[2][17]*(Fq110*C9+Fq39*S9));
  Fs17 = -(s->frc[1][7]-s->m[7]*(qdd[7]+ALPHA16+q[7]*BS16));
  Fs27 = -(s->frc[2][7]-s->m[7]*(ALPHA25+q[7]*BETA46+(2.0)*qd[7]*OM36));
  Fs37 = -(s->frc[3][7]-s->m[7]*(ALPHA36+q[7]*BETA76-(2.0)*qd[7]*OM26));
  Fs16 = -(s->frc[1][6]-s->m[6]*(ALPHA16+BETA36*s->l[3][6]+BS16*s->l[1][6]));
  Fs26 = -(s->frc[2][6]-s->m[6]*(ALPHA25+BETA46*s->l[1][6]+BETA66*s->l[3][6]));
  Fs36 = -(s->frc[3][6]-s->m[6]*(ALPHA36+BETA76*s->l[1][6]+BS96*s->l[3][6]));
  Fq16 = -(s->frc[1][18]+s->frc[1][19]+s->frc[1][20]+s->frc[1][21]-Fq113-Fq128-Fq133-Fq136-Fq18-Fs16-Fs17-Fq122*C22-
 Fq139*C39-Fq322*S22-Fq339*S39);
  Fq26 = Fq222+Fq228+Fq239+Fs26+Fs27-s->frc[2][18]*C18-s->frc[2][19]*C19-s->frc[2][20]*C20-s->frc[2][21]*C21+
 s->frc[3][18]*S18+s->frc[3][19]*S19+s->frc[3][20]*S20+s->frc[3][21]*S21+Fq213*C13+Fq233*C33+Fq236*C36+Fq28*C8-Fq313*S13-
 Fq333*S33-Fq336*S36-Fq38*S8;
  Fq36 = Fq328+Fs36+Fs37-s->frc[2][18]*S18-s->frc[2][19]*S19-s->frc[2][20]*S20-s->frc[2][21]*S21-s->frc[3][18]*C18-
 s->frc[3][19]*C19-s->frc[3][20]*C20-s->frc[3][21]*C21-Fq122*S22-Fq139*S39+Fq213*S13+Fq233*S33+Fq236*S36+Fq28*S8+Fq313*C13+
 Fq322*C22+Fq333*C33+Fq336*C36+Fq339*C39+Fq38*C8;
  Cq16 = -(s->trq[1][18]+s->trq[1][19]+s->trq[1][20]+s->trq[1][21]+s->trq[1][28]+s->trq[1][6]+s->trq[1][7]-Cq113-Cq129-
 Cq131-Cq133-Cq136-Cq18-q[28]*Fq328-s->In[1][6]*OMp16-Cq122*C22-Cq139*C39-Cq322*S22-Cq339*S39+Fq222*s->dpt[3][8]+Fq228*
 s->dpt[3][9]+Fq239*s->dpt[3][15]+Fs26*s->l[3][6]+OM26*OM36*(s->In[5][6]-s->In[9][6])-s->dpt[2][12]*(Fq233*S33+Fq333*C33)-
 s->dpt[2][13]*(Fq236*S36+Fq336*C36)-s->dpt[2][1]*(Fq28*S8+Fq38*C8)-s->dpt[2][2]*(Fq213*S13+Fq313*C13)+s->dpt[2][3]*(
 s->frc[2][18]*S18+s->frc[3][18]*C18)+s->dpt[2][42]*(s->frc[3][30]*C29-Fq229*S29)+s->dpt[2][43]*(s->frc[3][32]*C31-Fq231*S31)
 +s->dpt[2][4]*(s->frc[2][19]*S19+s->frc[3][19]*C19)+s->dpt[2][5]*(s->frc[2][20]*S20+s->frc[3][20]*C20)+s->dpt[2][7]*(
 s->frc[2][21]*S21+s->frc[3][21]*C21)+s->dpt[3][12]*(Fq233*C33-Fq333*S33)+s->dpt[3][13]*(Fq236*C36-Fq336*S36)+s->dpt[3][1]*(
 Fq28*C8-Fq38*S8)+s->dpt[3][2]*(Fq213*C13-Fq313*S13));
  Cq26 = -(s->trq[2][28]+s->trq[2][6]+s->trq[2][7]-Cq222-Cq239+q[7]*Fs37-s->In[5][6]*OMp26+s->trq[2][18]*C18+
 s->trq[2][19]*C19+s->trq[2][20]*C20+s->trq[2][21]*C21-s->trq[3][18]*S18-s->trq[3][19]*S19-s->trq[3][20]*S20-s->trq[3][21]*
 S21-s->trq[3][30]*S29-s->trq[3][32]*S31-Cq213*C13-Cq229*C29-Cq231*C31-Cq233*C33-Cq236*C36-Cq28*C8+Cq313*S13+Cq333*S33+Cq336*
 S36+Cq38*S8-Fq113*s->dpt[3][2]-Fq128*s->dpt[3][9]-Fq133*s->dpt[3][12]-Fq136*s->dpt[3][13]-Fq18*s->dpt[3][1]+Fq328*
 s->dpt[1][9]-Fs16*s->l[3][6]+Fs36*s->l[1][6]-OM16*OM36*(s->In[1][6]-s->In[9][6])+s->dpt[1][12]*(Fq233*S33+Fq333*C33)+
 s->dpt[1][13]*(Fq236*S36+Fq336*C36)-s->dpt[1][15]*(Fq139*S39-Fq339*C39)+s->dpt[1][1]*(Fq28*S8+Fq38*C8)+s->dpt[1][2]*(Fq213*
 S13+Fq313*C13)-s->dpt[1][5]*(s->frc[2][20]*S20+s->frc[3][20]*C20)-s->dpt[1][7]*(s->frc[2][21]*S21+s->frc[3][21]*C21)-
 s->dpt[1][8]*(Fq122*S22-Fq322*C22)-s->dpt[3][15]*(Fq139*C39+Fq339*S39)-s->dpt[3][8]*(Fq122*C22+Fq322*S22));
  Cq36 = -(s->trq[3][28]+s->trq[3][6]+s->trq[3][7]+q[28]*Fq128-q[7]*Fs27-s->In[9][6]*OMp36-s->frc[1][18]*s->dpt[2][3]-
 s->frc[1][19]*s->dpt[2][4]-s->frc[1][20]*s->dpt[2][5]-s->frc[1][21]*s->dpt[2][7]+s->trq[2][18]*S18+s->trq[2][19]*S19+
 s->trq[2][20]*S20+s->trq[2][21]*S21+s->trq[3][18]*C18+s->trq[3][19]*C19+s->trq[3][20]*C20+s->trq[3][21]*C21+s->trq[3][30]*
 C29+s->trq[3][32]*C31+Cq122*S22+Cq139*S39-Cq213*S13-Cq229*S29-Cq231*S31-Cq233*S33-Cq236*S36-Cq28*S8-Cq313*C13-Cq322*C22-
 Cq333*C33-Cq336*C36-Cq339*C39-Cq38*C8+Fq113*s->dpt[2][2]+Fq129*s->dpt[2][42]+Fq131*s->dpt[2][43]+Fq133*s->dpt[2][12]+Fq136*
 s->dpt[2][13]+Fq18*s->dpt[2][1]-Fq222*s->dpt[1][8]-Fq228*s->dpt[1][9]-Fq239*s->dpt[1][15]-Fs26*s->l[1][6]+OM16*OM26*(
 s->In[1][6]-s->In[5][6])-s->dpt[1][12]*(Fq233*C33-Fq333*S33)-s->dpt[1][13]*(Fq236*C36-Fq336*S36)-s->dpt[1][1]*(Fq28*C8-Fq38*
 S8)-s->dpt[1][2]*(Fq213*C13-Fq313*S13)+s->dpt[1][5]*(s->frc[2][20]*C20-s->frc[3][20]*S20)+s->dpt[1][7]*(s->frc[2][21]*C21-
 s->frc[3][21]*S21));
  Fq15 = Fq16*C6+Fq36*S6;
  Fq35 = -(Fq16*S6-Fq36*C6);
  Cq15 = Cq16*C6+Cq36*S6;
  Fq24 = Fq26*C5-Fq35*S5;
  Fq34 = Fq26*S5+Fq35*C5;
  Cq34 = Cq26*S5-C5*(Cq16*S6-Cq36*C6);
  Fq13 = Fq15*C4-Fq24*S4;
  Fq23 = Fq15*S4+Fq24*C4;

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  Qq[1] = Fq13;
  Qq[2] = Fq23;
  Qq[3] = Fq34;
  Qq[4] = Cq34;
  Qq[5] = Cq15;
  Qq[6] = Cq26;
  Qq[7] = Fs17;
  Qq[8] = Cq18;
  Qq[9] = Cq29;
  Qq[10] = Cq110;
  Qq[11] = Cq311;
  Qq[12] = Cq212;
  Qq[13] = Cq113;
  Qq[14] = Cq214;
  Qq[15] = Cq115;
  Qq[16] = Cq316;
  Qq[17] = Cq217;
  Qq[18] = -s->trq[1][18];
  Qq[19] = -s->trq[1][19];
  Qq[20] = -s->trq[1][20];
  Qq[21] = -s->trq[1][21];
  Qq[22] = Cq222;
  Qq[23] = Cq123;
  Qq[24] = -s->trq[2][24];
  Qq[25] = Cq225;
  Qq[26] = Cq126;
  Qq[27] = -s->trq[2][27];
  Qq[28] = Fq228;
  Qq[29] = Cq129;
  Qq[30] = -s->trq[3][30];
  Qq[31] = Cq131;
  Qq[32] = -s->trq[3][32];
  Qq[33] = Cq133;
  Qq[34] = Cq134;
  Qq[35] = Cq235;
  Qq[36] = Cq136;
  Qq[37] = Cq137;
  Qq[38] = Cq238;
  Qq[39] = Cq239;
  Qq[40] = Cq240;
  Qq[41] = Cq141;
  Qq[42] = -s->trq[2][42];
  Qq[43] = Cq143;
  Qq[44] = -s->trq[2][44];

// ====== END Task 0 ====== 


}
 

