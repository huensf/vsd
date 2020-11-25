//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 21/11/2017
//---------------------------

#include "mbs_matrix.h"
#include "math.h"
#include "useful_functions.h"

void tgc_car_kine_wheel(double x_I[4],double R_Y_I[4][4],
                    double xd_I[4],double omYI_I[4],double rnom,
                    double *pen_p,double *rz_p, double *angslip_p, double *angcamb_p,
                    double *slip_p, double u_I[4], double umd_I[4], double Rt_T_I[4][4], double dxF[4])
{
    /* There are two sets of notations
    - the user notations from the declaration .h
    - the book notations used in this .c file

    x_I     = Pw        [in]
    R_Y_I   = Rw        [in]
    xd_I    = Vw        [in]
    omYI_I  = OMw       [in]
    umd_I   = Vmct      [out]
    Rt_T_I  = Rtground  [out]
    */

    // wheel kinematics variables
    double pen;
    double sinf, tg_angslip;
    double angslip, angcamb, slip;

    // rotation matrix variables
    double Rt_Y_I[4][4] = { 0.0 };
    double R_T_I[4][4] = { 0.0 };
    double R_S_I[4][4] = { 0.0 };
    double Rt_S_I[4][4] = { 0.0 };
    double eS1_I[4] = { 0.0 }, eS2_I[4] = { 0.0 }, eS3_I[4] = { 0.0 };
    double eT1_I[4] = { 0.0 }, eT2_I[4] = { 0.0 }, eT3_I[4] = { 0.0 };
    double eX2proj_I[4] = { 0.0 };

    double omYI_x_v[4], omXI_x_v[4];
    double v_S[4], v_I[4];
    double up_I[4];
    double rho;
    double ud_I[4], ud_T[4];
    double umd_T[4];
    double xd_T[4];
    double omYI_Y[4], omXI_Y[4], omXI_I[4];
    /*
    Scheme of the different points (IDEALLY SHOULD BE CHANGED SO THAT IT FITS THE BOOK POINT LABELLING)

                            /
                           /
                          /
                         /
                        . G  (wheel center)
                       /
                      /
                     /
                    /
   ..............  .  ................................  .  .........................
                   Q (contact point )                   O (origin point)
    
    Model Assumptions
    - local flat and horizontal ground
    - frozen wheel frame has a camber and roll angle of max pi/4 (for which reason ?)
    Conventions
    - wheel spin axis has to be a Y axis (i.e. associated to a R2 rotation)
    - normal to the ground is point upward (because of z definition)
    Frame Definitions:
    - [I]       : inertial frame
    - [Y]   : rotating wheel frame
    - [S]   : wheel tangential frame
    - [T]   : wheel/ground contact frame aligned with the wheel along the long. direction and aligned with the ground normal vector (i.e., the vertical for a flat ground)
    Rotation matrices
    [Y] = R_Y_I*[I]
    [I] = Rt_Y_I*[Y]
    ...

    */

    /*	Frame Definition */
    transpose(R_Y_I, Rt_Y_I);
    // build the R_T_I matrix
    eX2proj_I[1] = R_Y_I[2][1];
    eX2proj_I[2] = R_Y_I[2][2];
    eX2proj_I[3] = 0.0;
    normalize_dvec_1(eX2proj_I, eT2_I, 3);
    // because the ground is flat 
    eT3_I[1] = 0.0;
    eT3_I[2] = 0.0;
    eT3_I[3] = 1.0;
    // by definition of unit vector
    cross_product(eT2_I, eT3_I, eT1_I);
    // by definition of rotation matrix
    R_T_I[1][1] = eT1_I[1];
    R_T_I[1][2] = eT1_I[2];
    R_T_I[1][3] = eT1_I[3];
    R_T_I[2][1] = eT2_I[1];
    R_T_I[2][2] = eT2_I[2];
    R_T_I[2][3] = eT2_I[3];
    R_T_I[3][1] = eT3_I[1];
    R_T_I[3][2] = eT3_I[2];
    R_T_I[3][3] = eT3_I[3];
    // get the transpose
    transpose(R_T_I, Rt_T_I);

    // build the R_S_I matrix 
    // as we have Y2=S2; 
    eS2_I[1] = R_Y_I[2][1];
    eS2_I[2] = R_Y_I[2][2];
    eS2_I[3] = R_Y_I[2][3];
    //  as we have T1=S1; 
    eS1_I[1] = eT1_I[1];
    eS1_I[2] = eT1_I[2];
    eS1_I[3] = eT1_I[3];
    // by definition of unit vector
    cross_product(eS1_I, eS2_I, eS3_I);
    // by definition of rotation matrix
    R_S_I[1][1] = eS1_I[1];
    R_S_I[1][2] = eS1_I[2];
    R_S_I[1][3] = eS1_I[3];
    R_S_I[2][1] = eS2_I[1];
    R_S_I[2][2] = eS2_I[2];
    R_S_I[2][3] = eS2_I[3];
    R_S_I[3][1] = eS3_I[1];
    R_S_I[3][2] = eS3_I[2];
    R_S_I[3][3] = eS3_I[3];
    // get the transpose
    transpose(R_S_I, Rt_S_I);

    /*	Camber Angle (f) */
    //	def: angle formed by the wheel median plane with respect to the normal to the ground.
    sinf = R_Y_I[2][3];       	//! valid on horizontal ground !
    angcamb = asin(sinf);
    *angcamb_p = angcamb;

    /*	Contact Point Kinematics */

    //--Position--
    // ground profile under the wheel center
    up_I[1] = x_I[1];
    up_I[2] = x_I[2];
    up_I[3] = 0.0; // user_GroundLevel(pOP_I[1],pOP_I[2]);
    // distance between the wheel center and the intersection of frame [S] with the flat ground.
    rho = (x_I[3]- up_I[3])/cos(angcamb);
    *rz_p = rho; 
    // GQ vector expressed in frame [T]
    v_S[1] = 0;
    v_S[2] = 0;
    v_S[3] = -rho;
    // GQ vector expressed in frame [I]
    matrix_product(Rt_S_I, v_S, v_I);
    // absolute position of point Q (expressed in [I])
    vector_sum(x_I, v_I, u_I);

    // vertical deflection calculation
    pen =  cos(angcamb)*(rnom - rho);
    *pen_p = pen;
    // Vector of force application (from the wheel center, G to the contact point, Q) expressed in [Y]. 
    matrix_product(R_Y_I, v_I, dxF);
    
    //--Speed--
    // driving velocity of the material contact point (expressed in [I])
    cross_product(omYI_I, v_I, omYI_x_v);
    // (absolute) velocity of the material contact point (expressed in [I])
    vector_sum(xd_I, omYI_x_v , umd_I);
    // (absolute) velocity of the material contact point (expressed in [T])
    matrix_product(R_T_I, umd_I, umd_T);

    /* lateral slip angle (angliss) */
    //  def: angle of the geometric point velocity with respect to the wheel median plane.  (computed in Rsol frame).
    // wheel center velocity expressed in [T]
    matrix_product(R_T_I, xd_I, xd_T);
    //frozen wheel angular velocity (expressed in [Y])
    matrix_product(R_Y_I, omYI_I, omYI_Y);
    omXI_Y[1] = omYI_Y[1];
    omXI_Y[2] = 0;
    omXI_Y[3] = omYI_Y[3];
    matrix_product(Rt_Y_I, omXI_Y, omXI_I);
    // driving velocity of geometrical contact point (expressed in [I])
    cross_product(omXI_I, v_I, omXI_x_v);
    // velocity of geometrical contact point (expressed in [I])
    vector_sum(xd_I, omXI_x_v, ud_I);
    // velocity of geometrical contact point (expressed in [T])
    matrix_product(R_T_I, ud_I, ud_T);
    // angle computation
    if ((ud_T[1] >= 1e-3) | (ud_T[1] <= -1e-3))
    {
        tg_angslip = ud_T[2] / ud_T[1];
    }
    else
    {
        tg_angslip = 0;
    }
    angslip = atan(tg_angslip);
    *angslip_p = angslip;

    /* Longitudinal slip (gliss) */
    //  def: ratio between the material contact point longitudinal speed and the wheel center long. speed (expressed in [T])
    // longitudinal speed of the material contact point : umd_T(1)
    // longitudinal speed of the wheel center           : xd_T(1)
    if ((xd_T[1] >= 1e-3) | (xd_T[1] <= -1e-3))
    {
        slip = -umd_T[1] / xd_T[1];
    }
    else
    {
        slip = 0;
    }
    *slip_p = slip;

}