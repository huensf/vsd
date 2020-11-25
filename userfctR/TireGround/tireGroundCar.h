#ifndef TIREGROUNDCAR_h
#define TIREGROUNDCAR_h



/**
* \brief compute car wheel kinematics which are necessary inputs for a tire/ground contact model.
*        Note that the car tire is defined by
*           - a nominal radius (from wheel center to contact patch)
*
* \p Pw  [in], Position of the wheel center in the inertial frame.
* \p Rw  [in], Rotation Matrix of the wheel body.
* \p Vw  [in], Velocity of the wheel center in the inertial frame.
* \p OMw [in], Angular velocity of the wheel center in the inertial frame.
* \p rnom       [in], wheel nominal radius 
* \p pen_p      [out], tire vertical deflection.
* \p rz_p       [out], distance between the wheel center and the intersection of frame [S] with the flat ground.
* \p angslip_p  [out], pointer for lateral slip angle.
* \p angcamb_p  [out], pointer for camber angle.
* \p slip_p     [out], pointer for longitudinal slip.
* \p Pct,         [out], Position of the contact point expressed in the inertial frame.
* \p Vmct  [out], Velocity of the material point in contact expressed in the inertial frame.
* \p Rt_ground       [out], transpose of Rotation matrix from [I] to [T].
* \p dxF        [out], Vector of force application (from the wheel center to the contact point) expressed in the rotating wheel frame [Y]
*/
void tgc_car_kine_wheel(double Pw[4], double Rw[4][4],
    double Vw[4], double OMw[4], double rnom,
    double *pen_p, double *rz_p, double *angslip_p, double *angcamb_p,
    double *slip_p, double Pcontact[4], double Vcontact[4], double Rt_ground[4][4], double dxF[4]);

/**
* \brief compute tire/ground contact forces according to the bakker model.
*
* \p Fwhl [out], tire/ground contact forces expressed in [R] frame.
* \p Mwhl,[out], tire/ground contact torques expressed in [R] frame.
* \p angslip  [in], lateral slip angle expressed in rad.
* \p angcamb  [in], camber angle expressed in rad.
* \p slip     [in], longitudinal slip expressed in rad.
*/
void tgc_bakker_contact(double Fwhl[4], double Mwhl[4], double angslip, double angcamb, double slip);

#endif