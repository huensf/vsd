//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h" 

#include "mbs_data.h"
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"

#include "user_all_id.h"
#include "user_model.h"

double* user_JointForces(MbsData *mbs_data, double tsim)
{
/*-- Begin of user code --*/

    if (tsim == 0.0) // equilibrium process and modal analysis
    {
        mbs_data->Qq[R2_wheel_rr_lt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[R2_wheel_rr_rt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[T2_rack_id] = mbs_data->user_model->EquilQuantities.Qrack;
    }

    // anti-roll bar
    mbs_data->Qq[R2_def_bar_ft_id] = -mbs_data->user_model->FrontSuspension.C_bar*mbs_data->q[R2_def_bar_ft_id];
    mbs_data->Qq[R2_def_bar_rr_id] = -mbs_data->user_model->RearSuspension.C_bar*mbs_data->q[R2_def_bar_rr_id];

/*-- End of user code --*/

    return mbs_data->Qq;
}
