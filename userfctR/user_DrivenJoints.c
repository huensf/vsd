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
#include "user_all_id.h"

void user_DrivenJoints(MbsData *mbs_data,double tsim)
{
	if (tsim >= 0.90 && tsim <= 6.15)
		mbs_data->q[T2_rack_id] = 0.01;
	else 
		mbs_data->q[T2_rack_id] = 0.0;
}

 
