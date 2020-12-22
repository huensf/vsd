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
#include "user_model.h"

#define pi 3.1415

void user_DrivenJoints(MbsData *mbs_data,double tsim)
{
	double Deltat = tsim - mbs_data->user_model->PI.Deltat;
	mbs_data->user_model->PI.Deltat = tsim;
	double R = 33.42;

	if (tsim >= 0.90)
	{
		double v = 10.0;
		double Theta_ref = (pi - (v/R)*(tsim-0.9));
		double y_ref = (mbs_data->user_model->PI.center + mbs_data->user_model->PI.yoffset) - R*cos(Theta_ref);
		double x_ref = mbs_data->user_model->PI.xoffset + R*sin(Theta_ref);

		if(tsim >= 6.15)
		{
			Theta_ref = pi/2;
		}

		double posX = mbs_data->q[T1_chassis_id] - mbs_data->user_model->PI.xoffset;
		double posY = (-R - mbs_data->q[T2_chassis_id] - mbs_data->user_model->PI.yoffset);

		//printf("posX : %f, posY : %f \n", posX, posY);

		//double Theta_now = pi + atan(posX/posY);
		double Theta_now = pi + mbs_data->q[R3_chassis_id];

		//printf("Theta_ref : %f , Theta_now : %f \n", Theta_ref*180/pi, Theta_now*180/pi);
		
		double Error_on_Theta = Theta_now - Theta_ref;

		mbs_data->user_model->PI.Integral_Error = mbs_data->user_model->PI.Integral_Error + Error_on_Theta*Deltat;

		double Kp = mbs_data->user_model->PI.Kp;
		double Ki = mbs_data->user_model->PI.Ki;
		double SumError = mbs_data->user_model->PI.Integral_Error;

		mbs_data->q[T2_rack_id] = Kp*Error_on_Theta + Ki*SumError;
	}
	else 
	{
		mbs_data->user_model->PI.xoffset = mbs_data->q[T1_chassis_id];
		mbs_data->user_model->PI.yoffset = mbs_data->q[T2_chassis_id];
		mbs_data->user_model->PI.center = mbs_data->q[T2_chassis_id] - R;
		mbs_data->q[T2_rack_id] = 0.0;
	}
}

 
