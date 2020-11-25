/* --------------------------------------------------------
 * This code was generated automatically by MBsysC modules.
 * MBsysC modules are distributed as part of the ROBOTRAN 
 * software. They provides functionalities for dealing with
 * symbolic equations generated by ROBOTRAN. 
 *
 * More info on www.robotran.be 
 *
 * Universite catholique de Louvain, Belgium 
 *
 * Last update : Mon Nov  2 08:49:27 2020
 * --------------------------------------------------------
 *
 */
#include "mbs_path.h"
#include "user_model.h"
#include "mbs_xml_binder_public.h"
#include "mbs_load_xml.h"
#include "useful_functions.h"
#include "math.h"

// ============================================================ //


UserModel* mbs_new_user_model() 
{
    UserModel* um;
    um = (UserModel*)malloc(sizeof(UserModel));
    um->FrontSuspension.K = 0.0;
    um->FrontSuspension.D = 0.0;
    um->FrontSuspension.L0 = 0.0;
    um->FrontSuspension.C_bar = 0.0;
 
    um->RearSuspension.K = 0.0;
    um->RearSuspension.D = 0.0;
    um->RearSuspension.L0 = 0.0;
    um->RearSuspension.C_bar = 0.0;
 
    um->FrontTire.R = 0.0;
    um->FrontTire.K = 0.0;
 
    um->RearTire.R = 0.0;
    um->RearTire.K = 0.0;
 
    um->EquilQuantities.Qpropulsion = 0.0;
    um->EquilQuantities.Qrack = 0.0;
 
    um->SteeringAssembly.delta_left = 0.0;
    um->SteeringAssembly.delta_right = 0.0;
 
    um->Rotations.R11G = 0.0;
    um->Rotations.R11D = 0.0;
    um->Rotations.R12G = 0.0;
    um->Rotations.R12D = 0.0;
    um->Rotations.R11C = 0.0;
    um->Rotations.R12C = 0.0;
 
    return um;
}

void mbs_delete_user_model(UserModel* um) 
{
    free(um);
}

 void mbs_load_user_model_xml(MbsInfos* mbs_infos, UserModel* um) 
{
    int ind_state_value = 1;

    um->FrontSuspension.K = mbs_infos->user_models->user_model_list[0]->parameter_list[0]->value_list[1];
    um->FrontSuspension.D = mbs_infos->user_models->user_model_list[0]->parameter_list[1]->value_list[1];
    um->FrontSuspension.L0 = mbs_infos->user_models->user_model_list[0]->parameter_list[2]->value_list[1];
    um->FrontSuspension.C_bar = mbs_infos->user_models->user_model_list[0]->parameter_list[3]->value_list[1];
 
    um->RearSuspension.K = mbs_infos->user_models->user_model_list[1]->parameter_list[0]->value_list[1];
    um->RearSuspension.D = mbs_infos->user_models->user_model_list[1]->parameter_list[1]->value_list[1];
    um->RearSuspension.L0 = mbs_infos->user_models->user_model_list[1]->parameter_list[2]->value_list[1];
    um->RearSuspension.C_bar = mbs_infos->user_models->user_model_list[1]->parameter_list[3]->value_list[1];
 
    um->FrontTire.R = mbs_infos->user_models->user_model_list[2]->parameter_list[0]->value_list[1];
    um->FrontTire.K = mbs_infos->user_models->user_model_list[2]->parameter_list[1]->value_list[1];
 
    um->RearTire.R = mbs_infos->user_models->user_model_list[3]->parameter_list[0]->value_list[1];
    um->RearTire.K = mbs_infos->user_models->user_model_list[3]->parameter_list[1]->value_list[1];
 
    um->EquilQuantities.Qpropulsion = mbs_infos->user_models->user_model_list[4]->parameter_list[0]->value_list[1];
    um->EquilQuantities.Qrack = mbs_infos->user_models->user_model_list[4]->parameter_list[1]->value_list[1];
 
    um->SteeringAssembly.delta_left = mbs_infos->user_models->user_model_list[5]->parameter_list[0]->value_list[1];
    um->SteeringAssembly.delta_right = mbs_infos->user_models->user_model_list[5]->parameter_list[1]->value_list[1];
 
    um->Rotations.R11G = mbs_infos->user_models->user_model_list[6]->parameter_list[0]->value_list[1];
    um->Rotations.R11D = mbs_infos->user_models->user_model_list[6]->parameter_list[1]->value_list[1];
    um->Rotations.R12G = mbs_infos->user_models->user_model_list[6]->parameter_list[2]->value_list[1];
    um->Rotations.R12D = mbs_infos->user_models->user_model_list[6]->parameter_list[3]->value_list[1];
    um->Rotations.R11C = mbs_infos->user_models->user_model_list[6]->parameter_list[4]->value_list[1];
    um->Rotations.R12C = mbs_infos->user_models->user_model_list[6]->parameter_list[5]->value_list[1];
 
}

 void mbs_bind_user_model(MbsInfos* mbs_infos, UserModel* um) 
{
    mbs_infos->user_models->user_model_list[0]->parameter_list[0]->val_ptr = &um->FrontSuspension.K;
    mbs_infos->user_models->user_model_list[0]->parameter_list[1]->val_ptr = &um->FrontSuspension.D;
    mbs_infos->user_models->user_model_list[0]->parameter_list[2]->val_ptr = &um->FrontSuspension.L0;
    mbs_infos->user_models->user_model_list[0]->parameter_list[3]->val_ptr = &um->FrontSuspension.C_bar;
 
    mbs_infos->user_models->user_model_list[1]->parameter_list[0]->val_ptr = &um->RearSuspension.K;
    mbs_infos->user_models->user_model_list[1]->parameter_list[1]->val_ptr = &um->RearSuspension.D;
    mbs_infos->user_models->user_model_list[1]->parameter_list[2]->val_ptr = &um->RearSuspension.L0;
    mbs_infos->user_models->user_model_list[1]->parameter_list[3]->val_ptr = &um->RearSuspension.C_bar;
 
    mbs_infos->user_models->user_model_list[2]->parameter_list[0]->val_ptr = &um->FrontTire.R;
    mbs_infos->user_models->user_model_list[2]->parameter_list[1]->val_ptr = &um->FrontTire.K;
 
    mbs_infos->user_models->user_model_list[3]->parameter_list[0]->val_ptr = &um->RearTire.R;
    mbs_infos->user_models->user_model_list[3]->parameter_list[1]->val_ptr = &um->RearTire.K;
 
    mbs_infos->user_models->user_model_list[4]->parameter_list[0]->val_ptr = &um->EquilQuantities.Qpropulsion;
    mbs_infos->user_models->user_model_list[4]->parameter_list[1]->val_ptr = &um->EquilQuantities.Qrack;
 
    mbs_infos->user_models->user_model_list[5]->parameter_list[0]->val_ptr = &um->SteeringAssembly.delta_left;
    mbs_infos->user_models->user_model_list[5]->parameter_list[1]->val_ptr = &um->SteeringAssembly.delta_right;
 
    mbs_infos->user_models->user_model_list[6]->parameter_list[0]->val_ptr = &um->Rotations.R11G;
    mbs_infos->user_models->user_model_list[6]->parameter_list[1]->val_ptr = &um->Rotations.R11D;
    mbs_infos->user_models->user_model_list[6]->parameter_list[2]->val_ptr = &um->Rotations.R12G;
    mbs_infos->user_models->user_model_list[6]->parameter_list[3]->val_ptr = &um->Rotations.R12D;
    mbs_infos->user_models->user_model_list[6]->parameter_list[4]->val_ptr = &um->Rotations.R11C;
    mbs_infos->user_models->user_model_list[6]->parameter_list[5]->val_ptr = &um->Rotations.R12C;
 
}
 
 void mbs_print_user_model(UserModel* um) 
{

    printf("user_model->FrontSuspension.K=%f\n", um->FrontSuspension.K);
    printf("user_model->FrontSuspension.D=%f\n", um->FrontSuspension.D);
    printf("user_model->FrontSuspension.L0=%f\n", um->FrontSuspension.L0);
    printf("user_model->FrontSuspension.C_bar=%f\n", um->FrontSuspension.C_bar);
 
    printf("user_model->RearSuspension.K=%f\n", um->RearSuspension.K);
    printf("user_model->RearSuspension.D=%f\n", um->RearSuspension.D);
    printf("user_model->RearSuspension.L0=%f\n", um->RearSuspension.L0);
    printf("user_model->RearSuspension.C_bar=%f\n", um->RearSuspension.C_bar);
 
    printf("user_model->FrontTire.R=%f\n", um->FrontTire.R);
    printf("user_model->FrontTire.K=%f\n", um->FrontTire.K);
 
    printf("user_model->RearTire.R=%f\n", um->RearTire.R);
    printf("user_model->RearTire.K=%f\n", um->RearTire.K);
 
    printf("user_model->EquilQuantities.Qpropulsion=%f\n", um->EquilQuantities.Qpropulsion);
    printf("user_model->EquilQuantities.Qrack=%f\n", um->EquilQuantities.Qrack);
 
    printf("user_model->SteeringAssembly.delta_left=%f\n", um->SteeringAssembly.delta_left);
    printf("user_model->SteeringAssembly.delta_right=%f\n", um->SteeringAssembly.delta_right);
 
    printf("user_model->Rotations.R11G=%f\n", um->Rotations.R11G);
    printf("user_model->Rotations.R11D=%f\n", um->Rotations.R11D);
    printf("user_model->Rotations.R12G=%f\n", um->Rotations.R12G);
    printf("user_model->Rotations.R12D=%f\n", um->Rotations.R12D);
    printf("user_model->Rotations.R11C=%f\n", um->Rotations.R11C);
    printf("user_model->Rotations.R12C=%f\n", um->Rotations.R12C);
 
}
 
void mbs_get_user_model_size(int *n_user_model) 
{
    *n_user_model  = 7; 
}
 
void mbs_get_user_model_list(int *user_model_list) 
{
    user_model_list[1]  = 4; 
    user_model_list[2]  = 4; 
    user_model_list[3]  = 2; 
    user_model_list[4]  = 2; 
    user_model_list[5]  = 2; 
    user_model_list[6]  = 2; 
    user_model_list[7]  = 6; 
}

// ============================================================ //
 
