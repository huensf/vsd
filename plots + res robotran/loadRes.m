clear all
close all

%% Normal 100%
q_normal = load('dirdyn_q_normal.res');
qd_normal = load('dirdyn_qd_normal.res');
qdd_normal = load('dirdyn_qdd_normal.res');
Qc_normal = load('dirdyn_Qc_normal.res');
Qq_normal = load('dirdyn_Qq_normal.res');
t = q_normal(:,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FrontRoll 0% mid
q_FR_0_mid = load('dirdyn_q_FrontRoll_0.res');
qd_FR_0_mid = load('dirdyn_qd_FrontRoll_0.res');
qdd_FR_0_mid = load('dirdyn_qdd_FrontRoll_0.res');
Qc_FR_0_mid = load('dirdyn_Qc_FrontRoll_0.res');
Qq_FR_0_mid = load('dirdyn_Qq_FrontRoll_0.res');

%% FrontRoll 30% mid
q_FR_30_mid = load('dirdyn_q_FrontRoll_30.res');
qd_FR_30_mid = load('dirdyn_qd_FrontRoll_30.res');
qdd_FR_30_mid = load('dirdyn_qdd_FrontRoll_30.res');
Qc_FR_30_mid = load('dirdyn_Qc_FrontRoll_30.res');
Qq_FR_30_mid = load('dirdyn_Qq_FrontRoll_30.res');

%% FrontRoll 70% mid
q_FR_70_mid = load('dirdyn_q_FrontRoll_70.res');
qd_FR_70_mid = load('dirdyn_qd_FrontRoll_70.res');
qdd_FR_70_mid = load('dirdyn_qdd_FrontRoll_70.res');
Qc_FR_70_mid = load('dirdyn_Qc_FrontRoll_70.res');
Qq_FR_70_mid = load('dirdyn_Qq_FrontRoll_70.res');

%% RearRoll 0% mid
q_RR_0_mid = load('dirdyn_q_RearRoll_0.res');
qd_RR_0_mid = load('dirdyn_qd_RearRoll_0.res');
qdd_RR_0_mid = load('dirdyn_qdd_RearRoll_0.res');
Qc_RR_0_mid = load('dirdyn_Qc_RearRoll_0.res');
Qq_RR_0_mid = load('dirdyn_Qq_RearRoll_0.res');

%% RearRoll 30% mid
q_RR_30_mid = load('dirdyn_q_RearRoll_30.res');
qd_RR_30_mid = load('dirdyn_qd_RearRoll_30.res');
qdd_RR_30_mid = load('dirdyn_qdd_RearRoll_30.res');
Qc_RR_30_mid = load('dirdyn_Qc_RearRoll_30.res');
Qq_RR_30_mid = load('dirdyn_Qq_RearRoll_30.res');

%% RearRoll 70% mid
q_RR_70_mid = load('dirdyn_q_RearRoll_70.res');
qd_RR_70_mid = load('dirdyn_qd_RearRoll_70.res');
qdd_RR_70_mid = load('dirdyn_qdd_RearRoll_70.res');
Qc_RR_70_mid = load('dirdyn_Qc_RearRoll_70.res');
Qq_RR_70_mid = load('dirdyn_Qq_RearRoll_70.res');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FrontRoll 0% init
q_FR_0_init = load('dirdyn_q_FrontRoll_0_init.res');
qd_FR_0_init = load('dirdyn_qd_FrontRoll_0_init.res');
qdd_FR_0_init = load('dirdyn_qdd_FrontRoll_0_init.res');
Qc_FR_0_init = load('dirdyn_Qc_FrontRoll_0_init.res');
Qq_FR_0_init = load('dirdyn_Qq_FrontRoll_0_init.res');

%% FrontRoll 30% init
q_FR_30_init = load('dirdyn_q_FrontRoll_30_init.res');
qd_FR_30_init = load('dirdyn_qd_FrontRoll_30_init.res');
qdd_FR_30_init = load('dirdyn_qdd_FrontRoll_30_init.res');
Qc_FR_30_init = load('dirdyn_Qc_FrontRoll_30_init.res');
Qq_FR_30_init = load('dirdyn_Qq_FrontRoll_30_init.res');

%% FrontRoll 70% init
q_FR_70_init = load('dirdyn_q_FrontRoll_70_init.res');
qd_FR_70_init = load('dirdyn_qd_FrontRoll_70_init.res');
qdd_FR_70_init = load('dirdyn_qdd_FrontRoll_70_init.res');
Qc_FR_70_init = load('dirdyn_Qc_FrontRoll_70_init.res');
Qq_FR_70_init = load('dirdyn_Qq_FrontRoll_70_init.res');

%% RearRoll 0% init
q_RR_0_init = load('dirdyn_q_RearRoll_0_init.res');
qd_RR_0_init = load('dirdyn_qd_RearRoll_0_init.res');
qdd_RR_0_init = load('dirdyn_qdd_RearRoll_0_init.res');
Qc_RR_0_init = load('dirdyn_Qc_RearRoll_0_init.res');
Qq_RR_0_init = load('dirdyn_Qq_RearRoll_0_init.res');

%% RearRoll 30% init
q_RR_30_init = load('dirdyn_q_RearRoll_30_init.res');
qd_RR_30_init = load('dirdyn_qd_RearRoll_30_init.res');
qdd_RR_30_init = load('dirdyn_qdd_RearRoll_30_init.res');
Qc_RR_30_init = load('dirdyn_Qc_RearRoll_30_init.res');
Qq_RR_30_init = load('dirdyn_Qq_RearRoll_30_init.res');

%% RearRoll 70% init
q_RR_70_init = load('dirdyn_q_RearRoll_70_init.res');
qd_RR_70_init = load('dirdyn_qd_RearRoll_70_init.res');
qdd_RR_70_init = load('dirdyn_qdd_RearRoll_70_init.res');
Qc_RR_70_init = load('dirdyn_Qc_RearRoll_70_init.res');
Qq_RR_70_init = load('dirdyn_Qq_RearRoll_70_init.res');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ExtForces normal
extForces_ar_d_normal = load('dirdyn_extForces_ar_d_normal.res');
extForces_ar_g_normal = load('dirdyn_extForces_ar_g_normal.res');
extForces_av_d_normal = load('dirdyn_extForces_av_d_normal.res');
extForces_av_g_normal = load('dirdyn_extForces_av_g_normal.res');
extForces_ar_d_normal(10002,:) = extForces_ar_d_normal(10001,:);
extForces_ar_g_normal(10002,:) = extForces_ar_g_normal(10001,:);
extForces_av_d_normal(10002,:) = extForces_av_d_normal(10001,:);
extForces_av_g_normal(10002,:) = extForces_av_g_normal(10001,:);

%% ExtForces FR 0
extForces_ar_d_FR_0_init = load('dirdyn_extForces_ar_d_FrontRoll_0_init.res');
extForces_ar_g_FR_0_init = load('dirdyn_extForces_ar_g_FrontRoll_0_init.res');
extForces_av_d_FR_0_init = load('dirdyn_extForces_av_d_FrontRoll_0_init.res');
extForces_av_g_FR_0_init = load('dirdyn_extForces_av_g_FrontRoll_0_init.res');
extForces_ar_d_FR_0_init(10002,:) = extForces_ar_d_FR_0_init(10001,:);
extForces_ar_g_FR_0_init(10002,:) = extForces_ar_g_FR_0_init(10001,:);
extForces_av_d_FR_0_init(10002,:) = extForces_av_d_FR_0_init(10001,:);
extForces_av_g_FR_0_init(10002,:) = extForces_av_g_FR_0_init(10001,:);

%% ExtForces FR 30
extForces_ar_d_FR_30_init = load('dirdyn_extForces_ar_d_FrontRoll_30_init.res');
extForces_ar_g_FR_30_init = load('dirdyn_extForces_ar_g_FrontRoll_30_init.res');
extForces_av_d_FR_30_init = load('dirdyn_extForces_av_d_FrontRoll_30_init.res');
extForces_av_g_FR_30_init = load('dirdyn_extForces_av_g_FrontRoll_30_init.res');
extForces_ar_d_FR_30_init(10002,:) = extForces_ar_d_FR_30_init(10001,:);
extForces_ar_g_FR_30_init(10002,:) = extForces_ar_g_FR_30_init(10001,:);
extForces_av_d_FR_30_init(10002,:) = extForces_av_d_FR_30_init(10001,:);
extForces_av_g_FR_30_init(10002,:) = extForces_av_g_FR_30_init(10001,:);

%% ExtForces FR 70
extForces_ar_d_FR_70_init = load('dirdyn_extForces_ar_d_FrontRoll_70_init.res');
extForces_ar_g_FR_70_init = load('dirdyn_extForces_ar_g_FrontRoll_70_init.res');
extForces_av_d_FR_70_init = load('dirdyn_extForces_av_d_FrontRoll_70_init.res');
extForces_av_g_FR_70_init = load('dirdyn_extForces_av_g_FrontRoll_70_init.res');
extForces_ar_d_FR_70_init(10002,:) = extForces_ar_d_FR_70_init(10001,:);
extForces_ar_g_FR_70_init(10002,:) = extForces_ar_g_FR_70_init(10001,:);
extForces_av_d_FR_70_init(10002,:) = extForces_av_d_FR_70_init(10001,:);
extForces_av_g_FR_70_init(10002,:) = extForces_av_g_FR_70_init(10001,:);

%% ExtForces RR 0
extForces_ar_d_RR_0_init = load('dirdyn_extForces_ar_d_RearRoll_0_init.res');
extForces_ar_g_RR_0_init = load('dirdyn_extForces_ar_g_RearRoll_0_init.res');
extForces_av_d_RR_0_init = load('dirdyn_extForces_av_d_RearRoll_0_init.res');
extForces_av_g_RR_0_init = load('dirdyn_extForces_av_g_RearRoll_0_init.res');
extForces_ar_d_RR_0_init(10002,:) = extForces_ar_d_RR_0_init(10001,:);
extForces_ar_g_RR_0_init(10002,:) = extForces_ar_g_RR_0_init(10001,:);
extForces_av_d_RR_0_init(10002,:) = extForces_av_d_RR_0_init(10001,:);
extForces_av_g_RR_0_init(10002,:) = extForces_av_g_RR_0_init(10001,:);

%% ExtForces RR 30
extForces_ar_d_RR_30_init = load('dirdyn_extForces_ar_d_RearRoll_30_init.res');
extForces_ar_g_RR_30_init = load('dirdyn_extForces_ar_g_RearRoll_30_init.res');
extForces_av_d_RR_30_init = load('dirdyn_extForces_av_d_RearRoll_30_init.res');
extForces_av_g_RR_30_init = load('dirdyn_extForces_av_g_RearRoll_30_init.res');
extForces_ar_d_RR_30_init(10002,:) = extForces_ar_d_RR_30_init(10001,:);
extForces_ar_g_RR_30_init(10002,:) = extForces_ar_g_RR_30_init(10001,:);
extForces_av_d_RR_30_init(10002,:) = extForces_av_d_RR_30_init(10001,:);
extForces_av_g_RR_30_init(10002,:) = extForces_av_g_RR_30_init(10001,:);

%% ExtForces RR 70
extForces_ar_d_RR_70_init = load('dirdyn_extForces_ar_d_RearRoll_70_init.res');
extForces_ar_g_RR_70_init = load('dirdyn_extForces_ar_g_RearRoll_70_init.res');
extForces_av_d_RR_70_init = load('dirdyn_extForces_av_d_RearRoll_70_init.res');
extForces_av_g_RR_70_init = load('dirdyn_extForces_av_g_RearRoll_70_init.res');
extForces_ar_d_RR_70_init(10002,:) = extForces_ar_d_RR_70_init(10001,:);
extForces_ar_g_RR_70_init(10002,:) = extForces_ar_g_RR_70_init(10001,:);
extForces_av_d_RR_70_init(10002,:) = extForces_av_d_RR_70_init(10001,:);
extForces_av_g_RR_70_init(10002,:) = extForces_av_g_RR_70_init(10001,:);

%% joints
% #define T1_chassis_id 1 -------------------------------
% #define T2_chassis_id 2 -------------------------------
% #define T3_chassis_id 3
% #define R3_chassis_id 4
% #define R1_chassis_id 5 -------------------------------
% #define R2_chassis_id 6
% #define T1_weight_id 7
% #define Joint_16_id 8
% #define Joint_4_id 9
% #define Joint_17_id 10
% #define Joint_18_id 11
% #define R2_wheel_ft_rt_id 12
% #define Joint_6_id 13
% #define Joint_5_id 14
% #define Joint_8_id 15
% #define Joint_9_id 16
% #define R2_wheel_ft_lt_id 17
% #define R1_bras_inf_av_g_1_id 18
% #define R1_bras_inf_av_d_1_id 19
% #define R1_bras_inf_ar_d_1_id 20
% #define R1_bras_inf_ar_g_1_id 21
% #define Joint_11_id 22
% #define Joint_21_id 23
% #define Joint_22_id 24
% #define R2_def_bar_ft_id 25
% #define Joint_24_id 26
% #define Joint_25_id 27
% #define T2_rack_id 28 -------------------------------
% #define Joint_0_id 29
% #define Joint_1_id 30
% #define Joint_2_id 31
% #define Joint_3_id 32
% #define Joint_31_id 33
% #define Joint_38_id 34
% #define R2_wheel_rr_lt_id 35
% #define Joint_27_id 36
% #define Joint_28_id 37
% #define R2_wheel_rr_rt_id 38
% #define Joint_32_id 39
% #define R2_def_bar_rr_id 40
% #define Joint_34_id 41
% #define Joint_35_id 42
% #define Joint_40_id 43
% #define Joint_41_id 44
