close all

F_ar_d_y_normal = extForces_ar_d_normal(:,1);
F_ar_d_z_normal = extForces_ar_d_normal(:,2);
F_ar_g_y_normal = extForces_ar_g_normal(:,1);
F_ar_g_z_normal = extForces_ar_g_normal(:,2);
F_av_d_y_normal = extForces_av_d_normal(:,1);
F_av_d_z_normal = extForces_av_d_normal(:,2);
F_av_g_y_normal = extForces_av_g_normal(:,1);
F_av_g_z_normal = extForces_av_g_normal(:,2);

F_ar_d_y_FR_0_init = extForces_ar_d_FR_0_init(:,1);
F_ar_d_z_FR_0_init = extForces_ar_d_FR_0_init(:,2);
F_ar_g_y_FR_0_init = extForces_ar_g_FR_0_init(:,1);
F_ar_g_z_FR_0_init = extForces_ar_g_FR_0_init(:,2);
F_av_d_y_FR_0_init = extForces_av_d_FR_0_init(:,1);
F_av_d_z_FR_0_init = extForces_av_d_FR_0_init(:,2);
F_av_g_y_FR_0_init = extForces_av_g_FR_0_init(:,1);
F_av_g_z_FR_0_init = extForces_av_g_FR_0_init(:,2);

F_ar_d_y_FR_30_init = extForces_ar_d_FR_30_init(:,1);
F_ar_d_z_FR_30_init = extForces_ar_d_FR_30_init(:,2);
F_ar_g_y_FR_30_init = extForces_ar_g_FR_30_init(:,1);
F_ar_g_z_FR_30_init = extForces_ar_g_FR_30_init(:,2);
F_av_d_y_FR_30_init = extForces_av_d_FR_30_init(:,1);
F_av_d_z_FR_30_init = extForces_av_d_FR_30_init(:,2);
F_av_g_y_FR_30_init = extForces_av_g_FR_30_init(:,1);
F_av_g_z_FR_30_init = extForces_av_g_FR_30_init(:,2);

F_ar_d_y_FR_70_init = extForces_ar_d_FR_70_init(:,1);
F_ar_d_z_FR_70_init = extForces_ar_d_FR_70_init(:,2);
F_ar_g_y_FR_70_init = extForces_ar_g_FR_70_init(:,1);
F_ar_g_z_FR_70_init = extForces_ar_g_FR_70_init(:,2);
F_av_d_y_FR_70_init = extForces_av_d_FR_70_init(:,1);
F_av_d_z_FR_70_init = extForces_av_d_FR_70_init(:,2);
F_av_g_y_FR_70_init = extForces_av_g_FR_70_init(:,1);
F_av_g_z_FR_70_init = extForces_av_g_FR_70_init(:,2);

F_ar_d_y_RR_0_init = extForces_ar_d_RR_0_init(:,1);
F_ar_d_z_RR_0_init = extForces_ar_d_RR_0_init(:,2);
F_ar_g_y_RR_0_init = extForces_ar_g_RR_0_init(:,1);
F_ar_g_z_RR_0_init = extForces_ar_g_RR_0_init(:,2);
F_av_d_y_RR_0_init = extForces_av_d_RR_0_init(:,1);
F_av_d_z_RR_0_init = extForces_av_d_RR_0_init(:,2);
F_av_g_y_RR_0_init = extForces_av_g_RR_0_init(:,1);
F_av_g_z_RR_0_init = extForces_av_g_RR_0_init(:,2);

F_ar_d_y_RR_30_init = extForces_ar_d_RR_30_init(:,1);
F_ar_d_z_RR_30_init = extForces_ar_d_RR_30_init(:,2);
F_ar_g_y_RR_30_init = extForces_ar_g_RR_30_init(:,1);
F_ar_g_z_RR_30_init = extForces_ar_g_RR_30_init(:,2);
F_av_d_y_RR_30_init = extForces_av_d_RR_30_init(:,1);
F_av_d_z_RR_30_init = extForces_av_d_RR_30_init(:,2);
F_av_g_y_RR_30_init = extForces_av_g_RR_30_init(:,1);
F_av_g_z_RR_30_init = extForces_av_g_RR_30_init(:,2);

F_ar_d_y_RR_70_init = extForces_ar_d_RR_70_init(:,1);
F_ar_d_z_RR_70_init = extForces_ar_d_RR_70_init(:,2);
F_ar_g_y_RR_70_init = extForces_ar_g_RR_70_init(:,1);
F_ar_g_z_RR_70_init = extForces_ar_g_RR_70_init(:,2);
F_av_d_y_RR_70_init = extForces_av_d_RR_70_init(:,1);
F_av_d_z_RR_70_init = extForces_av_d_RR_70_init(:,2);
F_av_g_y_RR_70_init = extForces_av_g_RR_70_init(:,1);
F_av_g_z_RR_70_init = extForces_av_g_RR_70_init(:,2);

%% VERTICAL : Avant: G/D - normal/30
figure
hold on
grid on
plot(t,F_av_g_z_normal,'linewidth',2);
plot(t,F_av_d_z_normal,'linewidth',2);
plot(t,F_av_g_z_FR_30_init,'linewidth',2);
plot(t,F_av_d_z_FR_30_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Forces at the contact points as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal - Front left', 'Normal - Front right', 'Front bar: 30% - Front left', ...
    'Front bar: 30% - Front right', 'location', 'best');
title(lgd, 'Vertical front forces');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)

%% VERTICAL : Arrière: G/D - normal/30
figure
hold on
grid on
plot(t,F_ar_g_z_normal,'linewidth',2);
plot(t,F_ar_d_z_normal,'linewidth',2);
plot(t,F_ar_g_z_RR_30_init,'linewidth',2);
plot(t,F_ar_d_z_RR_30_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Forces at the contact points as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal - Rear left', 'Normal - Rear right', 'Rear bar: 30% - Rear left', ...
    'Rear bar: 30% - Rear right', 'location', 'best');
title(lgd, 'Vertical rear forces');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)

%% LATERAL : Avant: G/D - normal/30
figure
hold on
grid on
plot(t,-F_av_g_y_normal,'linewidth',2);
plot(t,-F_av_d_y_normal,'linewidth',2);
plot(t,-F_av_g_y_FR_30_init,'linewidth',2);
plot(t,-F_av_d_y_FR_30_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Forces at the contact points as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal - Front left', 'Normal - Front right', 'Front bar: 30% - Front left', ...
    'Front bar: 30% - Front right', 'location', 'best');
title(lgd, 'Lateral front forces');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)

%% LATERAL : Avant: G/D - normal/30
figure
hold on
grid on
plot(t,-F_ar_g_y_normal,'linewidth',2);
plot(t,-F_ar_d_y_normal,'linewidth',2);
plot(t,-F_ar_g_y_RR_30_init,'linewidth',2);
plot(t,-F_ar_d_y_RR_30_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Forces at the contact points as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal - Rear left', 'Normal - Rear right', 'Rear bar: 30% - Rear left', ...
    'Rear bar: 30% - Rear right', 'location', 'best');
title(lgd, 'Lateral rear forces');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)
