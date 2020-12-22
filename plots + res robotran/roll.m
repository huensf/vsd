close all

%% Roulis pour differentes barres mid
R1_chassis_normal = q_normal(:,6);
R1_chassis_FR_70_mid = q_FR_70_mid(:,6);
R1_chassis_FR_30_mid = q_FR_30_mid(:,6);
R1_chassis_FR_0_mid = q_FR_0_mid(:,6);
R1_chassis_RR_70_mid = q_RR_70_mid(:,6);
R1_chassis_RR_30_mid = q_RR_30_mid(:,6);
R1_chassis_RR_0_mid = q_RR_0_mid(:,6);

figure
hold on
grid on
plot(t,-R1_chassis_normal,'linewidth',2);
plot(t,-R1_chassis_FR_70_mid,'linewidth',2);
plot(t,-R1_chassis_FR_30_mid,'linewidth',2);
plot(t,-R1_chassis_FR_0_mid,'linewidth',2);
plot(t,-R1_chassis_RR_70_mid,'linewidth',2);
plot(t,-R1_chassis_RR_30_mid,'linewidth',2);
plot(t,-R1_chassis_RR_0_mid,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*3.5, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Roll of the car as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('roll  [rad]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Change during the turn');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)

%% Roulis pour differentes barres init
R1_chassis_normal = q_normal(:,6);
R1_chassis_FR_70_init = q_FR_70_init(:,6);
R1_chassis_FR_30_init = q_FR_30_init(:,6);
R1_chassis_FR_0_init = q_FR_0_init(:,6);
R1_chassis_RR_70_init = q_RR_70_init(:,6);
R1_chassis_RR_30_init = q_RR_30_init(:,6);
R1_chassis_RR_0_init = q_RR_0_init(:,6);

figure
hold on
grid on
plot(t,-R1_chassis_normal,'linewidth',2);
plot(t,-R1_chassis_FR_70_init,'linewidth',2);
plot(t,-R1_chassis_FR_30_init,'linewidth',2);
plot(t,-R1_chassis_FR_0_init,'linewidth',2);
plot(t,-R1_chassis_RR_70_init,'linewidth',2);
plot(t,-R1_chassis_RR_30_init,'linewidth',2);
plot(t,-R1_chassis_RR_0_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Roll of the car as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('roll  [rad]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Initial change');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)

%% Comparaison: Front bar 70%
figure
hold on
grid on
plot(t,-R1_chassis_normal,'linewidth',2);
plot(t,-R1_chassis_FR_70_mid,'linewidth',2);
plot(t,-R1_chassis_FR_70_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*3.5, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Roll of the car as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('roll  [rad]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70% at the mid-turn', 'Front bar: initially 70%');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)




