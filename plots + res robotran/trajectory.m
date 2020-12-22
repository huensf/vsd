close all

%% Consigne rack
T2_rack = q_normal(:,29);

figure
grid on
plot(t,T2_rack,'linewidth',2);
title('Displacement of the rack as a function of time','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('lateral displacement  [m]', 'fontsize', 20);
xlim([0 10]);
ylim([0 0.012]);
set(gca,'FontSize',15)

% % Position mid
% T1_chassis_normal = q_normal(:,2);
% T2_chassis_normal = q_normal(:,3);
% T1_chassis_FR_70_mid = q_FR_70_mid(:,2);
% T2_chassis_FR_70_mid = q_FR_70_mid(:,3);
% T1_chassis_FR_30_mid = q_FR_30_mid(:,2);
% T2_chassis_FR_30_mid = q_FR_30_mid(:,3);
% T1_chassis_FR_0_mid = q_FR_0_mid(:,2);
% T2_chassis_FR_0_mid = q_FR_0_mid(:,3);
% T1_chassis_RR_70_mid = q_RR_70_mid(:,2);
% T2_chassis_RR_70_mid = q_RR_70_mid(:,3);
% T1_chassis_RR_30_mid = q_RR_30_mid(:,2);
% T2_chassis_RR_30_mid = q_RR_30_mid(:,3);
% T1_chassis_RR_0_mid = q_RR_0_mid(:,2);
% T2_chassis_RR_0_mid = q_RR_0_mid(:,3);
% 
% Normal
% figure
% hold on
% grid on
% plot(-T2_chassis_normal,T1_chassis_normal,'linewidth',2);
% plot(-T2_chassis_FR_70_mid,T1_chassis_FR_70_mid,'linewidth',2);
% plot(-T2_chassis_FR_30_mid,T1_chassis_FR_30_mid,'linewidth',2);
% plot(-T2_chassis_FR_0_mid,T1_chassis_FR_0_mid,'linewidth',2);
% plot(-T2_chassis_RR_70_mid,T1_chassis_RR_70_mid,'linewidth',2);
% plot(-T2_chassis_RR_30_mid,T1_chassis_RR_30_mid,'linewidth',2);
% plot(-T2_chassis_RR_0_mid,T1_chassis_RR_0_mid,'linewidth',2);
% title('Trajectory of the car for different anti-roll bars configurations','fontsize',20);
% xlabel('y-coordinate  [m]', 'fontsize', 20);
% ylabel('x-coordinate  [m]', 'fontsize', 20);
% lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
%     'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
% title(lgd, 'Change during the turn');
% lgd.FontSize = 20;
% xlim([-1 71]);
% ylim([0 45]);
% set(gca,'FontSize',15)
% 
% Zoom
% figure
% hold on
% grid on
% plot(-T2_chassis_normal,T1_chassis_normal,'linewidth',2);
% plot(-T2_chassis_FR_70_mid,T1_chassis_FR_70_mid,'linewidth',2);
% plot(-T2_chassis_FR_30_mid,T1_chassis_FR_30_mid,'linewidth',2);
% plot(-T2_chassis_FR_0_mid,T1_chassis_FR_0_mid,'linewidth',2);
% plot(-T2_chassis_RR_70_mid,T1_chassis_RR_70_mid,'linewidth',2);
% plot(-T2_chassis_RR_30_mid,T1_chassis_RR_30_mid,'linewidth',2);
% plot(-T2_chassis_RR_0_mid,T1_chassis_RR_0_mid,'linewidth',2);
% title('Trajectory of the car for different anti-roll bars configurations (zoom)','fontsize',20);
% xlabel('y-coordinate  [m]', 'fontsize', 20);
% ylabel('x-coordinate  [m]', 'fontsize', 20);
% lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
%     'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
% title(lgd, 'Change during the turn');
% lgd.FontSize = 20;
% xlim([40 75]);
% ylim([35 45]);
% set(gca,'FontSize',15)

%% Position init
T1_chassis_normal = q_normal(:,2);
T2_chassis_normal = q_normal(:,3);
T1_chassis_FR_70_init = q_FR_70_init(:,2);
T2_chassis_FR_70_init = q_FR_70_init(:,3);
T1_chassis_FR_30_init = q_FR_30_init(:,2);
T2_chassis_FR_30_init = q_FR_30_init(:,3);
T1_chassis_FR_0_init = q_FR_0_init(:,2);
T2_chassis_FR_0_init = q_FR_0_init(:,3);
T1_chassis_RR_70_init = q_RR_70_init(:,2);
T2_chassis_RR_70_init = q_RR_70_init(:,3);
T1_chassis_RR_30_init = q_RR_30_init(:,2);
T2_chassis_RR_30_init = q_RR_30_init(:,3);
T1_chassis_RR_0_init = q_RR_0_init(:,2);
T2_chassis_RR_0_init = q_RR_0_init(:,3);

% Normal
figure
hold on
grid on
plot(-T2_chassis_normal,T1_chassis_normal,'linewidth',2);
plot(-T2_chassis_FR_70_init,T1_chassis_FR_70_init,'linewidth',2);
plot(-T2_chassis_FR_30_init,T1_chassis_FR_30_init,'linewidth',2);
plot(-T2_chassis_FR_0_init,T1_chassis_FR_0_init,'linewidth',2);
plot(-T2_chassis_RR_70_init,T1_chassis_RR_70_init,'linewidth',2);
plot(-T2_chassis_RR_30_init,T1_chassis_RR_30_init,'linewidth',2);
plot(-T2_chassis_RR_0_init,T1_chassis_RR_0_init,'linewidth',2);
title('Trajectory of the car for different anti-roll bars configurations','fontsize',20);
xlabel('y-coordinate  [m]', 'fontsize', 20);
ylabel('x-coordinate  [m]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Initial change');
lgd.FontSize = 20;
xlim([-1 71]);
ylim([0 45]);
set(gca,'FontSize',15)

% Zoom
figure
hold on
grid on
plot(-T2_chassis_normal,T1_chassis_normal,'linewidth',2);
plot(-T2_chassis_FR_70_init,T1_chassis_FR_70_init,'linewidth',2);
plot(-T2_chassis_FR_30_init,T1_chassis_FR_30_init,'linewidth',2);
plot(-T2_chassis_FR_0_init,T1_chassis_FR_0_init,'linewidth',2);
plot(-T2_chassis_RR_70_init,T1_chassis_RR_70_init,'linewidth',2);
plot(-T2_chassis_RR_30_init,T1_chassis_RR_30_init,'linewidth',2);
plot(-T2_chassis_RR_0_init,T1_chassis_RR_0_init,'linewidth',2);
title('Trajectory of the car for different anti-roll bars configurations (zoom)','fontsize',20);
xlabel('y-coordinate  [m]', 'fontsize', 20);
ylabel('x-coordinate  [m]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Initial change');
lgd.FontSize = 20;
xlim([40 75]);
ylim([35 45]);
set(gca,'FontSize',15)

