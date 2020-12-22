close all

%% Force volant --> crémaillère mid
Force_T2_rack_normal = Qc_normal(:,29);
Force_T2_rack_FR_70_mid = Qc_FR_70_mid(:,29);
Force_T2_rack_FR_30_mid = Qc_FR_30_mid(:,29);
Force_T2_rack_FR_0_mid = Qc_FR_0_mid(:,29);
Force_T2_rack_RR_70_mid = Qc_RR_70_mid(:,29);
Force_T2_rack_RR_30_mid = Qc_RR_30_mid(:,29);
Force_T2_rack_RR_0_mid = Qc_RR_0_mid(:,29);

% Normal
figure
hold on
grid on
plot(t,Force_T2_rack_normal,'linewidth',2);
plot(t,Force_T2_rack_FR_70_mid,'linewidth',2);
plot(t,Force_T2_rack_FR_30_mid,'linewidth',2);
plot(t,Force_T2_rack_FR_0_mid,'linewidth',2);
plot(t,Force_T2_rack_RR_70_mid,'linewidth',2);
plot(t,Force_T2_rack_RR_30_mid,'linewidth',2);
plot(t,Force_T2_rack_RR_0_mid,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*3.5, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Force in the rack as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Change during the turn');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)

% Zoom
figure
hold on
grid on
plot(t,Force_T2_rack_normal,'linewidth',2);
plot(t,Force_T2_rack_FR_70_mid,'linewidth',2);
plot(t,Force_T2_rack_FR_30_mid,'linewidth',2);
plot(t,Force_T2_rack_FR_0_mid,'linewidth',2);
plot(t,Force_T2_rack_RR_70_mid,'linewidth',2);
plot(t,Force_T2_rack_RR_30_mid,'linewidth',2);
plot(t,Force_T2_rack_RR_0_mid,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*3.5, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Force in the rack as a function of time for different anti-roll bars configurations (zoom)','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Change during the turn');
lgd.FontSize = 20;
xlim([0.95 6]);
ylim([900 1300]);
set(gca,'FontSize',15)

%% Force volant --> crémaillère init
Force_T2_rack_normal = Qc_normal(:,29);
Force_T2_rack_FR_70_init = Qc_FR_70_init(:,29);
Force_T2_rack_FR_30_init = Qc_FR_30_init(:,29);
Force_T2_rack_FR_0_init = Qc_FR_0_init(:,29);
Force_T2_rack_RR_70_init = Qc_RR_70_init(:,29);
Force_T2_rack_RR_30_init = Qc_RR_30_init(:,29);
Force_T2_rack_RR_0_init = Qc_RR_0_init(:,29);

% Normal
figure
hold on
grid on
plot(t,Force_T2_rack_normal,'linewidth',2);
plot(t,Force_T2_rack_FR_70_init,'linewidth',2);
plot(t,Force_T2_rack_FR_30_init,'linewidth',2);
plot(t,Force_T2_rack_FR_0_init,'linewidth',2);
plot(t,Force_T2_rack_RR_70_init,'linewidth',2);
plot(t,Force_T2_rack_RR_30_init,'linewidth',2);
plot(t,Force_T2_rack_RR_0_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Force in the rack as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Initial change');
lgd.FontSize = 20;
xlim([0 10]);
set(gca,'FontSize',15)

% Zoom
figure
hold on
grid on
plot(t,Force_T2_rack_normal,'linewidth',2);
plot(t,Force_T2_rack_FR_70_init,'linewidth',2);
plot(t,Force_T2_rack_FR_30_init,'linewidth',2);
plot(t,Force_T2_rack_FR_0_init,'linewidth',2);
plot(t,Force_T2_rack_RR_70_init,'linewidth',2);
plot(t,Force_T2_rack_RR_30_init,'linewidth',2);
plot(t,Force_T2_rack_RR_0_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Force in the rack as a function of time for different anti-roll bars configurations (zoom)','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70%', 'Front bar: 30%', 'Front bar: 0%', ...
    'Rear bar: 70%', 'Rear bar: 30%', 'Rear bar: 0%', 'location', 'best');
title(lgd, 'Initial change');
lgd.FontSize = 20;
xlim([0.95 6]);
ylim([900 1300]);
set(gca,'FontSize',15)

%% Comparaison: Front bar 70%
figure
hold on
grid on
plot(t,Force_T2_rack_normal,'linewidth',2);
plot(t,Force_T2_rack_FR_70_mid,'linewidth',2);
plot(t,Force_T2_rack_FR_70_init,'linewidth',2);
plot([1 1]*0.9, ylim, '--k','linewidth',2);
plot([1 1]*3.5, ylim, '--k','linewidth',2);
plot([1 1]*6.15, ylim, '--k','linewidth',2);
title('Force in the rack as a function of time for different anti-roll bars configurations','fontsize',20);
xlabel('time  [s]', 'fontsize', 20);
ylabel('force  [N]', 'fontsize', 20);
lgd = legend('Normal', 'Front bar: 70% at the mid-turn', 'Front bar: initially 70%');
lgd.FontSize = 20;
xlim([0.95 6]);
ylim([900 1300]);
set(gca,'FontSize',15)
