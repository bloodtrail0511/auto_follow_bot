close all;clear;
datas= readmatrix('long_time_angle_error.csv'); % 原始數據
fused = datas(:,1);
gyro = datas(:,2);
mag = datas(:,3);
t = linspace(0, 30, 89998);
% t = linspace(0, 1800, 89998);

figure;
plot(t, mag, LineWidth=1);hold on;
plot(t, gyro, LineWidth=2);hold on;
plot(t, fused, LineWidth=1);hold on;
legend("magnetometer", "gyro", "fused", "Location","southwest");
% legend("magnetometer", "fused", "Location","southwest");
% legend("gyro", "fused", "Location","southwest");
% xlabel("time(min)");
xlabel("time(s)");
ylabel("angle(xy-plane)");
% ylim([-1,1]);
xlim([0,15]);
title("長時間下陀螺儀的角度飄移");
hold off;
grid on;
fontsize(15, "pixels");
% saveas(gcf, "long_time_angle_error.png");
% saveas(gcf, "long_time_angle_error_nogyro.png");
% saveas(gcf, "long_time_angle_error_nomag.png");
saveas(gcf, "long_time_angle_error_15min.png");