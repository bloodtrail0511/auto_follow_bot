close all;clear;
data1 = readmatrix('roll_xyangle.csv');
data2 = readmatrix('pitch_xyangle.csv');
data3 = readmatrix('roll_xyangle2.csv');
data4 = readmatrix('pitch_xyangle2.csv');

%%%%%%%%%%%最大偏移量<10度
dot_size = 15;

roll = data1(:,1)*180/pi;
xy_angle1 = data1(:,2);

pitch = data2(:,1)*180/pi;
xy_angle2 = data2(:,2);

roll2 = data3(:,1)*180/pi;
xy_angle3 = data3(:,2);

pitch2 = data4(:,1)*180/pi;
xy_angle4 = data4(:,2);

fig = figure();
set(fig,'Position',[100 100 1000 500]);
scatter(roll, xy_angle1, dot_size, "filled");
hold on;
% scatter(pitch, xy_angle2, dot_size, "filled");
hold on;
scatter(roll2, xy_angle3, dot_size, "filled");
hold on;
% scatter(pitch2, xy_angle4, dot_size, "filled");
% legend("補償前(roll)", "補償前(pitch)", "補償後(roll)", "補償後(pitch)", 'Location','northwest');
% legend("旋轉roll", "旋轉pitch", 'Location','northwest');
legend("補償前", "補償後", 'Location','northwest');
% xlabel("roll\\pitch旋轉角度(deg)");
xlabel("roll旋轉角度(deg)");
% xlabel("pitch旋轉角度(deg)");
ylabel("磁力計估算之角度(deg)");
xlim([-5, 85]);
% ylim([-20, 50]);
ylim([-20,200]);
% title("未補償的磁力計角度偏移");
title("roll旋轉補償");
% title("pitch旋轉補償");
% title("磁力計角度偏移");
grid on;
fontsize(20, "pixels");
% saveas(fig, "未校正的磁力計角度偏移.png");
saveas(fig, "roll旋轉補償.png");
% saveas(fig, "pitch旋轉補償.png");
% saveas(fig, "磁力計角度偏移.png");