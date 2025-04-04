close all;clear;clc;
% datas = readmatrix('data.csv'); % 原始數據
datas = readmatrix('data2.csv'); % 原始數據
% datas = readmatrix('data_only_esp.csv'); % 原始數據


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 使用 magcal 進行校正
[softIronMatrix, hardIronOffset] = magcal(datas);
disp("{"+hardIronOffset(1)+","+hardIronOffset(2)+","+hardIronOffset(3)+"}");
disp("{"+softIronMatrix(1,1)+","+softIronMatrix(1,2)+","+softIronMatrix(1,3)+"},");
disp("{"+softIronMatrix(2,1)+","+softIronMatrix(2,2)+","+softIronMatrix(2,3)+"},");
disp("{"+softIronMatrix(3,1)+","+softIronMatrix(3,2)+","+softIronMatrix(3,3)+"}");

% hardIronOffset = [-0.067592,1.217,0.20732];
% softIronMatrix = [1.0013,0.1359,-0.0084488;
%     0.1359,1.0507,0.0018683;
%     -0.0084488,0.0018683,0.96752];
% 校正數據
M_calibrated = (datas- hardIronOffset) * softIronMatrix;
% writematrix(M_calibrated, "calibrated_datas.csv");
% 分別獲取校正後的 X, Y, Z
X_cal = M_calibrated(:, 1);
Y_cal = M_calibrated(:, 2);
Z_cal = M_calibrated(:, 3);

figure;
scatter3(datas(:,1), datas(:,2), datas(:,3), 3, 'filled'); % 原始數據
hold on;
scatter3(X_cal, Y_cal, Z_cal, 3, 'filled'); % 校正後數據
line([-1.5, 1.5], [0, 0], [0, 0], 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '--'); % X 軸輔助線
line([0, 0], [-1.5, 1.5], [0, 0], 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '--'); % Y 軸輔助線
line([0, 0], [0, 0], [-1.5, 1.5], 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '--'); % Z 軸輔助線
legend('Raw Data', 'Calibrated Data');
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-2 2]); % 固定 X 軸範圍
ylim([-2 2]); % 固定 Y 軸範圍
zlim([-2 2]); % 固定 Z 軸範圍
axis manual;
axis equal;
grid on;



% % 固定速率旋轉
% % 設定旋轉速度 (度/秒)
% rotationSpeed = 60; % 每秒旋轉角度
% frameRate = 30; % 更新次數 (frames per second)
% 
% % 啟動旋轉
% while ishandle(gca) % 檢查窗口是否還存在
%     for az = 0:rotationSpeed/frameRate:360
%         view(az, 30); % 改變視角，30 是固定的俯仰角
%         pause(1/frameRate); % 控制更新速度
%     end
% end
