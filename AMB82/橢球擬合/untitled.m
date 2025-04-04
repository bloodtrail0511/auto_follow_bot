clear;close all;
% datas = readmatrix('data.csv'); % 原始數據
datas = readmatrix('data_only_esp.csv'); % 原始數據
psize = 3;
params = fit_ellipsoid(datas);
A = params(1); B = params(2); C = params(3);
D = params(4); E = params(5); F = params(6);
G = params(7); H = params(8); I = params(9);
J = params(10);
disp(params);


Q = [A, D, F;
     D, B, E;
     F, E, C];
g_vec = [G; H; I];
if det(Q) < 0
    Q = -Q;
    g_vec = -g_vec;
end
hardIronOffset = -0.5*(Q^-1 * g_vec);
softIronMatrix = sqrtm(Q/(det(Q))^(1/3));

disp("{"+hardIronOffset(1)+","+hardIronOffset(2)+","+hardIronOffset(3)+"}");
disp("{"+softIronMatrix(1,1)+","+softIronMatrix(1,2)+","+softIronMatrix(1,3)+"},");
disp("{"+softIronMatrix(2,1)+","+softIronMatrix(2,2)+","+softIronMatrix(2,3)+"},");
disp("{"+softIronMatrix(3,1)+","+softIronMatrix(3,2)+","+softIronMatrix(3,3)+"}");


% 校正數據
M_calibrated = (datas- transpose(hardIronOffset)) * softIronMatrix;
% writematrix(M_calibrated, "calibrated_datas.csv");
% 分別獲取校正後的 X, Y, Z
X_cal = M_calibrated(:, 1);
Y_cal = M_calibrated(:, 2);
Z_cal = M_calibrated(:, 3);

% figure();
figure('Position', [0, 0, 1100, 900]); % [左, 下, 寬, 高]
scatter3(datas(:,1), datas(:,2), datas(:,3), psize, 'filled'); % 原始數據
hold on;
scatter3(X_cal, Y_cal, Z_cal, psize, 'filled'); % 校正後數據
line([-1.5, 1.5], [0, 0], [0, 0], 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '--'); % X 軸輔助線
line([0, 0], [-1.5, 1.5], [0, 0], 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '--'); % Y 軸輔助線
line([0, 0], [0, 0], [-1.5, 1.5], 'Color', 'k', 'LineWidth', 1.5, 'LineStyle', '--'); % Z 軸輔助線
lgd = legend('Raw Data', 'Calibrated Data');
% legend('Raw Data', 'Calibrated Data', 'Location', 'northeast');
lgd.Position = [0.6 0.8 0.15 0.07]; % 自訂位置 (左, 下, 寬, 高)

title("橢球校正");
% text(0, 0, 2.5, '橢球校正', 'HorizontalAlignment', 'center');
xlabel('X(Gauss)');
ylabel('Y(Gauss)');
zlabel('Z(Gauss)');
xlim([-1.5 1.5]); % 固定 X 軸範圍
ylim([-1.5 1.5]); % 固定 Y 軸範圍
zlim([-1.5 1.5]); % 固定 Z 軸範圍
axis manual;
axis equal;
% axis vis3d; % 固定三維軸的比例
% camproj('perspective'); % 可選，設置透視視角
grid on;
fontsize(25, "pixels");



% % 旋轉並保存為 GIF
% rotationSpeed = 45; % 每秒旋轉角度
% frameRate = 60; % 幀率
% filename = 'calibration.gif'; % GIF 文件名
% totalFrames = frameRate * 360/rotationSpeed; % 旋轉一圈需要的總幀數
% viewAngle = linspace(0, 360, totalFrames); % 生成旋轉角度
% 
% for i = 1:totalFrames
%     view(viewAngle(i), 30); % 更新視角
%     drawnow; % 刷新圖形
%     % 捕捉當前幀
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [imind, cm] = rgb2ind(im, 256);
%     % 將幀寫入 GIF
%     if i == 1
%         imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 1/frameRate);
%     else
%         imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1/frameRate);
%     end
% end










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







function params = fit_ellipsoid(data)
    % 擷取 X, Y, Z 數據
    X = data(:, 1);
    Y = data(:, 2);
    Z = data(:, 3);

    % 構建矩陣 M
    % M = [X.^2, Y.^2, Z.^2, 2*X.*Y, 2*Y.*Z, 2*X.*Z, 2*X, 2*Y, 2*Z, ones(size(X))];
    M = [X.^2, Y.^2, Z.^2, 2*X.*Y, 2*Y.*Z, 2*X.*Z, X, Y, Z, ones(size(X))];

    % 奇異值分解 SVD
    [~, ~, V] = svd(M, 'econ');

    % 取 V 的最後一列 (最小奇異值對應的向量)
    params = V(:, end);
end