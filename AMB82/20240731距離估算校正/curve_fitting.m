close all; clear all;
% 定義多項式函數
order_1 = @(x) -14.04 + 1.18 * x;
order_2 = @(x) 0.1054 + 0.9397579 * x + 8.86754e-4 * (x.^2);
order_3 = @(x) -15.5162 + 1.367 * x - 2.5996e-3 * (x.^2) + 8.6963e-6 * (x.^3);
order_4 = @(x) 3.89298 + 0.64038 * x + 6.6809e-3 * (x.^2) - 3.994e-5 * (x.^3) + 8.97e-8 * (x.^4);
order_5 = @(x) 467.69487 - 21.4 * x + 0.39583 * (x.^2) - 3.25866e-3 * (x.^3) + 1.26646e-5 * (x.^4) - 1.871e-8 * (x.^5);

% 讀取資料
data = readtable('./data.csv');
raw_measure = data.measure;
raw_distance = data.distance;

% 生成理想距離範圍
ideal = 50:250;

% 計算各階多項式的結果
res1 = arrayfun(order_1, ideal);
res2 = arrayfun(order_2, ideal);
res3 = arrayfun(order_3, ideal);
res4 = arrayfun(order_4, ideal);
res5 = arrayfun(order_5, ideal);

% 畫圖
figure;
hold on;
grid on;

% 繪製原始數據點
scatter(raw_measure, raw_distance, 'DisplayName', 'Original Data Point', 'MarkerFaceColor', '#1f77b4');
% plot(raw_measure, raw_distance, 'o', 'DisplayName', 'Original Data Point', 'MarkerSize', 5);

% 繪製多項式擬合曲線
plot(ideal, res1, 'DisplayName', '1st Order', LineWidth=1.5);
plot(ideal, res2, 'DisplayName', '2nd Order', LineWidth=1.5);
plot(ideal, res3, 'DisplayName', '3rd Order', LineWidth=1.5);
plot(ideal, res4, 'DisplayName', '4th Order', LineWidth=1.5);
plot(ideal, res5, 'DisplayName', '5th Order', 'Color', '#8c564b', LineWidth=1.5);

xlim([30, 270]);

% 添加標籤和圖例
xlabel('Calculated Distance', 'FontSize', 14);
ylabel('Ideal Distance', 'FontSize', 14);
legend(Location="northwest", FontSize=12);
title('線性回歸擬合結果');
hold off;
