
clear all; close all;
% 加载数据
data = readtable('distance_predict.xlsx');

% 提取数据
actual_distance = data.Var1(2:6);
measured_before = data.Var9(2:6);
error_before = data.Var11(2:6);

% 校正后的量测数据和误差
measured_after = data.Var9(13:17);
error_after = data.Var11(13:17);

% 绘制图表1：实际距离 vs. 量测距离
figure;
% subplot(2, 1, 1);
hold on;
plot(actual_distance, actual_distance, 'k--', 'DisplayName', '實際距離', 'LineWidth',2);
plot(actual_distance, measured_before, 'r-', 'DisplayName', '校正前', 'LineWidth',2);
plot(actual_distance, measured_after, 'b-', 'DisplayName', '校正後', 'LineWidth',2);
xlabel('實際距離(cm)');
ylabel('量測距離(cm)');
title('距離估算校正前後之結果');
grid on;
legend('Location','southeast');
% hold off;

% 绘制图表2：实际距离 vs. 误差
figure;
% subplot(2, 1, 2);
hold on;
plot(actual_distance, zeros(size(actual_distance)), 'k--', 'DisplayName', '零誤差', 'LineWidth',2);
plot(actual_distance, error_before, 'r-', 'DisplayName', '校正前誤差', 'LineWidth',2);
plot(actual_distance, error_after, 'b-', 'DisplayName', '校正後誤差', 'LineWidth',2);
ylim([-10, 30]);
xlabel('實際距離(cm)');
ylabel('誤差值(%)');
title('距離估算校正前後之誤差值');
grid on;
legend('Location','southeast');
hold off;
