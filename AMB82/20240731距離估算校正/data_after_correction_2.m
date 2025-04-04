% clear all; close all;

data = readtable('data_after_correction_2.csv');
x = data.distance;
ideal = data.distance;
original = data.original;
order_1 = data.order_1;
order_2 = data.order_2;
order_3 = data.order_3;
order_4 = data.order_4;
order_5 = data.order_5;

figure;
hold on;
plot(x, zeros(size(x)), 'DisplayName', '零誤差', 'LineWidth',2);
plot(x, original, 'DisplayName', '校正前誤差', 'LineWidth',2);
plot(x, order_1, 'DisplayName', '一階校正');
plot(x, order_2, 'DisplayName', '二階校正', 'LineWidth',2);
plot(x, order_3, 'DisplayName', '三階校正');
plot(x, order_4, 'DisplayName', '四階校正');
plot(x, order_5, 'DisplayName', '五階校正');
grid on;
box on;
legend('Location','northwest');
xlabel('實際距離(cm)');
ylabel('誤差值(%)');
title('距離估算校正前後之誤差值');