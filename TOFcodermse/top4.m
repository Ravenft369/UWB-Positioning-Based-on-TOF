A0 = [0, 0, 0];
A1 = [5000, 0, 1700];
A2 = [0, 5000, 1700];
A3 = [5000, 5000, 1300];
A4 = [2500, 2500, 0];
A5 = [4000, 1000, 500];
A6 = [4000, 3000, 300];
A7 = [1000, 2000, 1600];
A8 = [2000, 4000, 1400];
anchors = [A0; A1; A2; A3; A4; A5; A6; A7; A8];

num_steps = 200;

t = linspace(0, 2*pi, num_steps);
true_targets = zeros(3, num_steps);
true_targets(1, :) = 1000 * sin(t) + 2500;
true_targets(2, :) = 1000 * cos(t) + 2500;
true_targets(3, :) = 500 * sin(2*t) + 1000;

TOF_estimates = zeros(3, num_steps);
UKF_estimates = zeros(3, num_steps);

x0 = [true_targets(1, 1); true_targets(2, 1); true_targets(3, 1)];

for t = 1:num_steps
    true_target = true_targets(:, t);
    measurement_noise = 0.1; % 噪声水平 (ns)
    noisy_tof_times = DS_TWR(measurement_noise, anchors, true_target);
    TOF_estimates(:, t) = TOF_localization(anchors, noisy_tof_times);  % 使用TOF定位函数
end

UKF_estimates = UKF(anchors, x0, TOF_estimates);

figure;
plot3(true_targets(1, :), true_targets(2, :), true_targets(3, :), 'r', 'DisplayName', '真实位置');
hold on;
plot3(TOF_estimates(1, :), TOF_estimates(2, :), TOF_estimates(3, :), 'b--', 'DisplayName', 'TOF定位结果');
plot3(UKF_estimates(1, :), UKF_estimates(2, :), UKF_estimates(3, :), 'g-.', 'DisplayName', 'UKF滤波结果');

% 绘制基站位置并标记
for i = 1:size(anchors, 1)
    scatter3(anchors(i, 1), anchors(i, 2), anchors(i, 3), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', '基站位置');
end

xlabel('x坐标 (mm)');
ylabel('y坐标 (mm)');
zlabel('z坐标 (mm)');
title('TOF与UKF联合轨迹定位仿真');
legend;
grid on;
axis equal;

TOF_errors = sqrt(sum((TOF_estimates - true_targets).^2, 1));
UKF_errors = sqrt(sum((UKF_estimates - true_targets).^2, 1));

figure;
plot(1:num_steps, TOF_errors, 'b', 'DisplayName', 'TOF定位误差');
hold on;
plot(1:num_steps, UKF_errors, 'g', 'DisplayName', 'UKF滤波误差');
xlabel('步数');
ylabel('定位误差 (mm)');
title('定位误差对比');
legend;
grid on;

TOF_avg_error = mean(TOF_errors);
UKF_avg_error = mean(UKF_errors);

fprintf('TOF定位的平均误差: %.2f mm\n', TOF_avg_error);
fprintf('UKF滤波的平均误差: %.2f mm\n', UKF_avg_error);