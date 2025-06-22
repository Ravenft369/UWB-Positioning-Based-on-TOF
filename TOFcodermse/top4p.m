% 定义基站位置
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

% 原轨迹
t = linspace(0, 2*pi, num_steps);
true_targets_1 = zeros(3, num_steps);
true_targets_1(1, :) = 1000 * sin(t) + 2500;
true_targets_1(2, :) = 1000 * cos(t) + 2500;
true_targets_1(3, :) = 500 * sin(2*t) + 1000;

% 正方形轨迹（含不连续点）
square_side = 2000;
center_x = 2500;
center_y = 2500;
z_square = 800;
num_points_per_side = num_steps / 4;
x_square = [center_x - square_side/2 * ones(1, num_points_per_side), ...
            linspace(center_x - square_side/2, center_x + square_side/2, num_points_per_side), ...
            center_x + square_side/2 * ones(1, num_points_per_side), ...
            linspace(center_x + square_side/2, center_x - square_side/2, num_points_per_side)];
y_square = [linspace(center_y - square_side/2, center_y + square_side/2, num_points_per_side), ...
            center_y + square_side/2 * ones(1, num_points_per_side), ...
            linspace(center_y + square_side/2, center_y - square_side/2, num_points_per_side), ...
            center_y - square_side/2 * ones(1, num_points_per_side)];
z_square = z_square * ones(1, num_steps);
true_targets_2 = [x_square; y_square; z_square];

% 椭圆螺旋线轨迹
a = 1000; % 椭圆长半轴
b = 800; % 椭圆短半轴
h = 1000; % 螺旋线高度
center_x_ellipse = 2500;
center_y_ellipse = 2500;
t_ellipse = linspace(0, 4*pi, num_steps);
x_ellipse = a * cos(t_ellipse) + center_x_ellipse;
y_ellipse = b * sin(t_ellipse) + center_y_ellipse;
z_ellipse = h * t_ellipse / (4*pi); % 高度随角度线性增加
true_targets_3 = [x_ellipse; y_ellipse; z_ellipse];

% 初始化估计结果
TOF_estimates_1 = zeros(3, num_steps);
UKF_estimates_1 = zeros(3, num_steps);
TOF_estimates_2 = zeros(3, num_steps);
UKF_estimates_2 = zeros(3, num_steps);
TOF_estimates_3 = zeros(3, num_steps);
UKF_estimates_3 = zeros(3, num_steps);

% 初始化三个轨迹的UKF初始状态
x0_1 = [true_targets_1(1, 1); true_targets_1(2, 1); true_targets_1(3, 1)];
x0_2 = [true_targets_2(1, 1); true_targets_2(2, 1); true_targets_2(3, 1)];
x0_3 = [true_targets_3(1, 1); true_targets_3(2, 1); true_targets_3(3, 1)];

% 并行轮询处理三个轨迹
for t = 1:num_steps
    % 处理第一条轨迹
    true_target_1 = true_targets_1(:, t);
    noisy_tof_times_1 = DS_TWR(0.1, anchors, true_target_1);
    TOF_estimates_1(:, t) = TOF_localization(anchors, noisy_tof_times_1);
    
    % 处理第二条轨迹
    true_target_2 = true_targets_2(:, t);
    noisy_tof_times_2 = DS_TWR(0.1, anchors, true_target_2);
    TOF_estimates_2(:, t) = TOF_localization(anchors, noisy_tof_times_2);
    
    % 处理第三条轨迹
    true_target_3 = true_targets_3(:, t);
    noisy_tof_times_3 = DS_TWR(0.1, anchors, true_target_3);
    TOF_estimates_3(:, t) = TOF_localization(anchors, noisy_tof_times_3);
end

% 分别对三个轨迹应用UKF滤波
UKF_estimates_1 = UKF(anchors, x0_1, TOF_estimates_1);
UKF_estimates_2 = UKF(anchors, x0_2, TOF_estimates_2);
UKF_estimates_3 = UKF(anchors, x0_3, TOF_estimates_3);

% 计算RMSE
TOF_rmse_1 = sqrt(sum(sum((TOF_estimates_1 - true_targets_1).^2)) / (3 * num_steps));
UKF_rmse_1 = sqrt(sum(sum((UKF_estimates_1 - true_targets_1).^2)) / (3 * num_steps));
TOF_rmse_2 = sqrt(sum(sum((TOF_estimates_2 - true_targets_2).^2)) / (3 * num_steps));
UKF_rmse_2 = sqrt(sum(sum((UKF_estimates_2 - true_targets_2).^2)) / (3 * num_steps));
TOF_rmse_3 = sqrt(sum(sum((TOF_estimates_3 - true_targets_3).^2)) / (3 * num_steps));
UKF_rmse_3 = sqrt(sum(sum((UKF_estimates_3 - true_targets_3).^2)) / (3 * num_steps));

% 计算点误差（与原代码保持一致）
TOF_errors_1 = sqrt(sum((TOF_estimates_1 - true_targets_1).^2, 1));
UKF_errors_1 = sqrt(sum((UKF_estimates_1 - true_targets_1).^2, 1));
TOF_errors_2 = sqrt(sum((TOF_estimates_2 - true_targets_2).^2, 1));
UKF_errors_2 = sqrt(sum((UKF_estimates_2 - true_targets_2).^2, 1));
TOF_errors_3 = sqrt(sum((TOF_estimates_3 - true_targets_3).^2, 1));
UKF_errors_3 = sqrt(sum((UKF_estimates_3 - true_targets_3).^2, 1));

% 计算平均误差（与原代码保持一致）
TOF_avg_error_1 = mean(TOF_errors_1);
UKF_avg_error_1 = mean(UKF_errors_1);
TOF_avg_error_2 = mean(TOF_errors_2);
UKF_avg_error_2 = mean(UKF_errors_2);
TOF_avg_error_3 = mean(TOF_errors_3);
UKF_avg_error_3 = mean(UKF_errors_3);

% 绘制三维轨迹图
figure;
plot3(true_targets_1(1, :), true_targets_1(2, :), true_targets_1(3, :), 'r', 'DisplayName', '真实位置1');
hold on;
plot3(TOF_estimates_1(1, :), TOF_estimates_1(2, :), TOF_estimates_1(3, :), 'b--', 'DisplayName', 'TOF定位结果1');
plot3(UKF_estimates_1(1, :), UKF_estimates_1(2, :), UKF_estimates_1(3, :), 'g-.', 'DisplayName', 'UKF滤波结果1');

plot3(true_targets_2(1, :), true_targets_2(2, :), true_targets_2(3, :), 'm', 'DisplayName', '真实位置2（正方形）');
plot3(TOF_estimates_2(1, :), TOF_estimates_2(2, :), TOF_estimates_2(3, :), 'c--', 'DisplayName', 'TOF定位结果2');
plot3(UKF_estimates_2(1, :), UKF_estimates_2(2, :), UKF_estimates_2(3, :), 'y-.', 'DisplayName', 'UKF滤波结果2');

plot3(true_targets_3(1, :), true_targets_3(2, :), true_targets_3(3, :), 'k', 'DisplayName', '真实位置3（椭圆螺旋线）');
plot3(TOF_estimates_3(1, :), TOF_estimates_3(2, :), TOF_estimates_3(3, :), 'r--', 'DisplayName', 'TOF定位结果3');
plot3(UKF_estimates_3(1, :), UKF_estimates_3(2, :), UKF_estimates_3(3, :), 'g--', 'DisplayName', 'UKF滤波结果3');

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

% 分别绘制三条轨迹的误差对比图
% 第一条轨迹的误差对比图
figure;
plot(1:num_steps, TOF_errors_1, 'b', 'DisplayName', 'TOF定位误差1');
hold on;
plot(1:num_steps, UKF_errors_1, 'g', 'DisplayName', 'UKF滤波误差1');
xlabel('步数');
ylabel('定位误差 (mm)');
title('第一条轨迹定位误差对比');
legend;
grid on;

% 第二条轨迹的误差对比图
figure;
plot(1:num_steps, TOF_errors_2, 'c', 'DisplayName', 'TOF定位误差2');
hold on;
plot(1:num_steps, UKF_errors_2, 'y', 'DisplayName', 'UKF滤波误差2');
xlabel('步数');
ylabel('定位误差 (mm)');
title('第二条轨迹（正方形）定位误差对比');
legend;
grid on;

% 第三条轨迹的误差对比图
figure;
plot(1:num_steps, TOF_errors_3, 'r', 'DisplayName', 'TOF定位误差3');
hold on;
plot(1:num_steps, UKF_errors_3, 'g--', 'DisplayName', 'UKF滤波误差3');
xlabel('步数');
ylabel('定位误差 (mm)');
title('第三条轨迹（椭圆螺旋线）定位误差对比');
legend;
grid on;

% 输出平均误差和RMSE
fprintf('第一条轨迹 TOF 定位的平均误差: %.2f mm, RMSE: %.2f mm\n', TOF_avg_error_1, TOF_rmse_1);
fprintf('第一条轨迹 UKF 滤波的平均误差: %.2f mm, RMSE: %.2f mm\n', UKF_avg_error_1, UKF_rmse_1);
fprintf('第二条轨迹（正方形） TOF 定位的平均误差: %.2f mm, RMSE: %.2f mm\n', TOF_avg_error_2, TOF_rmse_2);
fprintf('第二条轨迹（正方形） UKF 滤波的平均误差: %.2f mm, RMSE: %.2f mm\n', UKF_avg_error_2, UKF_rmse_2);
fprintf('第三条轨迹（椭圆螺旋线） TOF 定位的平均误差: %.2f mm, RMSE: %.2f mm\n', TOF_avg_error_3, TOF_rmse_3);
fprintf('第三条轨迹（椭圆螺旋线） UKF 滤波的平均误差: %.2f mm, RMSE: %.2f mm\n', UKF_avg_error_3, UKF_rmse_3);