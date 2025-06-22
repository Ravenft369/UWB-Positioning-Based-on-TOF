% 定义锚点坐标 (单位：mm)
A0 = [0, 0, 0];
A1 = [5000, 0, 1700];
A2 = [0, 5000, 1700];
A3 = [5000, 5000, 1300];
anchors = [A0; A1; A2; A3];

% 仿真步数
num_steps = 200;

% 生成更复杂的真实目标位置序列
t = linspace(0, 2*pi, num_steps);
true_targets = zeros(3, num_steps);
true_targets(1, :) = 1000 * sin(t) + 2500;
true_targets(2, :) = 1000 * cos(t) + 2500;
true_targets(3, :) = 500 * sin(2*t) + 1000;

% 光速
c = 299792458; 

% 初始状态均值
x0 = [true_targets(1, 1); true_targets(2, 1); true_targets(3, 1)];

% 定义 measurement_noise 的范围
measurement_noise_range = 3:-0.01:0;
num_noise_levels = length(measurement_noise_range);

% 存储不同 measurement_noise 下的平均误差
TOF_avg_errors = zeros(1, num_noise_levels);
UKF_avg_errors = zeros(1, num_noise_levels);

for i = 1:num_noise_levels
    measurement_noise = measurement_noise_range(i);
    
    % 存储TOF定位结果和UKF滤波结果
    TOF_estimates = zeros(3, num_steps);
    UKF_estimates = zeros(3, num_steps);
    
    % 模拟每一步的TOF测量和定位
    for t = 1:num_steps
        true_target = true_targets(:, t);
        % 计算真实距离 (TOF距离)
        true_distances = sqrt(sum((anchors - true_target').^2, 2))'; % 真实距离 (mm)
        tof_times = (true_distances * 1e-3 / c) * 1e9; % 转换为纳秒
        
        % 添加测量噪声 (模拟真实TOF测量)
        noisy_tof_times = tof_times + (measurement_noise/sqrt(3)) * randn(1,size(anchors,1));
        %noisy_tof_times = DS_TWR(measurement_noise,anchors,tof_times);
        
        % 使用TOF定位函数进行定位
        TOF_estimates(:, t) = TOF_localization(anchors, noisy_tof_times);
    end
    
    % 使用UKF对TOF定位结果进行滤波
    UKF_estimates = UKF(anchors, x0, TOF_estimates);
    
    % 计算定位误差
    TOF_errors = sqrt(sum((TOF_estimates - true_targets).^2, 1));
    UKF_errors = sqrt(sum((UKF_estimates - true_targets).^2, 1));
    
    % 计算平均误差
    TOF_avg_errors(i) = mean(TOF_errors);
    UKF_avg_errors(i) = mean(UKF_errors);
end

% 绘制对比图像
figure;
plot(measurement_noise_range, TOF_avg_errors, 'b', 'DisplayName', '无UKF滤波平均误差');
hold on;
plot(measurement_noise_range, UKF_avg_errors, 'r', 'DisplayName', '有UKF滤波平均误差');
xlabel('噪声系数');
ylabel('平均定位误差 (mm)');
title('不同测量噪声水平下有UKF和无UKF的定位误差对比');
legend;
grid on;