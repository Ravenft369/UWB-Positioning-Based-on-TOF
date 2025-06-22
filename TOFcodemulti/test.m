% 定义 9 个基站的位置
A0 = [0, 0, 1300];
A1 = [5000, 0, 1700];
A2 = [0, 5000, 1700];
A3 = [5000, 5000, 1300];
A4 = [2500, 2500, 1500];
A5 = [1000, 1000, 1400];
A6 = [4000, 1000, 1600];
A7 = [1000, 4000, 1600];
A8 = [4000, 4000, 1400];
anchors = [A0; A1; A2; A3; A4; A5; A6; A7; A8];

% 定义真实目标位置
true_target = [2500; 2500; 1000];

% 定义测量噪声水平
measurement_noise = 0.1;

% 调用 DS_TWR 函数获取 TOF 时间
noisy_tof_times = DS_TWR(measurement_noise, anchors, true_target);

% 调用 TOF_localization 函数进行定位
estimated_pos = TOF_localization(anchors, noisy_tof_times);

% 计算定位误差
error = norm(estimated_pos - true_target);

% 显示结果
fprintf('真实目标位置: [%.2f, %.2f, %.2f] mm\n', true_target);
fprintf('估计位置: [%.2f, %.2f, %.2f] mm\n', estimated_pos);
fprintf('定位误差: %.2f mm\n', error);