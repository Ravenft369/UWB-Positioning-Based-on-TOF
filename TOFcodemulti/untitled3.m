anchors = [0, 0, 0; 1, 0, 0; 0, 1, 0; 1, 1, 0; 2, 2, 0; 3, 3, 0; 4, 4, 0; 5, 5, 0; 6, 6, 0];
tof_times = [10e-9, 5e-9, 3e-9, 8e-9, 12e-9, 15e-9, 20e-9, 25e-9, 30e-9];

% 调用改进后的 TOF_localization 函数
estimated_pos = TOF_localization(anchors, tof_times);
disp(estimated_pos);