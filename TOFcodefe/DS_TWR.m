function true_times = DS_TWR(measurement_noise, anchors, true_target)
    c = 299792458; 
    true_distances = sqrt(sum((anchors - true_target').^2, 2))'; % mm

    noise_scale = 0.1;  % 噪声缩放因子(控制整体噪声强度)
    distance_coefficient = 1 + noise_scale * (true_distances ./ (1 + true_distances));
    
    tof_times = (true_distances * 1e-3 / c) * 1e9; % ns
    
    % 调整标准差，使高斯噪声的强度与原均匀噪声相当
    gaussian_std = measurement_noise / sqrt(3); % 匹配原均匀分布的标准差
    
    % 生成高斯白噪声
    noise_prop1 = gaussian_std .* randn(1,size(anchors,1)) .* distance_coefficient;
    noise_prop2 = gaussian_std .* randn(1,size(anchors,1)) .* distance_coefficient;
    noise_prop3 = gaussian_std .* randn(1,size(anchors,1)) .* distance_coefficient;
    
    times_prop1 = tof_times + noise_prop1;
    times_prop2 = tof_times + noise_prop2;
    times_prop3 = tof_times + noise_prop3;
    times_reply = 0.01;
    times_round1 = times_prop1 + times_prop2 + times_reply;
    times_round2 = times_prop2 + times_prop3 + times_reply;
    true_times = (times_round1-2*times_reply+times_round2)/4;
end