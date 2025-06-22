function estimated_pos = TOF_localization(anchors, tof_times)
    % 选取 tof_times 中最小的前 4 个时间的索引
    [~, sorted_indices] = sort(tof_times);
    selected_indices = sorted_indices(1:4);

    % 根据索引选取对应的基站和传播时间
    selected_anchors = anchors(selected_indices, :);
    selected_tof_times = tof_times(selected_indices);

    c = 299792458; % 光速 (m/s)
    % 将传播时间转换为距离
    distances = (selected_tof_times * 1e-9) * c * 1e3; % 转换为距离 (mm)

    num_anchors = size(selected_anchors, 1);
    A = zeros(num_anchors - 1, 3);
    b = zeros(num_anchors - 1, 1);

    % 以第一个锚点为参考
    for i = 2:num_anchors
        A(i - 1, :) = 2 * (selected_anchors(i, :) - selected_anchors(1, :));
        b(i - 1) = distances(1)^2 - distances(i)^2 + ...
            sum(selected_anchors(i, :).^2) - sum(selected_anchors(1, :).^2);
    end

    % 使用论文中的加权矩阵设计方法
    sum_distances = sum(distances(2:end));  % 只计算参与矩阵运算的距离之和
    w = 1 - distances(2:end) / sum_distances;  % 计算权重向量 w_i = 1 - (d_i / sum(d))
    
    % 确保权重非负（处理可能的数值问题）
    w = max(1e-10, w);  % 设置最小权重阈值，避免除零或极小值
    
    % 构建对角加权矩阵
    W = diag(w);

    % 添加正则化项提高稳定性
    lambda = 1e-6;  % 正则化参数
    reg_term = lambda * eye(3);
    
    % 加权最小二乘解
    estimated_pos = (A' * W * A + reg_term) \ (A' * W * b);
end
