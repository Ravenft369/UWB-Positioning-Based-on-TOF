function [pos, gdop] = TOF_localization(selected_anchors, tof_times)
    num_anchors = size(selected_anchors, 1);
    c = 299792458; % 光速 (m/s)
    distances = (tof_times * 1e-9) * c * 1e3; % 转换为距离 (mm)

    % 构建最小二乘方程（以第一个锚点为参考）
    A = zeros(num_anchors-1, 3);
    b = zeros(num_anchors-1, 1);
    for i = 2:num_anchors
        A(i-1,:) = 2*(selected_anchors(i,:) - selected_anchors(1,:));
        b(i-1) = distances(1)^2 - distances(i)^2 + ...
                 sum(selected_anchors(i,:).^2) - sum(selected_anchors(1,:).^2);
    end

    % 加权最小二乘（权重为距离平方倒数）
    relevant_distances = distances(2:end);
    sum_distances = sum(relevant_distances);
    w = 1 - relevant_distances / sum_distances;  % 计算权重向量
    W = diag(w);  % 创建(num_anchors-1) x (num_anchors-1)的对角矩阵
    lambda = 1e-6; % 正则化参数
    pos = (A'*W*A + lambda*eye(3)) \ (A'*W*b);

    % 计算GDOP
    gdop = GDOP(selected_anchors, pos);
end

