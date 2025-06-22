function X = GDOP(selected_anchors, estimated_pos)
    % 输入参数：
    % selected_anchors : N×3矩阵，选中的锚点坐标（必须≥4个锚点）
    % estimated_pos    : 3×1向量，待评估点的估计位置
    
    % 确保至少4个锚点
    if size(selected_anchors,1) < 4
        error('至少需要4个锚点计算GDOP');
    end
    
    % 提取参考锚点（第一个锚点）
    ref_anchor = selected_anchors(1,:);
    x0 = estimated_pos(1);
    y0 = estimated_pos(2);
    z0 = estimated_pos(3);
    
    % 构造H矩阵（方向余弦差）
    H = [];
    for i = 2:size(selected_anchors,1)
        anchor_i = selected_anchors(i,:);
        ri = norm(estimated_pos - anchor_i');    % 到当前锚点的距离
        r_ref = norm(estimated_pos - ref_anchor'); % 到参考锚点的距离
        
        % 计算方向余弦差
        dx = (x0 - anchor_i(1))/ri - (x0 - ref_anchor(1))/r_ref;
        dy = (y0 - anchor_i(2))/ri - (y0 - ref_anchor(2))/r_ref;
        dz = (z0 - anchor_i(3))/ri - (z0 - ref_anchor(3))/r_ref;
        
        H = [H; dx, dy, dz]; % 每行对应一个观测
    end
    
    % 计算GDOP（增加正则化防止奇异）
    lambda = 1e-8;
    X = sqrt(trace(inv(H'*H + lambda*eye(3))));
end