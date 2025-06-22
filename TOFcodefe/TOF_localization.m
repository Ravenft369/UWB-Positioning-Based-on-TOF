function [estimated_pos, gdop] = TOF_localization(anchors, tof_times)
    % 参数设置
    candidate_num = 4;        % 最终选择的锚点数
    pre_select_num = 8;       % 预选锚点数（从前8近的锚点中选4个）
    max_combinations = 20;    % 最大尝试组合数（防止计算量过大）

    % 初步筛选：选择距离最近的pre_select_num个锚点
    [~, sorted_indices] = sort(tof_times);
    candidate_indices = sorted_indices(1:pre_select_num);
    
    % 生成候选组合（随机选取部分组合）
    num_candidates = min(nchoosek(pre_select_num, candidate_num), max_combinations);
    all_combinations = zeros(num_candidates, candidate_num);
    for i = 1:num_candidates
        all_combinations(i,:) = randperm(pre_select_num, candidate_num);
    end

    % 遍历所有候选组合，寻找GDOP最小的
    min_gdop = Inf;
    best_pos = zeros(3,1);
    for i = 1:size(all_combinations,1)
        selected_indices = candidate_indices(all_combinations(i,:));
        selected_anchors = anchors(selected_indices, :);
        selected_tof = tof_times(selected_indices);
        
        % 计算临时位置和GDOP
        [temp_pos, temp_gdop] = partial_localization(selected_anchors, selected_tof);
        
        % 更新最优解
        if temp_gdop < min_gdop
            min_gdop = temp_gdop;
            best_pos = temp_pos;
        end
    end
    
    estimated_pos = best_pos;
    gdop = min_gdop;
end