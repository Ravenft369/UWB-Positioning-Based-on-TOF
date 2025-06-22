function [X_est] = UKF(anchors, x0, real_positions)
    % 系统参数
    n = 3; % 维度
    m = size(anchors, 1); % 观测维度

  
    P0 = diag([10, 10, 10]);   % 初始协方差
    qp = 0.1;
    Q = diag([qp, qp, qp]); % 过程噪声
    R = diag(repmat(0.1^2, m, 1));% 观测噪声
    h = @(x) sqrt(sum((repmat(x', m, 1) - anchors).^2, 2));
    f = @(x) x; % 状态转移设位置不变

    % UKF参数
    alpha = 1e-3; 
    beta = 2; 
    kappa = 0; 
    lambda = alpha^2 * (n + kappa) - n;
    c = n + lambda;

    % 权重
    Wm = [lambda/c; 0.5/c*ones(2*n,1)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % 初始化
    x = x0; % 初始状态均值x0
    P = P0;
    num_steps = size(real_positions, 2);
    X_est = zeros(n, num_steps);

    for t = 1:num_steps
        % Sigma点
        sqrt_P = chol(P + 1e-6*eye(n))';
        X_sig = [x, x + sqrt_P*sqrt(n + lambda), x - sqrt_P*sqrt(n + lambda)];
        
        % 预测
        X_sig_pred = zeros(n, 2*n + 1);
        for i = 1:2*n+1
            X_sig_pred(:,i) = f(X_sig(:,i));
        end
        x_pred = X_sig_pred * Wm;
        P_pred = Q;
        for i = 1:2*n+1
            P_pred = P_pred + Wc(i)*(X_sig_pred(:,i)-x_pred)*(X_sig_pred(:,i)-x_pred)';
        end
        
        %观测预测
        Y_sig_pred = zeros(m, 2*n+1);
        for i = 1:2*n+1
            Y_sig_pred(:,i) = h(X_sig_pred(:,i));
        end
        y_pred = Y_sig_pred * Wm;
        
        %协方差
        P_yy = R;
        P_xy = zeros(n, m);
        for i = 1:2*n+1
            dy = Y_sig_pred(:,i) - y_pred;
            dx = X_sig_pred(:,i) - x_pred;
            P_yy = P_yy + Wc(i)*(dy*dy');
            P_xy = P_xy + Wc(i)*(dx*dy');
        end
        
        K = P_xy / (P_yy + 1e-6*eye(m));
        z = h(real_positions(:, t)) + sqrt(R)*randn(m,1);
        x = x_pred + K*(z - y_pred);
        P = P_pred - K*P_yy*K';
        [V, D] = eig(P);%正定
        D(D < 0) = 1e-6; 
        P = V * D * V';
        
        X_est(:,t) = x;
    end
end