function [x_pred, P_pred] = ukf_predict_state(x, P, Q, z_g, dt, Wm, Wc, lambda)
% UKF_PREDICT_STATE  UKF time update for state x with gyro input z_g
%
%   x, P : current mean and covariance
%   Q    : process noise covariance (13x13)
%   z_g  : 3x1 gyro measurement
%   dt   : timestep
%   Wm,Wc,lambda: UKF weights and scaling

    n = numel(x);
    % Sigma points from (x,P)
    X = sigma_points(x, P, lambda);

    % Propagate each sigma point through process model
    L = size(X,2);   % 2n+1
    X_pred = zeros(n, L);
    for i = 1:L
        X_pred(:,i) = f_state(X(:,i), z_g, dt);
    end

    % Predicted mean
    x_pred = zeros(n,1);
    for i = 1:L
        x_pred = x_pred + Wm(i) * X_pred(:,i);
    end

    % Renormalize quaternion in mean
    x_pred(1:4) = x_pred(1:4) / norm(x_pred(1:4));

    % Predicted covariance
    P_pred = zeros(n,n);
    for i = 1:L
        dx = X_pred(:,i) - x_pred;
        P_pred = P_pred + Wc(i) * (dx * dx.');
    end
    P_pred = P_pred + Q;
end
