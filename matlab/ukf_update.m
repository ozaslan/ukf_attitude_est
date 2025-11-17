function [x_upd, P_upd] = ukf_update(x, P, z_meas, R, h_func, h_param, Wm, Wc, lambda)
% UKF_UPDATE  Generic UKF measurement update
%
%   x,P     : predicted state mean and covariance
%   z_meas  : actual measurement (3x1 here)
%   R       : measurement noise covariance (3x3)
%   h_func  : handle to measurement function, signature z_pred = h_func(x, h_param)
%   h_param : extra parameter for measurement function (e.g., g or m_ref)

n = numel(x);
m = numel(z_meas);

% Sigma points from (x,P)
X = sigma_points(x, P, lambda);
L = size(X,2);

% Ensure quaternion portion of each sigma point remains normalized
for i = 1:L
    X(1:4, i) = X(1:4, i) / norm(X(1:4, i));
end

% Map sigma points into measurement space
Z = zeros(m, L);
for i = 1:L
    Z(:,i) = feval(h_func, X(:,i), h_param);
end

% Predicted measurement mean
z_pred = zeros(m,1);
for i = 1:L
    z_pred = z_pred + Wm(i) * Z(:,i);
end

% Innovation covariance S and cross covariance Pxz
S   = zeros(m,m);
Pxz = zeros(n,m);
for i = 1:L
    dx = X(:,i) - x;
    dz = Z(:,i) - z_pred;
    S   = S   + Wc(i) * (dz * dz.');
    Pxz = Pxz + Wc(i) * (dx * dz.');
end
S = S + R;

% Kalman gain
K = Pxz / S;

% Update
innov = z_meas - z_pred;
x_upd = x + K * innov;
P_upd = P - K * S * K.';

% Renormalize quaternion
x_upd(1:4) = x_upd(1:4) / norm(x_upd(1:4));
end
