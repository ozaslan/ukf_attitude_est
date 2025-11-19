function [x, P, m_ref] = ukf_init_state(acc_init, gyro_init, mag_init, g)
% UKF_INIT_STATE  Initialize 13D state and covariance and reference mag
%
%   acc_init  : 3xN accelerometer samples (stationary)
%   gyro_init : 3xN gyroscope samples (stationary)
%   mag_init  : 3xN magnetometer samples (stationary)
%   g         : gravity magnitude (e.g., 9.81)
%
%   x    : 13x1 state [q; bg; ba; bm]
%   P    : 13x13 covariance
%   m_ref: 3x1 reference mag vector in world frame

mean_acc  = mean(acc_init, 2);
mean_gyro = mean(gyro_init, 2);
mean_mag  = mean(mag_init, 2);

% Normalize gravity direction
hat_g = mean_acc / norm(mean_acc);
gx = hat_g(1);
gy = hat_g(2);
gz = hat_g(3);

% Roll and pitch from accelerometer (yaw = 0)
phi   = atan2(gy,  gz);                       % roll
theta = atan2(-gx, sqrt(gy^2 + gz^2));        % pitch
psi   = 0.0;                                  % yaw

% R_BW = R_y(theta)*R_x(phi) (world -> body)
R_x = [1 0 0;
    0 cos(phi) -sin(phi);
    0 sin(phi)  cos(phi)];
R_y = [ cos(theta) 0 sin(theta);
    0          1 0;
    -sin(theta) 0 cos(theta)];
R_BW = R_y * R_x;

q_BW = rotm_to_quat(R_BW);     % [qw; qx; qy; qz]

bg0 = mean_gyro;               % gyro bias estimate
ba0 = zeros(3,1);              % accel bias ~0 (no easy config)
bm0 = zeros(3,1);              % mag bias ~0

x = [q_BW; bg0; ba0; bm0];

% Initial covariance (tunable)
P = eye(13);
P(1:4,1:4)   = eye(4) * 1e-2;  % orientation
P(5:7,5:7)   = eye(3) * 1e-3;  % gyro bias
P(8:10,8:10) = eye(3) * 1e-2;  % accel bias
P(11:13,11:13) = eye(3) * 1e-2; % mag bias

% Reference magnetometer measurement expressed in world frame. The
% measured mean is in the body frame, so rotate it using the transpose of
% R_BW (i.e., the body-to-world rotation) to obtain the world-frame
% reference vector.
m_ref = R_BW' * mean_mag;   % 3x1
end
