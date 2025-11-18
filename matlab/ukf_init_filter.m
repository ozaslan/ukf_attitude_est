function filter = ukf_init_filter(acc_data, gyro_data, mag_data, abb_quaternion, g, m_ref_init_len)
%UKF_INIT_FILTER Configure UKF parameters and initial state.
%
%   acc_data       : 3xN accelerometer data (m/s^2)
%   gyro_data      : 3xN gyroscope data (rad/s)
%   mag_data       : 3xN magnetometer data
%   abb_quaternion : 4xN ground truth orientation (qw, qx, qy, qz)
%   g              : gravity magnitude (e.g., 9.81)
%   m_ref_init_len : number of samples for reference mag/acc init
%
%   filter: struct containing UKF dimensions, weights, noise, and
%           initialized state/covariance/reference fields.

if nargin < 6
    error('All inputs including ground truth quaternions are required.');
end

filter.n = 13;               % state dimension
filter.alpha = 0.3;          % spread parameter (keeps n+lambda well-conditioned)
filter.beta = 2;
filter.kappa = 0;

[filter.Wm, filter.Wc, filter.lambda] = ukf_weights(filter.n, ...
    filter.alpha, filter.beta, filter.kappa);

% Process noise (example values, tune for your IMU)
q_var  = 1e-6;   % small quaternion process noise
bg_var = 1e-6;
ba_var = 1e-5;
bm_var = 1e-5;
filter.Q = diag([ q_var*ones(4,1); ...
    bg_var*ones(3,1); ...
    ba_var*ones(3,1); ...
    bm_var*ones(3,1) ]);

% Measurement noise covariances (from stationary segments or datasheet)
filter.Ra = eye(3) * 1e-3;      % accel noise covariance
filter.Rm = eye(3) * 5e-3;      % mag noise covariance

% -------------------- Initialization --------------------
acc_init  = acc_data(:,1:m_ref_init_len);
gyro_init = gyro_data(:,1:m_ref_init_len);
mag_init  = mag_data(:,1:m_ref_init_len);

[x, P, m_ref] = ukf_init_state(acc_init, gyro_init, mag_init, g);

if isempty(abb_quaternion)
    error('Ground truth ABB quaternion is required to initialize the filter state.');
end

q_gt0 = normalize_quaternions(abb_quaternion(:,1));
filter.x0 = x;
filter.x0(1:4) = q_gt0;  % initialize orientation from ground truth
filter.P0 = P;
filter.m_ref = m_ref;
end
