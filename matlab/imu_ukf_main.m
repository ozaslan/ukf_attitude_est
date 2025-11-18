function imu_ukf_main()
% IMU UKF attitude & bias estimator with gyro+accel+mag
%
% This is a template. You must provide sensor data arrays:
%   acc_data : 3xN accelerometer (m/s^2)
%   gyro_data: 3xN gyroscope (rad/s)
%   mag_data : 3xN magnetometer
%   t        : 1xN timestamps (s)

% -------------------- USER: load your data here --------------------
% Select the dataset folder, speed/sequence, and sensor to load.
dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
selection.velocity = 'eV50p';   % top-level folder name under dataset/
selection.sequence = 1;         % Sequencia<sequence> folder (set [] if none)
selection.sensor = 'MPU6500DMP'; % .mat filename without extension

[acc_data, gyro_data, mag_data, t, meta] = load_imu_dataset(dataset_root, ...
    selection.velocity, selection.sequence, selection.sensor);

g = 9.81;              % gravity magnitude
m_ref_init_len = 500;  % number of samples for initialization

% -------------------- UKF parameters --------------------
n = 13;           % state dimension
alpha = 0.3;      % spread parameter (keeps n+lambda well-conditioned)
beta  = 2;
kappa = 0;

[Wm, Wc, lambda] = ukf_weights(n, alpha, beta, kappa);

% Process noise (example values, tune for your IMU)
q_var  = 1e-8;   % small quaternion process noise
bg_var = 1e-8;
ba_var = 1e-7;
bm_var = 1e-7;
Q = diag([ q_var*ones(4,1); ...
    bg_var*ones(3,1); ...
    ba_var*ones(3,1); ...
    bm_var*ones(3,1) ]);

% Measurement noise covariances (from stationary segments or datasheet)
Ra = eye(3) * 1e-3;      % accel noise covariance
Rm = eye(3) * 5e-3;      % mag noise covariance

% -------------------- Initialization --------------------
acc_init  = acc_data(:,1:m_ref_init_len);
gyro_init = gyro_data(:,1:m_ref_init_len);
mag_init  = mag_data(:,1:m_ref_init_len);

[x, P, m_ref] = ukf_init_state(acc_init, gyro_init, mag_init, g);

N = size(acc_data, 2);
x_hist = zeros(n, N);
x_hist(:,1) = x;

% -------------------- Main UKF loop --------------------
for k = 2:N
    dt  = t(k) - t(k-1);
    z_g = gyro_data(:,k);
    z_a = acc_data(:,k);
    z_m = mag_data(:,k);

    % Prediction
    [x, P] = ukf_predict_state(x, P, Q, z_g, dt, Wm, Wc, lambda);

    % Accelerometer update
    [x, P] = ukf_update(x, P, z_a, Ra, @h_acc, g, Wm, Wc, lambda);

    % Magnetometer update
    [x, P] = ukf_update(x, P, z_m, Rm, @h_mag, m_ref, Wm, Wc, lambda);

    x_hist(:,k) = x;
end

% -------------------- Example output: plot Euler angles --------------------
q_hist = x_hist(1:4, :);
num_samples = N;
q_gt = [];

if isfield(meta, 'abb_quaternion') && ~isempty(meta.abb_quaternion)
    q_gt = meta.abb_quaternion;
    num_samples = min(size(q_gt, 2), N);
    if size(q_gt, 2) ~= N
        warning('Ground truth samples (%d) differ from IMU samples (%d). Trimming to %d.', ...
            size(q_gt, 2), N, num_samples);
    end
end

t_plot = t(1:num_samples);
q_est_plot = normalize_quaternions(q_hist(:,1:num_samples));

eul_est = zeros(3, num_samples);
for k = 1:num_samples
    R = quat_to_rotm(q_est_plot(:,k));
    % ZYX yaw-pitch-roll (for example)
    eul_est(:,k) = rotm2eul_zyx(R);
end
eul_est = unwrap_angles(eul_est);

if isempty(q_gt)
    error('ABB quaternion ground truth not found in dataset metadata.');
end

q_gt_plot = normalize_quaternions(q_gt(:,1:num_samples));
eul_gt = zeros(3, num_samples);
for k = 1:num_samples
    eul_gt(:,k) = rotm2eul_zyx(quat_to_rotm(q_gt_plot(:,k)));
end
eul_gt = unwrap_angles(eul_gt);

figure('Name','Attitude Estimate vs ABB Ground Truth','Color','w');
subplot(3,1,1);
plot(t_plot, rad2deg(eul_gt(3,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_est(3,:)), '--', 'LineWidth', 1.5, 'Color', [0.9 0.3 0.2]);
ylabel('Roll (deg)');
legend({'ABB ground truth','UKF estimate'}, 'Location', 'best');
title('Roll (X-axis rotation)');

subplot(3,1,2);
plot(t_plot, rad2deg(eul_gt(2,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_est(2,:)), '--', 'LineWidth', 1.5, 'Color', [0.9 0.3 0.2]);
ylabel('Pitch (deg)');
legend({'ABB ground truth','UKF estimate'}, 'Location', 'best');
title('Pitch (Y-axis rotation)');

subplot(3,1,3);
plot(t_plot, rad2deg(eul_gt(1,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_est(1,:)), '--', 'LineWidth', 1.5, 'Color', [0.9 0.3 0.2]);
ylabel('Yaw (deg)');
xlabel('Time (s)');
legend({'ABB ground truth','UKF estimate'}, 'Location', 'best');
title('Yaw (Z-axis rotation)');
sgtitle('UKF Attitude vs. ABB Ground Truth (ZYX yaw-pitch-roll)');

angle_error = wrap_to_pi(eul_est - eul_gt);
angle_error_deg = rad2deg(angle_error);

figure('Name','Euler Angle Errors','Color','w');
plot(t_plot, angle_error_deg(3,:), 'LineWidth', 1.5, 'Color', [0.2 0.3 0.7]);
hold on; grid on;
plot(t_plot, angle_error_deg(2,:), 'LineWidth', 1.5, 'Color', [0.9 0.5 0.1]);
plot(t_plot, angle_error_deg(1,:), 'LineWidth', 1.5, 'Color', [0.5 0.1 0.6]);
legend({'Roll error','Pitch error','Yaw error'}, 'Location', 'best');
xlabel('Time (s)');
ylabel('Angle error (deg)');
title('Euler angle error (estimate - ground truth)');

q_rel = quat_multiply(q_est_plot, quat_conjugate(q_gt_plot));
q_rel = normalize_quaternions(q_rel);
relative_angle_deg = rad2deg(2 * acos(clamp_unit(q_rel(1,:))));

figure('Name','Relative Orientation Angle','Color','w');
plot(t_plot, relative_angle_deg, 'LineWidth', 1.6, 'Color', [0.1 0.5 0.8]);
grid on;
xlabel('Time (s)');
ylabel('Geodesic angle (deg)');
title('SO(3) relative angle between estimate and ABB quaternion');

end

function q_norm = normalize_quaternions(q)
norms = vecnorm(q);
norms(norms == 0) = 1;
q_norm = q ./ norms;
end

function angles = wrap_to_pi(angles)
angles = mod(angles + pi, 2*pi) - pi;
end

function clamped = clamp_unit(value)
clamped = min(1, max(-1, value));
end

function unwrapped = unwrap_angles(angles)
% Smooth cyclic angle trajectories by removing 2*pi jumps along each row
unwrapped = angles;
for i = 1:size(angles, 1)
    unwrapped(i, :) = unwrap(angles(i, :));
end
end
