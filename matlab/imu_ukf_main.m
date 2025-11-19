function imu_ukf_main()
% IMU UKF attitude & bias estimator with gyro+accel+mag
%
% This is a template. You must provide sensor data arrays:
%   acc_data : 3xN accelerometer (m/s^2)
%   gyro_data: 3xN gyroscope (rad/s)
%   mag_data : 3xN magnetometer
%   t        : 1xN timestamps (s)

% -------------------- USER: load your data here --------------------

close all;

% Select the dataset folder, speed/sequence, and sensor to load.
dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
selection.velocity = 'V_300';   % top-level folder name under dataset/
selection.sequence = 3;         % Sequencia<sequence> folder (set [] if none)
% selection.sensor = 'MPU9150'; % .mat filename without extension
selection.sensor = 'LSM9DS0'; % Alternatives are MPU9150, MPU6500RM3100, MPU6050RM3100, LSM9DS0
% selection.sensor = 'MPU6500RM3100';
% selection.sensor = 'MPU6050RM3100';
                             
                             
                             
[acc_data, gyro_data, mag_data, abb_quaternion, t, meta] = load_imu_dataset(dataset_root, ...
    selection.velocity, selection.sequence, selection.sensor);

g = 9.81;              % gravity magnitude
m_ref_init_len = 50;  % number of samples for initialization

% -------------------- Plot raw sensor data --------------------
plot_raw_imu_data(t, acc_data, gyro_data, mag_data);

% -------------------- UKF parameters --------------------
filter = ukf_init_filter(acc_data, gyro_data, mag_data, abb_quaternion, g, ...
    m_ref_init_len, meta, dataset_root);
n = filter.n;      % state dimension (centralized in initializer)

alpha = filter.alpha;
beta = filter.beta;
kappa = filter.kappa;
Wm = filter.Wm;
Wc = filter.Wc;
lambda = filter.lambda;
Q = filter.Q;
Ra = filter.Ra;
Rm = filter.Rm;

% -------------------- Initialization --------------------
x = filter.x0;
P = filter.P0;
m_ref = filter.m_ref;

N = size(acc_data, 2);
x_hist = zeros(n, N);
x_hist(:,1) = x;

% -------------------- Main UKF loop --------------------
for k = 2:N
    try
        dt  = t(k) - t(k-1);
        z_g = gyro_data(:,k);
        z_a = acc_data(:,k);
        z_m = mag_data(:,k);

        if ~isfinite(dt)
            warning('Skipping step %d due to NaN or Inf timestamp.', k);
            x_hist(:,k) = x;
            continue;
        end

        % Prediction
        if all(isfinite(z_g))
            [x, P] = ukf_predict_state(x, P, Q, z_g, dt, Wm, Wc, lambda);
        else
            warning('Skipping prediction at step %d due to NaN gyroscope data.', k);
        end

        % Accelerometer update
        if all(isfinite(z_a))
            [x, P] = ukf_update(x, P, z_a, Ra, @h_acc, g, Wm, Wc, lambda);
        else
            warning('Skipping accelerometer update at step %d due to NaN data.', k);
        end

        % Magnetometer update
        if all(isfinite(z_m))
            % [x, P] = ukf_update(x, P, z_m, Rm, @h_mag, m_ref, Wm, Wc, lambda);
        else
            warning('Skipping magnetometer update at step %d due to NaN data.', k);
        end

        if any(isnan(x(:))) || any(isnan(P(:)))
            error('UKF state or covariance became NaN at step %d.', k);
        end

        x_hist(:,k) = x;
    catch ukfErr
        stepError = MException('ukf:step', 'UKF update failed at step %d.', k);
        stepError = addCause(stepError, ukfErr);
        throw(stepError);
    end
end

% -------------------- Example output: plot Euler angles --------------------
q_hist = x_hist(1:4, :);
num_samples = N;
q_gt = quat_conjugate(abb_quaternion);

if ~isempty(q_gt)
    num_samples = min(size(q_gt, 2), N);
    if size(q_gt, 2) ~= N
        warning('Ground truth samples (%d) differ from IMU samples (%d). Trimming to %d.', ...
            size(q_gt, 2), N, num_samples);
    end
end

t_plot = t(1:num_samples);
q_est_plot = normalize_quaternions(q_hist(:,1:num_samples));
eul_est = quaternions_to_euler_zyx(q_est_plot);

if isempty(q_gt)
    error('ABB quaternion ground truth not found in dataset metadata.');
end
q_gt_plot = normalize_quaternions(q_gt(:,1:num_samples));
eul_gt = quaternions_to_euler_zyx(q_gt_plot);

plot_attitude_comparison(t_plot, eul_gt, eul_est);

angle_error = unwrap_angles(wrap_to_pi(eul_est - eul_gt));
angle_error_deg = rad2deg(angle_error);
plot_euler_angle_errors(t_plot, angle_error_deg);

q_rel = quat_multiply(q_est_plot, quat_conjugate(q_gt_plot));
q_rel = normalize_quaternions(q_rel);
relative_angle_deg = rad2deg(2 * acos(clamp_unit(q_rel(1,:))));

plot_relative_orientation_angle(t_plot, relative_angle_deg);

end
