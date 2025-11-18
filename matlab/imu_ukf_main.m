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

[acc_data, gyro_data, mag_data, abb_quaternion, t, meta] = load_imu_dataset(dataset_root, ...
    selection.velocity, selection.sequence, selection.sensor);

g = 9.81;              % gravity magnitude
m_ref_init_len = 500;  % number of samples for initialization

% -------------------- Plot raw sensor data --------------------
plot_raw_imu_data(t, acc_data, gyro_data, mag_data);

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
    try
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
q_gt = abb_quaternion;

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
