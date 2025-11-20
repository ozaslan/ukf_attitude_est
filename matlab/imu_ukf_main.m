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
acc_mag_tolerance = 0.5;       % allowable deviation from gravity magnitude (m/s^2)
acc_cov_inflation_gain = 15.0; % scales measurement covariance when |a|-g exceeds tolerance
acc_cov_inflation_max = 75.0;  % hard cap on inflation factor to avoid numerical blow-up

% Gyroscope gating and process noise inflation (based on rotation speed)
gyro_speed_inflation_start_deg = 75; % begin inflating noise above this speed (deg/s)
gyro_speed_disable_deg = 750;         % treat gyro as unavailable above this speed (deg/s)
gyro_cov_inflation_gain = 10.0;       % scales process noise beyond inflation start
gyro_cov_inflation_max = 50.0;       % maximum inflation multiplier
gyro_gate_debug = false;              % emit warnings for gyro gating/inflation

% Accelerometer gating (reject update when magnitude is implausible).
acc_mag_gate_enable = true;        % turn gating on/off without removing code
acc_mag_gate_abs_min = 0.25 * g;   % reject if |a| falls below this absolute floor (m/s^2)
acc_mag_gate_abs_max = 2.0 * g;    % reject if |a| exceeds this ceiling (m/s^2)
acc_mag_gate_error_max = 1.5;      % reject if | |a|-g | exceeds this error (m/s^2)
acc_gate_debug = false;             % emit warnings when a gate is triggered

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

Ra_nominal = Ra;
Q_nominal = Q;

% -------------------- Initialization --------------------
x = filter.x0;
P = filter.P0;
m_ref = filter.m_ref;

N = size(acc_data, 2);
x_hist = zeros(n, N);
x_hist(:,1) = x;
P_diag_hist = zeros(n, N);
P_diag_hist(:,1) = diag(P);

% -------------------- Main UKF loop --------------------
for k = 2:N
    try
        dt  = t(k) - t(k-1);
        z_g = gyro_data(:,k);
        z_a = acc_data(:,k);
        z_m = mag_data(:,k);

        z_g = gyro_data([2,1,3],k);
        z_g = [-1 -1 -1]' .* z_g;

        if ~isfinite(dt)
            warning('Skipping step %d due to NaN or Inf timestamp.', k);
            x_hist(:,k) = x;
            continue;
        end

        % Prediction (gyro-driven) with speed-based gating/inflation
        if all(isfinite(z_g))
            gyro_speed = norm(z_g);              % rad/s
            gyro_speed_deg = rad2deg(gyro_speed);
            gyro_inflation_start = deg2rad(gyro_speed_inflation_start_deg);
            gyro_disable_thresh = deg2rad(gyro_speed_disable_deg);

            gyro_gated = false;
            Q_use = Q_nominal;

            if gyro_speed > gyro_disable_thresh
                gyro_gated = true;
                if gyro_gate_debug
                    warning(['Skipping prediction at step %d: |w|=%.2f deg/s ' ...
                        'exceeds disable threshold %.2f deg/s. Gyro treated as unavailable.'], ...
                        k, gyro_speed_deg, gyro_speed_disable_deg);
                end
            elseif gyro_speed > gyro_inflation_start
                inflation = 1 + gyro_cov_inflation_gain * ...
                    ((gyro_speed - gyro_inflation_start) / gyro_inflation_start);
                inflation = min(inflation, gyro_cov_inflation_max);
                Q_use = Q_nominal * inflation;

                if gyro_gate_debug
                    warning(['Inflating gyro process noise at step %d: |w|=%.2f deg/s ' ...
                        '-> %.2fx (start %.2f deg/s).'], k, gyro_speed_deg, inflation, ...
                        gyro_speed_inflation_start_deg);
                end
            end

            if ~gyro_gated
                [x, P] = ukf_predict_state(x, P, Q_use, z_g, dt, Wm, Wc, lambda);
            end
        else
            warning('Skipping prediction at step %d due to NaN gyroscope data.', k);
        end

        % Accelerometer update
        if all(isfinite(z_a))
            acc_mag = norm(z_a);
            mag_error = abs(acc_mag - g);

            gate_triggered = false;
            if acc_mag_gate_enable
                gate_triggered = (acc_mag < acc_mag_gate_abs_min) || ...
                    (acc_mag > acc_mag_gate_abs_max) || ...
                    (mag_error > acc_mag_gate_error_max);

                if gate_triggered && acc_gate_debug
                    warning(['Skipping accelerometer update at step %d: |a|=%.3f m/s^2 ' ...
                        '(error=%.3f) violated gate thresholds [%.3f, %.3f] or %.3f error.'], ...
                        k, acc_mag, mag_error, acc_mag_gate_abs_min, acc_mag_gate_abs_max, ...
                        acc_mag_gate_error_max);
                end
            end

            if ~gate_triggered
                if mag_error > acc_mag_tolerance
                    inflation = 1 + acc_cov_inflation_gain * (mag_error / g);
                    inflation = min(inflation, acc_cov_inflation_max);
                    Ra = Ra_nominal * inflation;
                else
                    Ra = Ra_nominal;
                end

                [x, P] = ukf_update(x, P, z_a, Ra, @h_acc, g, Wm, Wc, lambda);
            end
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
        P_diag_hist(:,k) = diag(P);
    catch ukfErr
        stepError = MException('ukf:step', 'UKF update failed at step %d.', k);
        stepError = addCause(stepError, ukfErr);
        throw(stepError);
    end
end

% -------------------- Covariance diagnostics --------------------
plot_covariance_diagonals(t, P_diag_hist);

% -------------------- Bias and magnitude visualization --------------------
plot_bias_estimates(t, acc_data, x_hist, g);

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

eul_gt(2, :) = -eul_gt(2, :);
eul_gt(1, :) = -eul_gt(1, :); % ###

plot_attitude_comparison(t_plot, eul_gt, eul_est);

angle_error = unwrap_angles(wrap_to_pi(eul_est - eul_gt));
angle_error_deg = rad2deg(angle_error);
plot_euler_angle_errors(t_plot, angle_error_deg);

q_rel = quat_multiply(q_est_plot, quat_conjugate(q_gt_plot));
q_rel = normalize_quaternions(q_rel);
relative_angle_deg = rad2deg(2 * acos(clamp_unit(q_rel(1,:))));

plot_relative_orientation_angle(t_plot, relative_angle_deg);

end
