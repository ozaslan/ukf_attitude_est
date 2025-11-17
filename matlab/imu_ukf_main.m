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
selection.sensor = 'XSENS';     % .mat filename without extension

[acc_data, gyro_data, mag_data, t] = load_imu_dataset(dataset_root, ...
    selection.velocity, selection.sequence, selection.sensor);

g = 9.81;              % gravity magnitude
m_ref_init_len = 500;  % number of samples for initialization

% -------------------- UKF parameters --------------------
n = 13;           % state dimension
alpha = 1e-3;
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
eul_hist = zeros(3, N);
for k = 1:N
    R = quat_to_rotm(q_hist(:,k));
    % ZYX yaw-pitch-roll (for example)
    eul_hist(:,k) = rotm2eul_zyx(R);
end

figure;
subplot(3,1,1); plot(t, rad2deg(eul_hist(3,:))); ylabel('Roll (deg)');
subplot(3,1,2); plot(t, rad2deg(eul_hist(2,:))); ylabel('Pitch (deg)');
subplot(3,1,3); plot(t, rad2deg(eul_hist(1,:))); ylabel('Yaw (deg)');
xlabel('Time (s)');
sgtitle('UKF Attitude (ZYX yaw-pitch-roll)');

end
