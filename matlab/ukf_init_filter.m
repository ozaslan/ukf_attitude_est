function filter = ukf_init_filter(acc_data, gyro_data, mag_data, abb_quaternion, g, m_ref_init_len, meta, dataset_root)
%UKF_INIT_FILTER Configure UKF parameters and initial state.
%
%   acc_data       : 3xN accelerometer data (m/s^2)
%   gyro_data      : 3xN gyroscope data (rad/s)
%   mag_data       : 3xN magnetometer data
%   abb_quaternion : 4xN ground truth orientation (qw, qx, qy, qz)
%   g              : gravity magnitude (e.g., 9.81)
%   m_ref_init_len : number of samples for reference mag/acc init
%   meta           : struct returned by load_imu_dataset (used for sensor selection)
%   dataset_root   : root of the dataset tree (used to locate sensor_noise_stats.mat)
%
%   filter: struct containing UKF dimensions, weights, noise, and
%           initialized state/covariance/reference fields.

if nargin < 6
    error('All inputs including ground truth quaternions are required.');
end

if nargin < 7
    meta = struct();
end

if nargin < 8 || isempty(dataset_root)
    dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
end

filter.n = 13;               % state dimension
filter.alpha = 0.3;          % spread parameter (keeps n+lambda well-conditioned)
filter.beta = 2;
filter.kappa = 0;

[filter.Wm, filter.Wc, filter.lambda] = ukf_weights(filter.n, ...
    filter.alpha, filter.beta, filter.kappa);

noise_stats = load_noise_statistics(dataset_root, meta);

% Process noise derived from sensor noise statistics
[q_var, bg_var, ba_var, bm_var] = derive_process_noise(noise_stats);
filter.Q = diag([ q_var*ones(4,1); ...
    bg_var*ones(3,1) / 100; ...
    ba_var*ones(3,1) / 100; ...
    bm_var*ones(3,1) ]);

% Measurement noise covariances (per-axis)
filter.Ra = diag(select_variance(noise_stats, 'acc'));
filter.Rm = diag(select_variance(noise_stats, 'mag'));

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
filter.x0(1:4) = quat_conjugate(q_gt0);  % initialize orientation from ground truth
filter.P0 = P;
filter.m_ref = m_ref;
end

function noise_stats = load_noise_statistics(dataset_root, meta)
stats_file = fullfile(dataset_root, 'sensor_noise_stats.mat');

if exist(stats_file, 'file') ~= 2
    fprintf('Sensor noise stats not found at %s. Computing...\n', stats_file);
    compute_sensor_noise(dataset_root, [], stats_file);
end

raw = load(stats_file, 'noise_stats');
if ~isfield(raw, 'noise_stats') || isempty(raw.noise_stats)
    error('Noise statistics file %s is missing expected variable noise_stats.', stats_file);
end

if isfield(meta, 'sensor') && ~isempty(meta.sensor)
    idx = find(strcmpi({raw.noise_stats.sensor}, meta.sensor), 1);
else
    idx = [];
end

if isempty(idx)
    warning(['Sensor %s not found in noise stats. Using aggregate of all sensors.'], ...
        safe_sensor_name(meta)); %#ok<SPWRN>
    noise_stats = merge_noise_stats(raw.noise_stats);
else
    noise_stats = raw.noise_stats(idx).aggregate;
end
end

function merged = merge_noise_stats(stats_array)
acc_vars = collect_variances(stats_array, 'acc_variance');
gyro_vars = collect_variances(stats_array, 'gyro_variance');
mag_vars = collect_variances(stats_array, 'mag_variance');

merged = struct( ...
    'acc_variance', mean(acc_vars, 2, 'omitnan'), ...
    'gyro_variance', mean(gyro_vars, 2, 'omitnan'), ...
    'mag_variance', mean(mag_vars, 2, 'omitnan'));
end

function vars = collect_variances(stats_array, field_name)
vars = [];
for i = 1:numel(stats_array)
    agg = stats_array(i).aggregate;
    if isfield(agg, field_name) && ~isempty(agg.(field_name))
        vars = [vars, agg.(field_name)]; %#ok<AGROW>
    end
end
if isempty(vars)
    vars = nan(3,1);
end
end

function [q_var, bg_var, ba_var, bm_var] = derive_process_noise(noise_stats)
gyro_var = select_variance(noise_stats, 'gyro');
acc_var = select_variance(noise_stats, 'acc');
mag_var = select_variance(noise_stats, 'mag');

q_var = mean(gyro_var, 'omitnan');
if isnan(q_var)
    q_var = 1e-6;
end

bg_var = mean(gyro_var, 'omitnan');
if isnan(bg_var)
    bg_var = 1e-6;
end

ba_var = mean(acc_var, 'omitnan');
if isnan(ba_var)
    ba_var = 1e-5;
end

bm_var = mean(mag_var, 'omitnan');
if isnan(bm_var)
    bm_var = 1e-5;
end
end

function variance = select_variance(noise_stats, field_prefix)
field_name = [field_prefix '_variance'];
if isstruct(noise_stats) && isfield(noise_stats, field_name)
    variance = noise_stats.(field_name);
else
    variance = nan(3,1);
end

if isempty(variance)
    variance = nan(3,1);
end

if numel(variance) == 1
    variance = repmat(variance, 3, 1);
end

replacement = mean(variance, 'omitnan');
if isnan(replacement)
    replacement = 0;
end
variance = fillmissing(variance(:), 'constant', replacement); %#ok<FILLM>

if any(isnan(variance))
    % Fallback to small default values when statistics are missing.
    switch field_prefix
        case 'acc'
            variance = 1e-3 * ones(3,1);
        case 'mag'
            variance = 5e-3 * ones(3,1);
        otherwise
            variance = 1e-6 * ones(3,1);
    end
end
end

function name = safe_sensor_name(meta)
if isstruct(meta) && isfield(meta, 'sensor')
    name = meta.sensor;
else
    name = 'unknown';
end
end
