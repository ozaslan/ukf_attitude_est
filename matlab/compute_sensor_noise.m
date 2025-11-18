function noise_stats = compute_sensor_noise(dataset_root, window_size, output_path)
%COMPUTE_SENSOR_NOISE Estimate per-axis sensor noise across all datasets.
%
%   noise_stats = compute_sensor_noise(dataset_root, window_size, output_path)
%   traverses every allowed dataset, applies a moving-average filter to the
%   raw accelerometer, gyroscope, and magnetometer signals, and computes the
%   variance of the residual (raw - smoothed) for each axis. Results are
%   aggregated per sensor type and written to a MAT file for later reuse.
%
%   Inputs (all optional):
%     dataset_root : Root folder for the dataset tree (default: ../dataset).
%     window_size  : Moving-average window length in samples (default: 5).
%     output_path  : Destination .mat file (default: dataset_root/sensor_noise_stats.mat).
%
%   Output:
%     noise_stats  : Struct array, one element per sensor type, containing:
%        - sensor       : Sensor name.
%        - window_size  : Averaging window length.
%        - datasets     : Per-dataset variance statistics.
%        - aggregate    : Variance across all datasets for this sensor.
%
%   Example:
%     addpath('matlab');
%     stats = compute_sensor_noise();
%
%   The generated MAT file mirrors the noise_stats output variable.

if nargin < 1 || isempty(dataset_root)
    dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
end

if nargin < 2 || isempty(window_size)
    window_size = 5;
end

if nargin < 3 || isempty(output_path)
    output_path = fullfile(dataset_root, 'sensor_noise_stats.mat');
end

inventory = dataset_inventory(dataset_root);
unique_sensors = unique({inventory.sensor});

noise_stats = struct('sensor', {}, 'window_size', {}, 'datasets', {}, 'aggregate', {});

for s = 1:numel(unique_sensors)
    sensor_name = unique_sensors{s};
    entries = inventory(strcmp({inventory.sensor}, sensor_name));

    residual_acc_all = [];
    residual_gyro_all = [];
    residual_mag_all = [];
    dataset_stats = struct('velocity', {}, 'sequence', {}, 'path', {}, ...
        'acc_variance', {}, 'gyro_variance', {}, 'mag_variance', {});

    for k = 1:numel(entries)
        entry = entries(k);
        [acc_data, gyro_data, mag_data] = load_imu_dataset(dataset_root, ...
            entry.velocity, entry.sequence, sensor_name); %#ok<ASGLU>

        [acc_residuals, acc_var] = compute_residuals(acc_data, window_size);
        [gyro_residuals, gyro_var] = compute_residuals(gyro_data, window_size);
        [mag_residuals, mag_var] = compute_residuals(mag_data, window_size);

        residual_acc_all = [residual_acc_all, acc_residuals]; %#ok<AGROW>
        residual_gyro_all = [residual_gyro_all, gyro_residuals]; %#ok<AGROW>
        residual_mag_all = [residual_mag_all, mag_residuals]; %#ok<AGROW>

        dataset_stats(end+1) = struct( ...
            'velocity', entry.velocity, ...
            'sequence', entry.sequence, ...
            'path', entry.path, ...
            'acc_variance', acc_var, ...
            'gyro_variance', gyro_var, ...
            'mag_variance', mag_var); %#ok<AGROW>
    end

    aggregate_stats = struct( ...
        'acc_variance', aggregate_variance(residual_acc_all), ...
        'gyro_variance', aggregate_variance(residual_gyro_all), ...
        'mag_variance', aggregate_variance(residual_mag_all));

    noise_stats(end+1) = struct( ...
        'sensor', sensor_name, ...
        'window_size', window_size, ...
        'datasets', dataset_stats, ...
        'aggregate', aggregate_stats); %#ok<AGROW>
end

if ~isempty(output_path)
    save(output_path, 'noise_stats');
    fprintf('Saved sensor noise statistics to %s\n', output_path);
end

end

function [residuals, variance_values] = compute_residuals(data, window_size)
if isempty(data)
    residuals = [];
    variance_values = [];
    return;
end

smoothed = movmean(data, window_size, 2);
residuals = data - smoothed;
variance_values = var(residuals, 0, 2);
end

function variance_values = aggregate_variance(residuals)
if isempty(residuals)
    variance_values = [];
    return;
end

variance_values = var(residuals, 0, 2);
end
