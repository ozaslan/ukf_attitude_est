function plot_imu_dataset()
%PLOT_IMU_DATASET Load an IMU dataset and visualize raw signals and ABB attitude.
%   This helper loads the configured dataset without running the UKF and
%   generates figures for quick inspection.

% -------------------- USER: configure your dataset here --------------------
dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
selection.velocity = 'eV50p';   % top-level folder name under dataset/
selection.sequence = 1;         % Sequencia<sequence> folder (set [] if none)
selection.sensor = 'MPU6500DMP'; % .mat filename without extension
% --------------------------------------------------------------------------

[acc_data, gyro_data, mag_data, abb_quaternion, t] = load_imu_dataset(dataset_root, ...
    selection.velocity, selection.sequence, selection.sensor);

plot_raw_imu_data(t, acc_data, gyro_data, mag_data);

if isempty(abb_quaternion)
    warning('ABB quaternion ground truth not found in the selected dataset.');
    return;
end

q_gt = normalize_quaternions(abb_quaternion);
n_samples = min(size(q_gt, 2), numel(t));

if size(q_gt, 2) ~= numel(t)
    warning('ABB quaternion samples (%d) differ from IMU samples (%d). Trimming to %d.', ...
        size(q_gt, 2), numel(t), n_samples);
end

t_plot = t(1:n_samples);
eul_gt = quaternions_to_euler_zyx(q_gt(:,1:n_samples));
plot_ground_truth_attitude(t_plot, eul_gt);

end
