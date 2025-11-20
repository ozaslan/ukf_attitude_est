function plot_bias_estimates(t, acc_data, x_hist, g)
%PLOT_BIAS_ESTIMATES Visualize accelerometer magnitude and sensor biases.
%
%   t         : 1xN time vector
%   acc_data  : 3xN accelerometer measurements (m/s^2)
%   x_hist    : 13xN UKF state history [q; bg; ba; bm]
%   g         : gravity magnitude reference (default: 9.81)
%
% Generates a single figure with subplots for accelerometer magnitude
% (raw vs bias-removed), accelerometer bias, gyroscope bias, and
% magnetometer bias.

if nargin < 3 || isempty(x_hist)
    warning('No state history provided to plot_bias_estimates.');
    return;
end

if nargin < 4 || isempty(g)
    g = 9.81;
end

num_samples = min([numel(t), size(acc_data, 2), size(x_hist, 2)]);
if num_samples == 0
    warning('No overlapping samples to plot in plot_bias_estimates.');
    return;
end

time = reshape(t(1:num_samples), 1, []);
acc_samples = acc_data(:, 1:num_samples);
state_hist = x_hist(:, 1:num_samples);

component_colors = [
    0.85 0.33 0.10; ... % x
    0.10 0.60 0.30; ... % y
    0.20 0.30 0.80  ... % z
    ];
component_labels = {'X', 'Y', 'Z'};

acc_bias = state_hist(8:10, :);
gyro_bias = state_hist(5:7, :);
mag_bias = state_hist(11:13, :);

acc_magnitude_raw = vecnorm(acc_samples, 2, 1);
acc_samples_bias_removed = acc_samples - acc_bias;
acc_magnitude_bias_removed = vecnorm(acc_samples_bias_removed, 2, 1);

window_size = max(1, round(0.05 * num_samples));
acc_mag_raw_mean = movmean(acc_magnitude_raw, window_size);
acc_mag_bias_removed_mean = movmean(acc_magnitude_bias_removed, window_size);

fig = figure('Name', 'Bias Estimates and Accelerometer Magnitude', ...
    'NumberTitle', 'off', 'Color', 'w');
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(time, acc_magnitude_raw, 'LineWidth', 1.2, 'Color', [0.12 0.47 0.71]);
hold on; grid on;
plot(time, acc_mag_raw_mean, 'LineWidth', 2, 'Color', [0.12 0.47 0.71]);
plot(time, acc_magnitude_bias_removed, 'LineWidth', 1.2, 'Color', [0.17 0.63 0.17]);
plot(time, acc_mag_bias_removed_mean, 'LineWidth', 2, 'Color', [0.17 0.63 0.17]);
yline(g, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Accel Magnitude (m/s^2)');
title('Accelerometer Magnitude');
legend({'Raw', 'Raw (mean filtered)', 'Bias removed', ...
    'Bias removed (mean filtered)', 'Gravity'}, 'Location', 'best');

nexttile;
plot(time, acc_bias(1,:), 'LineWidth', 1.2, 'Color', component_colors(1,:));
hold on; grid on;
plot(time, acc_bias(2,:), 'LineWidth', 1.2, 'Color', component_colors(2,:));
plot(time, acc_bias(3,:), 'LineWidth', 1.2, 'Color', component_colors(3,:));
xlabel('Time (s)');
ylabel('Accel Bias (m/s^2)');
title('Accelerometer Bias');
legend(component_labels, 'Location', 'best');

nexttile;
plot(time, gyro_bias(1,:), 'LineWidth', 1.2, 'Color', component_colors(1,:));
hold on; grid on;
plot(time, gyro_bias(2,:), 'LineWidth', 1.2, 'Color', component_colors(2,:));
plot(time, gyro_bias(3,:), 'LineWidth', 1.2, 'Color', component_colors(3,:));
xlabel('Time (s)');
ylabel('Gyro Bias (rad/s)');
title('Gyroscope Bias');
legend(component_labels, 'Location', 'best');

nexttile;
plot(time, mag_bias(1,:), 'LineWidth', 1.2, 'Color', component_colors(1,:));
hold on; grid on;
plot(time, mag_bias(2,:), 'LineWidth', 1.2, 'Color', component_colors(2,:));
plot(time, mag_bias(3,:), 'LineWidth', 1.2, 'Color', component_colors(3,:));
xlabel('Time (s)');
ylabel('Mag Bias');
title('Magnetometer Bias');
legend(component_labels, 'Location', 'best');

sgtitle(layout, 'Sensor Bias Estimates and Accelerometer Magnitude');
end
