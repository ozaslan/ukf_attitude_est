function plot_covariance_diagonals(t, P_diag_hist)
%PLOT_COVARIANCE_DIAGONALS Visualize state covariance diagonals.
%
%   t            : 1xN time vector
%   P_diag_hist  : 13xN history of covariance diagonal entries
%
% Generates a single figure with subplots for quaternion, gyro bias, accel
% bias, and magnetometer bias covariance elements.

if nargin < 2 || isempty(P_diag_hist)
    warning('No covariance history provided to plot_covariance_diagonals.');
    return;
end

if size(P_diag_hist, 2) ~= numel(t)
    warning(['Covariance history length (%d) differs from time vector (%d). ' ...
        'Trimming to the minimum length.'], size(P_diag_hist, 2), numel(t));
end

num_samples = min(size(P_diag_hist, 2), numel(t));
time = reshape(t(1:num_samples), 1, []);
cov_hist = P_diag_hist(:, 1:num_samples);
cov_hist = reshape(cov_hist, size(P_diag_hist, 1), num_samples);

segment_indices = {
    1:4, ...
    5:7, ...
    8:10, ...
    11:13 ...
};

segment_titles = {
    'Quaternion Covariance Diagonals', ...
    'Gyro Bias Covariance Diagonals', ...
    'Accel Bias Covariance Diagonals', ...
    'Mag Bias Covariance Diagonals' ...
};

segment_labels = {
    {'q_w','q_x','q_y','q_z'}, ...
    {'b_{gx}','b_{gy}','b_{gz}'}, ...
    {'b_{ax}','b_{ay}','b_{az}'}, ...
    {'b_{mx}','b_{my}','b_{mz}'} ...
};

fig = figure('Name', 'State Covariance Diagonals', 'NumberTitle', 'off');
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

num_states = size(cov_hist, 1);

for i = 1:numel(segment_indices)
    idx = segment_indices{i};
    valid_idx = idx(idx <= num_states);
    nexttile;

    if isempty(valid_idx)
        axis off;
        text(0.5, 0.5, 'No data available', 'HorizontalAlignment', 'center');
        continue;
    end

    plot(time, cov_hist(valid_idx, :).');
    grid on;
    xlabel('Time (s)');
    ylabel('Covariance');
    title(segment_titles{i});

    legend(segment_labels{i}(1:numel(valid_idx)), ...
        'Interpreter', 'tex', 'Location', 'best');
end
end
