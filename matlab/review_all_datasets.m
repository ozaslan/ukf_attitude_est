function good_datasets = review_all_datasets(dataset_root)
%REVIEW_ALL_DATASETS Iterate through every dataset and mark quality.
%   good_datasets = review_all_datasets(dataset_root) loads each available
%   dataset, plots the raw IMU signals together with ABB quaternion-derived
%   Euler angles in a single screen-filling figure, and prompts the user to
%   mark the dataset as "Good" or "Bad". The function returns a struct
%   array (subset of the dataset inventory) for the datasets marked as good.
%
%   The dialog displays how many datasets remain to be reviewed to ensure
%   none are skipped. Close the dialog only after selecting an option to
%   proceed to the next dataset.

if nargin < 1 || isempty(dataset_root)
    dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
end

inventory = dataset_inventory(dataset_root);
if isempty(inventory)
    error('No datasets found under %s.', dataset_root);
end

n_total = numel(inventory);
good_mask = false(1, n_total);

for k = 1:n_total
    item = inventory(k);
    [acc_data, gyro_data, mag_data, abb_quaternion, t] = load_imu_dataset( ...
        dataset_root, item.velocity, item.sequence, item.sensor);

    fig_title = sprintf('Dataset %d/%d: %s / Sequencia%s / %s', ...
        k, n_total, item.velocity, sequence_label(item.sequence), item.sensor);

    fig = figure('Name', fig_title, 'Color', 'w', ...
        'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

    tiled = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    component_colors = [
        0.85 0.33 0.10; ... % x
        0.10 0.60 0.30; ... % y
        0.20 0.30 0.80  ... % z
        ];
    component_labels = {'X', 'Y', 'Z'};

    plot_vector_components(nexttile(tiled), t, acc_data, component_colors, ...
        component_labels, 'Accel (m/s^2)', 'Accelerometer');
    plot_vector_components(nexttile(tiled), t, rad2deg(gyro_data), component_colors, ...
        component_labels, 'Gyro (deg/s)', 'Gyroscope');
    plot_vector_components(nexttile(tiled), t, mag_data, component_colors, ...
        component_labels, 'Mag', 'Magnetometer');

    if isempty(abb_quaternion)
        % Reserve space so the layout remains consistent.
        for empty_plot = 1:3
            ax = nexttile(tiled);
            text(ax, 0.5, 0.5, 'No ABB quaternion in dataset', ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            axis(ax, 'off');
        end
    else
        q_gt = normalize_quaternions(abb_quaternion);
        n_samples = min(size(q_gt, 2), numel(t));
        if size(q_gt, 2) ~= numel(t)
            warning(['ABB quaternion samples (%d) differ from IMU samples (%d). ' ...
                'Trimming to %d.'], size(q_gt, 2), numel(t), n_samples);
        end
        t_plot = t(1:n_samples);
        eul_gt = quaternions_to_euler_zyx(q_gt(:, 1:n_samples));
        plot_euler_components(tiled, t_plot, eul_gt);
    end

    title(tiled, fig_title);
    drawnow;

    remaining = n_total - k;
    prompt = sprintf(['Dataset %d/%d:\n  Velocity: %s\n  Sequencia: %s\n' ...
        '  Sensor: %s\n\nDatasets remaining: %d'], ...
        k, n_total, item.velocity, sequence_label(item.sequence), ...
        item.sensor, remaining);

    choice = questdlg(prompt, 'Dataset quality check', 'Good', 'Bad', 'Good');
    while isempty(choice)
        choice = questdlg(prompt, 'Dataset quality check', 'Good', 'Bad', 'Good');
    end

    good_mask(k) = strcmp(choice, 'Good');
    close(fig);
end

good_datasets = inventory(good_mask);

fprintf('Marked %d/%d datasets as good.\n', nnz(good_mask), n_total);
for idx = 1:numel(good_datasets)
    item = good_datasets(idx);
    fprintf('  - %s / Sequencia%s / %s\n', item.velocity, ...
        sequence_label(item.sequence), item.sensor);
end

end

function label = sequence_label(seq)
if isempty(seq)
    label = 'none';
else
    label = num2str(seq);
end
end

function plot_vector_components(ax, t, data, colors, labels, y_label, title_text)
axes(ax); %#ok<LAXES>
hold(ax, 'on'); grid(ax, 'on');
for i = 1:3
    plot(ax, t, data(i, :), 'LineWidth', 1.2, 'Color', colors(i, :));
end
ylabel(ax, y_label);
legend(ax, labels, 'Location', 'best');
title(ax, title_text);
end

function plot_euler_components(tiled, t_plot, eul_gt)
labels = {'Yaw (deg)', 'Pitch (deg)', 'Roll (deg)'};
order = [1 2 3]; % yaw, pitch, roll order from quaternions_to_euler_zyx output
for idx = 1:3
    ax = nexttile(tiled);
    plot(ax, t_plot, rad2deg(eul_gt(order(idx), :)), 'LineWidth', 1.25, ...
        'Color', [0.2 0.6 0.2]);
    grid(ax, 'on');
    ylabel(ax, labels{idx});
    if idx == 3
        xlabel(ax, 'Time (s)');
    end
    title(ax, erase(labels{idx}, ' (deg)'));
end
end
