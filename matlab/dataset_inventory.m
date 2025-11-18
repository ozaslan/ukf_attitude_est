function inventory = dataset_inventory(dataset_root)
%DATASET_INVENTORY Enumerate available dataset combinations.
%
%   inventory = dataset_inventory(dataset_root) returns a struct array with
%   fields:
%     velocity : Top-level folder name under dataset/
%     sequence : Numeric sequence index (empty when none)
%     sensor   : Sensor name (derived from .mat filename)
%     path     : Full path to the .mat file

if nargin < 1 || isempty(dataset_root)
    dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
end

allowed_sensors = allowed_sensor_names();
allowed_keys = allowed_dataset_keys();

inventory = struct('velocity', {}, 'sequence', {}, 'sensor', {}, 'path', {});
velocity_dirs = dir(dataset_root);

for v = 1:numel(velocity_dirs)
    entry = velocity_dirs(v);
    if ~entry.isdir || startsWith(entry.name, '.')
        continue;
    end

    velocity_name = entry.name;
    velocity_path = fullfile(dataset_root, velocity_name);

    % Files directly under the velocity directory
    inventory = append_files(inventory, velocity_name, [], velocity_path, ...
        allowed_sensors, allowed_keys);

    % Sequencia folders inside the velocity directory
    seq_dirs = dir(fullfile(velocity_path, 'Sequencia*'));
    for s = 1:numel(seq_dirs)
        seq_entry = seq_dirs(s);
        if ~seq_entry.isdir
            continue;
        end

        seq_name = seq_entry.name;
        seq_num = parse_sequence_number(seq_name);
        seq_path = fullfile(velocity_path, seq_name);

        inventory = append_files(inventory, velocity_name, seq_num, seq_path, ...
            allowed_sensors, allowed_keys);
    end
end

end

function inventory = append_files(inventory, velocity_name, sequence_num, folder, allowed_sensors, allowed_keys)
files = dir(fullfile(folder, '*.mat'));
for f = 1:numel(files)
    sensor_name = erase(files(f).name, '.mat');
    if ~is_allowed_sensor(sensor_name, allowed_sensors)
        continue;
    end
    if ~is_allowed_dataset(velocity_name, sequence_num, sensor_name, allowed_keys)
        continue;
    end
    inventory(end+1) = struct('velocity', velocity_name, ...
        'sequence', sequence_num, 'sensor', sensor_name, ...
        'path', fullfile(folder, files(f).name)); %#ok<AGROW>
end
end

function seq_num = parse_sequence_number(seq_name)
match = regexp(seq_name, 'Sequencia(\d+)', 'tokens', 'once');
if isempty(match)
    seq_num = [];
else
    seq_num = str2double(match{1});
end
end

function names = allowed_sensor_names()
names = upper({
    'MPU9150', ...
    'MPU6500RM3100', ...
    'MPU6500DMP', ...
    'MPU6050RM3100', ...
    'MPU6050DMP', ...
    'LSM9DS0'
    });
end

function keys = allowed_dataset_keys()
entries = {
    'V1000', 1, 'LSM9DS0';
    'V1000', 1, 'MPU6050RM3100';
    'V1000', 1, 'MPU6500RM3100';
    'V1000', 1, 'MPU9150';
    'V1000', 2, 'LSM9DS0';
    'V1000', 3, 'LSM9DS0';
    'V1000', 3, 'MPU6050RM3100';
    'V1000', 3, 'MPU6500RM3100';
    'V1000', 3, 'MPU9150';
    'V1000', 4, 'LSM9DS0';
    'V1000', 4, 'MPU6050RM3100';
    'V1000', 4, 'MPU6500RM3100';
    'V1000', 4, 'MPU9150';
    'V1500', 1, 'LSM9DS0';
    'V1500', 1, 'MPU6050RM3100';
    'V1500', 1, 'MPU6500RM3100';
    'V1500', 1, 'MPU9150';
    'V1500', 3, 'LSM9DS0';
    'V1500', 3, 'MPU6050RM3100';
    'V1500', 3, 'MPU6500RM3100';
    'V1500', 3, 'MPU9150';
    'V1500', 4, 'LSM9DS0';
    'V1500', 4, 'MPU6050RM3100';
    'V1500', 4, 'MPU6500RM3100';
    'V1500', 4, 'MPU9150';
    'V_300', 1, 'LSM9DS0';
    'V_300', 1, 'MPU6050RM3100';
    'V_300', 1, 'MPU6500RM3100';
    'V_300', 1, 'MPU9150';
    'V_300', 3, 'LSM9DS0';
    'V_300', 3, 'MPU6050RM3100';
    'V_300', 3, 'MPU6500RM3100';
    'V_300', 3, 'MPU9150';
    'V_300', 4, 'LSM9DS0';
    'V_300', 4, 'MPU6050RM3100';
    'V_300', 4, 'MPU6500RM3100';
    'V_300', 4, 'MPU9150';
    'V_500', 1, 'LSM9DS0';
    'V_500', 1, 'MPU6050RM3100';
    'V_500', 1, 'MPU6500RM3100';
    'V_500', 1, 'MPU9150';
    'V_500', 2, 'LSM9DS0';
    'V_500', 3, 'LSM9DS0';
    'V_500', 3, 'MPU6050RM3100';
    'V_500', 3, 'MPU6500RM3100';
    'V_500', 3, 'MPU9150';
    'V_500', 4, 'LSM9DS0';
    'V_500', 4, 'MPU6050RM3100';
    'V_500', 4, 'MPU6500RM3100';
    'V_500', 4, 'MPU9150';
    'V_800', 1, 'LSM9DS0';
    'V_800', 1, 'MPU6050RM3100';
    'V_800', 1, 'MPU6500RM3100';
    'V_800', 1, 'MPU9150';
    'V_800', 3, 'LSM9DS0';
    'V_800', 3, 'MPU6050RM3100';
    'V_800', 3, 'MPU6500RM3100';
    'V_800', 3, 'MPU9150';
    'V_800', 4, 'LSM9DS0';
    'V_800', 4, 'MPU6050RM3100';
    'V_800', 4, 'MPU6500RM3100';
    'V_800', 4, 'MPU9150';
    'eV50p', [], 'MPU6050RM3100';
    'eV50p', 1, 'LSM9DS0';
    'eV50p', 1, 'MPU6050RM3100';
    'eV50p', 1, 'MPU6500RM3100';
    'eV50p', 1, 'MPU9150';
    'eV50p', 2, 'LSM9DS0';
    'eV50p', 2, 'MPU6050RM3100';
    'eV50p', 2, 'MPU6500RM3100';
    'eV50p', 2, 'MPU9150';
    'eV50p', 3, 'LSM9DS0';
    'eV50p', 3, 'MPU6050RM3100';
    'eV50p', 3, 'MPU6500RM3100';
    'eV50p', 3, 'MPU9150';
    'eV50p', 4, 'LSM9DS0';
    'eV50p', 4, 'MPU6050RM3100';
    'eV50p', 4, 'MPU6500RM3100';
    'eV50p', 4, 'MPU9150';
    };

keys = cell(1, size(entries, 1));
for idx = 1:size(entries, 1)
    keys{idx} = dataset_key(entries{idx, 1}, entries{idx, 2}, entries{idx, 3});
end
end

function tf = is_allowed_sensor(sensor_name, allowed_sensors)
tf = any(strcmpi(sensor_name, allowed_sensors));
end

function tf = is_allowed_dataset(velocity_name, sequence_num, sensor_name, allowed_keys)
key = dataset_key(velocity_name, sequence_num, sensor_name);
tf = any(strcmpi(key, allowed_keys));
end

function key = dataset_key(velocity_name, sequence_num, sensor_name)
if isempty(sequence_num)
    seq_label = 'none';
else
    seq_label = num2str(sequence_num);
end
key = sprintf('%s|%s|%s', upper(char(velocity_name)), seq_label, upper(char(sensor_name)));
end
