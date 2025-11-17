function [acc_data, gyro_data, mag_data, t, meta] = load_imu_dataset(dataset_root, velocity, sequence, sensor, field_map)
%LOAD_IMU_DATASET Load accelerometer, gyroscope, magnetometer, and time data.
%
%   [acc_data, gyro_data, mag_data, t, meta] = load_imu_dataset(
%       dataset_root, velocity, sequence, sensor)
%
%   Arguments:
%     dataset_root : Root folder that contains the dataset/ tree.
%     velocity     : Name of the top-level folder (e.g., 'eV50p', 'V_300').
%     sequence     : Sequence number (e.g., 1, 2, 3, 4). Set [] when the
%                    .mat file is directly inside the velocity folder.
%     sensor       : Sensor name (e.g., 'XSENS', 'MPU6050RM3100').
%     field_map    : Optional struct with fields 'acc', 'gyro', 'mag', 't'
%                    to explicitly select variables inside the .mat file.
%
%   Returns:
%     acc_data, gyro_data, mag_data : 3xN arrays.
%     t                             : 1xN time vector.
%     meta                          : Struct describing the chosen dataset.
%
%   The function automatically scans for available datasets and provides
%   informative errors when the requested combination is missing.

if nargin < 1 || isempty(dataset_root)
    dataset_root = fullfile(fileparts(mfilename('fullpath')), '..', 'dataset');
end

if nargin < 4
    error('Provide dataset_root, velocity, sequence, and sensor parameters.');
end

allowed_sensors = allowed_sensor_names();
if ~is_allowed_sensor(sensor, allowed_sensors)
    error(['Unsupported sensor "' sensor '". ', ...
        'Allowed sensors: ' strjoin(allowed_sensors, ', '), '.']);
end

inventory = dataset_inventory(dataset_root);
mat_file = resolve_dataset_path(inventory, dataset_root, velocity, sequence, sensor);

raw = load(mat_file);
raw = unwrap_single_struct(raw);

if nargin >= 5 && ~isempty(field_map)
    acc_data  = raw.(field_map.acc);
    gyro_data = raw.(field_map.gyro);
    mag_data  = raw.(field_map.mag);
    t         = raw.(field_map.t);
else
    % Default field names constrained to dataset definitions.
    acc_data  = select_field(raw, {'sensacc'});
    gyro_data = select_field(raw, {'sensgyro'});
    mag_data  = select_field(raw, {'sensmag'});
    t         = select_field(raw, {'matlabtime'});
end

[acc_data, gyro_data, mag_data, t] = normalize_shapes(acc_data, gyro_data, mag_data, t);
[acc_data, gyro_data, mag_data] = align_to_abb_axes(sensor, acc_data, gyro_data, mag_data);

abb_quaternion = select_field(raw, {'abbquaternion'});

% Unit conversions based on dataset documentation.
% - Accelerometer: g -> m/s^2
% - Gyroscope    : deg/s -> rad/s
acc_data  = acc_data * 9.81;
gyro_data = deg2rad(gyro_data);

meta = struct('path', mat_file, 'velocity', velocity, 'sequence', sequence, ...
    'sensor', sensor, 'abb_quaternion', abb_quaternion);

end

function mat_file = resolve_dataset_path(inventory, dataset_root, velocity, sequence, sensor)
matches = strcmp({inventory.velocity}, velocity) & strcmp({inventory.sensor}, sensor);

if isempty(sequence)
    seq_matches = cellfun(@(s) isempty(s), {inventory.sequence});
else
    seq_matches = cellfun(@(s) ~isempty(s) && isequal(s, sequence), {inventory.sequence});
end

idx = find(matches & seq_matches, 1);
if ~isempty(idx)
    mat_file = inventory(idx).path;
    return;
end

% Fall back to checking the expected path and provide context when missing.
mat_file = expected_path(dataset_root, velocity, sequence, sensor);
if exist(mat_file, 'file') == 2
    return;
end

error_message = sprintf('Requested dataset not found. Available combinations:\n');
for k = 1:numel(inventory)
    seq_label = inventory(k).sequence;
    if isempty(seq_label)
        seq_label = 'none';
    end
    error_message = [error_message, sprintf('  - %s / Sequencia%s / %s\n', ...
        inventory(k).velocity, num2str(seq_label), inventory(k).sensor)]; %#ok<AGROW>
end

error('%sExpected path: %s', error_message, mat_file);
end

function mat_file = expected_path(dataset_root, velocity, sequence, sensor)
folder = fullfile(dataset_root, velocity);
if ~isempty(sequence)
    folder = fullfile(folder, sprintf('Sequencia%d', sequence));
end
mat_file = fullfile(folder, [sensor '.mat']);
end

function raw = unwrap_single_struct(raw)
% If the loaded .mat file wraps data inside a single struct, unwrap it.
fields = fieldnames(raw);
if numel(fields) == 1 && isstruct(raw.(fields{1}))
    raw = raw.(fields{1});
end
end

function value = select_field(raw, candidates)
names = fieldnames(raw);
lower_names = lower(names);

match_idx = find(matches_any(lower_names, candidates), 1);
if isempty(match_idx)
    error('Could not find expected fields (%s). Available: %s', ...
        strjoin(candidates, ', '), strjoin(names, ', '));
end

value = raw.(names{match_idx});

if isstruct(value)
    % If the field itself is a struct, try to find numeric arrays within it.
    inner_names = fieldnames(value);
    numeric_idx = find(structfun(@(v) isnumeric(v), value), 1);
    if ~isempty(numeric_idx)
        value = value.(inner_names{numeric_idx});
    end
end
end

function tf = matches_any(names, candidates)
tf = false(size(names));
for i = 1:numel(candidates)
    tf = tf | contains(names, lower(candidates{i}));
end
end

function [acc_data, gyro_data, mag_data, t] = normalize_shapes(acc_data, gyro_data, mag_data, t)
acc_data = orient_vector_data(acc_data);
gyro_data = orient_vector_data(gyro_data);
mag_data = orient_vector_data(mag_data);

t = reshape(t, 1, []);
end

function data = orient_vector_data(data)
if ismatrix(data)
    if size(data, 1) == 3
        return;
    elseif size(data, 2) == 3
        data = data';
    end
end
end

function [acc_data, gyro_data, mag_data] = align_to_abb_axes(sensor, acc_data, gyro_data, mag_data)
%ALIGN_TO_ABB_AXES Align sensor axes to the ABB robot frame as in reference script.
% The dataset documentation provides explicit re-orderings per sensor type.

sensor = upper(sensor);

switch sensor
    case 'MPU9150'
        % magx = ABBy, magy = ABBx, magz = -ABBz
        aux = mag_data(1,:);
        mag_data(1,:) = mag_data(2,:);
        mag_data(2,:) = aux;
        mag_data(3,:) = -mag_data(3,:);

    case 'MPU6050RM3100'
        % magx = -ABBy, magy = ABBx, magz = -ABBz
        aux = mag_data(1,:);
        mag_data(1,:) = -mag_data(2,:);
        mag_data(2,:) = -aux;
        mag_data(3,:) = -mag_data(3,:);

    case 'MPU6500RM3100'
        % magx = -ABBy, magy = ABBx, magz = -ABBz
        aux = mag_data(1,:);
        mag_data(1,:) = -mag_data(2,:);
        mag_data(2,:) = -aux;
        mag_data(3,:) = -mag_data(3,:);

    case 'LSM9DS0'
        % gyrox = ABBy, gyroy = -ABBx; accx = ABBy, accy = -ABBx
        aux = gyro_data(1,:);
        gyro_data(1,:) = -gyro_data(2,:);
        gyro_data(2,:) = aux;

        aux = acc_data(1,:);
        acc_data(1,:) = -acc_data(2,:);
        acc_data(2,:) = aux;

        % magx = ABBy, magy = -ABBx, magz = -ABBz
        aux = mag_data(1,:);
        mag_data(1,:) = -mag_data(2,:);
        mag_data(2,:) = aux;
        mag_data(3,:) = -mag_data(3,:);

    otherwise
        % No additional alignment needed.
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

function tf = is_allowed_sensor(sensor_name, allowed_sensors)
tf = any(strcmpi(sensor_name, allowed_sensors));
end
