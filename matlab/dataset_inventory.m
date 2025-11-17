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
    inventory = append_files(inventory, velocity_name, [], velocity_path);

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

        inventory = append_files(inventory, velocity_name, seq_num, seq_path);
    end
end

end

function inventory = append_files(inventory, velocity_name, sequence_num, folder)
files = dir(fullfile(folder, '*.mat'));
for f = 1:numel(files)
    sensor_name = erase(files(f).name, '.mat');
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
