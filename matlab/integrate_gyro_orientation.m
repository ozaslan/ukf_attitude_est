function [q_series, eul_gyro] = integrate_gyro_orientation(gyro_data, t, q0)
%INTEGRATE_GYRO_ORIENTATION Propagate orientation by integrating gyro rates.
%   [q_series, eul_gyro] = INTEGRATE_GYRO_ORIENTATION(gyro_data, t, q0)
%   integrates the body-frame angular velocity measurements in gyro_data
%   starting from the initial quaternion q0. The function returns the
%   propagated quaternion series q_series (4xN, qw-qx-qy-qz) and the
%   corresponding yaw-pitch-roll Euler angles eul_gyro (ZYX ordering) in
%   radians.
%
%   Inputs:
%       gyro_data : 3xN gyroscope measurements (rad/s)
%       t         : 1xN timestamps (s)
%       q0        : 4x1 initial quaternion [qw; qx; qy; qz]
%
%   Outputs:
%       q_series  : 4xN propagated quaternions
%       eul_gyro  : 3xN Euler angles (yaw, pitch, roll) in radians
%
%   The integration assumes piecewise-constant angular velocity between
%   samples and normalizes the quaternion at each step to control numerical
%   drift.

arguments
    gyro_data (3, :) double
    t (1, :) double
    q0 (4, 1) double
end

num_samples = size(gyro_data, 2);
if num_samples ~= numel(t)
    error('Gyro samples (%d) and timestamps (%d) must have the same length.', ...
        num_samples, numel(t));
end

q_current = normalize_quaternions(q0(:));
q_series = zeros(4, num_samples);
q_series(:, 1) = q_current;

for k = 2:num_samples
    dt = t(k) - t(k-1);
    if ~isfinite(dt) || dt <= 0
        warning('Skipping gyro integration at step %d due to invalid dt=%.3g.', k, dt);
        q_series(:, k) = q_current;
        continue;
    end

    omega = gyro_data(:, k);
    theta = norm(omega) * dt;

    if theta > 0
        u = omega / norm(omega);
        dq = [cos(theta/2);
            u * sin(theta/2)];
    else
        dq = [1; 0; 0; 0];
    end

    q_current = quat_multiply(q_current, dq);
    q_current = q_current / norm(q_current);
    q_series(:, k) = q_current;
end

eul_gyro = quaternions_to_euler_zyx(q_series);
end
