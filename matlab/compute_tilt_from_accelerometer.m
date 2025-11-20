function tilt_eul = compute_tilt_from_accelerometer(acc_data)
%COMPUTE_TILT_FROM_ACCELEROMETER Estimate yaw, pitch, and roll using only accelerometer data.
%   tilt_eul = COMPUTE_TILT_FROM_ACCELEROMETER(acc_data) returns a 3xN
%   matrix of yaw-pitch-roll angles (ZYX convention) computed from the
%   accelerometer samples in acc_data. The computation assumes gravity is
%   aligned with the sensor +Z axis when level and ignores accelerometer
%   bias so that the result behaves like a simple tilt sensor.
%
%   Inputs:
%       acc_data : 3xN accelerometer measurements in m/s^2.
%
%   Outputs:
%       tilt_eul : 3xN matrix of yaw (all zeros), pitch, and roll angles
%                  in radians following the ZYX yaw-pitch-roll ordering
%                  used elsewhere in the project.
%
%   Roll is computed from the projection of gravity onto the Y-Z plane,
%   and pitch is computed from the projection onto the X-Z plane. Yaw
%   cannot be observed from the accelerometer alone and is set to zero.

arguments
    acc_data (3, :) double
end

ax = acc_data(1, :);
ay = acc_data(2, :);
az = acc_data(3, :);

roll = atan2(ay, az);
pitch = atan2(-ax, sqrt(ay.^2 + az.^2));

tilt_eul = [zeros(1, numel(ax)); pitch; roll];
end
