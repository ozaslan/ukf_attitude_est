function eul = quaternions_to_euler_zyx(q_series)
%QUATERNIONS_TO_EULER_ZYX Convert quaternion time series to ZYX Euler angles.
%   q_series must be 4xN. Output eul is 3xN in radians.

q_series = normalize_quaternions(q_series);
num_samples = size(q_series, 2);
eul = zeros(3, num_samples);
for k = 1:num_samples
    R = quat_to_rotm(q_series(:,k));
    eul(:,k) = rotm2eul_zyx(R);
end

eul = unwrap_angles(eul);
end
