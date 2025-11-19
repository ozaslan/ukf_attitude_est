function z_pred = h_acc(x, g)
% H_ACC  Accelerometer measurement model
%
% The accelerometer measures gravity expressed in the body frame plus the
% accelerometer bias.  For a unit quaternion q that rotates vectors from the
% body frame to the inertial frame, the rotation matrix R = quat_to_rotm(q)
% has its third column equal to
%
%   R(:,3) = [2(qx*qz + qy*qw);
%             2(qy*qz - qx*qw);
%             1 - 2*(qx^2 + qy^2)].
%
% Therefore z_pred = g * R(:,3) + b_a.

try
    q  = x(1:4);
    ba = x(8:10);

    R = quat_to_rotm(q);
    z_pred = g * R(:,3) + ba;

    if any(isnan(z_pred(:)))
        error('h_acc produced NaN values.');
    end
catch ME
    rethrow(addCause(MException('ukf:h_acc', 'h_acc failed'), ME));
end

end
