function q_conj = quat_conjugate(q)
%QUAT_CONJUGATE Quaternion conjugate for [qw; qx; qy; qz] column(s)
%   Input:  q - 4xN quaternion array
%   Output: q_conj - 4xN quaternion array with vector part negated

if size(q,1) ~= 4
    error('Quaternion must be shaped as 4xN.');
end

q_conj = [q(1,:); -q(2:4,:)];
end
