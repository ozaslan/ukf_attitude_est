function q = quat_multiply(q1, q2)
% QUAT_MULTIPLY  Quaternion product q = q1 ⊗ q2
%   q1, q2 are [qw; qx; qy; qz] and can be 4xN with matching columns

if size(q1, 1) ~= 4 || size(q2, 1) ~= 4
    error('Both quaternions must be shaped as 4xN.');
end

% Allow multiplying one quaternion by many (broadcast along columns)
n1 = size(q1, 2);
n2 = size(q2, 2);
if n1 ~= n2
    if n1 == 1
        q1 = repmat(q1, 1, n2);
    elseif n2 == 1
        q2 = repmat(q2, 1, n1);
    else
        error('Quaternion arrays must have the same number of columns or one column.');
    end
end

w1 = q1(1, :); v1 = q1(2:4, :);
w2 = q2(1, :); v2 = q2(2:4, :);

w = w1 .* w2 - sum(v1 .* v2, 1);
v = w1 .* v2 + w2 .* v1 + cross(v1, v2, 1);

q = [w; v];
end
