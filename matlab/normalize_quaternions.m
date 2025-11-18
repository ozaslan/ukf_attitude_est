function q_norm = normalize_quaternions(q)
%NORMALIZE_QUATERNIONS Normalize quaternion array columns to unit length.
if isempty(q)
    q_norm = q;
    return;
end

norms = vecnorm(q);
norms(norms == 0) = 1;
q_norm = q ./ norms;
end
