function clamped = clamp_unit(value)
%CLAMP_UNIT Clamp values to the closed interval [-1, 1].
clamped = min(1, max(-1, value));
end
