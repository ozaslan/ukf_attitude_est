function unwrapped = unwrap_angles(angles)
%UNWRAP_ANGLES Smooth cyclic angle trajectories by removing 2*pi jumps along each row.
unwrapped = angles;
for i = 1:size(angles, 1)
    unwrapped(i, :) = unwrap(angles(i, :));
end
end
