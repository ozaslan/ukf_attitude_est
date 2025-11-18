function angles = wrap_to_pi(angles)
%WRAP_TO_PI Wrap angles to the [-pi, pi] interval.
angles = mod(angles + pi, 2*pi) - pi;
end
