function x_next = f_state(x, z_g, dt)
% F_STATE  Process model: quaternion propagated by gyro, biases random walk
%
%   x    : 13x1 state vector [q; bg; ba; bm]
%   z_g  : 3x1 gyro measurement
%   dt   : time step

% Unpack
q  = x(1:4);       % [qw; qx; qy; qz]
bg = x(5:7);
ba = x(8:10);
bm = x(11:13);

% Bias-compensated angular velocity
omega = z_g - bg;

% Incremental quaternion
theta = norm(omega) * dt;
if theta > 0
    u = omega / norm(omega);
    dq = [cos(theta/2);
        u * sin(theta/2)];
else
    dq = [1; 0; 0; 0];
end

% Quaternion update
q_next = quat_multiply(q, dq);
q_next = q_next / norm(q_next);

% Biases: mean stays the same (random walk)
bg_next = bg;
ba_next = ba;
bm_next = bm;

x_next = [q_next; bg_next; ba_next; bm_next];

end
