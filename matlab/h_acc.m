function z_pred = h_acc(x, g)
% H_ACC  Accelerometer measurement model
%
%   z_pred = (-g) * [2(qx qz + qy qw);
%                    2(qy qz - qx qw);
%                    1 - 2(qx^2 + qy^2)] + b_a

try
    q  = x(1:4);
    ba = x(8:10);

    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    hx = (g) * 2 * (qx*qz + qy*qw);
    hy = (g) * 2 * (qy*qz - qx*qw);
    hz = (g) * (1 - 2*(qx^2 + qy^2));

    z_pred = [hx; hy; hz] + ba;

    if any(isnan(z_pred(:)))
        error('h_acc produced NaN values.');
    end
catch ME
    rethrow(addCause(MException('ukf:h_acc', 'h_acc failed'), ME));
end

end
