function z_pred = h_mag(x, m_ref)
% H_MAG  Magnetometer measurement model
%
%   z_pred = R(q)*m_ref + b_m

try
    q  = x(1:4);
    bm = x(11:13);

    R = quat_to_rotm(q);

    z_pred = R * m_ref + bm;

    if any(isnan(z_pred(:)))
        error('h_mag produced NaN values.');
    end
catch ME
    rethrow(addCause(MException('ukf:h_mag', 'h_mag failed'), ME));
end

end
