function z_pred = h_mag(x, m_ref)
% H_MAG  Magnetometer measurement model
%
%   z_pred = R(q)*m_ref + b_m

q  = x(1:4);
bm = x(11:13);

R = quat_to_rotm(q);

z_pred = R * m_ref + bm;

end
