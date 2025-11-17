function eul = rotm2eul_zyx(R)
% ROTM2EUL_ZYX  Euler angles (yaw, pitch, roll) from rotation matrix
% R = Rz(yaw)*Ry(pitch)*Rx(roll)
% Returns [yaw; pitch; roll]

r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

yaw   = atan2(r21, r11);
pitch = atan2(-r31, sqrt(r32^2 + r33^2));
roll  = atan2(r32, r33);

eul = [yaw; pitch; roll];
end
