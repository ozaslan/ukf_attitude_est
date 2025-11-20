function plot_tilt_angle_comparison(t_plot, eul_gt, tilt_eul)
%PLOT_TILT_ANGLE_COMPARISON Compare roll/pitch from ABB quaternion vs. accel-only tilt.
%   PLOT_TILT_ANGLE_COMPARISON(t_plot, eul_gt, tilt_eul) plots the roll and
%   pitch angles suggested by the ABB quaternion ground truth alongside the
%   simple accelerometer-derived tilt estimate.

figure('Name','ABB vs. accelerometer tilt','Color','w');

subplot(2,1,1);
plot(t_plot, rad2deg(eul_gt(3,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(tilt_eul(3,:)), '--', 'LineWidth', 1.5, 'Color', [0.2 0.3 0.8]);
ylabel('Roll (deg)');
legend({'ABB ground truth','Accel tilt'}, 'Location', 'best');
title('Roll (X-axis rotation)');

subplot(2,1,2);
plot(t_plot, rad2deg(eul_gt(2,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(tilt_eul(2,:)), '--', 'LineWidth', 1.5, 'Color', [0.2 0.3 0.8]);
ylabel('Pitch (deg)');
xlabel('Time (s)');
legend({'ABB ground truth','Accel tilt'}, 'Location', 'best');
title('Pitch (Y-axis rotation)');

sgtitle('ABB quaternion vs. accelerometer-only tilt (ZYX yaw-pitch-roll)');
end
