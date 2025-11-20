function plot_gyro_angle_comparison(t_plot, eul_gt, eul_gyro)
%PLOT_GYRO_ANGLE_COMPARISON Compare ABB quaternion vs. gyro-only integration.
%   PLOT_GYRO_ANGLE_COMPARISON(t_plot, eul_gt, eul_gyro) overlays the
%   yaw, pitch, and roll angles derived from the ABB quaternion ground truth
%   with the angles obtained by integrating gyroscope measurements starting
%   from the initial ABB orientation.

figure('Name','ABB vs. gyro integration','Color','w');

subplot(3,1,1);
plot(t_plot, rad2deg(eul_gt(3,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_gyro(3,:)), '--', 'LineWidth', 1.5, 'Color', [0.8 0.3 0.2]);
ylabel('Roll (deg)');
legend({'ABB ground truth','Gyro integration'}, 'Location', 'best');
title('Roll (X-axis rotation)');

subplot(3,1,2);
plot(t_plot, rad2deg(eul_gt(2,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_gyro(2,:)), '--', 'LineWidth', 1.5, 'Color', [0.8 0.3 0.2]);
ylabel('Pitch (deg)');
legend({'ABB ground truth','Gyro integration'}, 'Location', 'best');
title('Pitch (Y-axis rotation)');

subplot(3,1,3);
plot(t_plot, rad2deg(eul_gt(1,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_gyro(1,:)), '--', 'LineWidth', 1.5, 'Color', [0.8 0.3 0.2]);
ylabel('Yaw (deg)');
xlabel('Time (s)');
legend({'ABB ground truth','Gyro integration'}, 'Location', 'best');
title('Yaw (Z-axis rotation)');

sgtitle('ABB quaternion vs. gyroscope-only integration (ZYX yaw-pitch-roll)');
end
