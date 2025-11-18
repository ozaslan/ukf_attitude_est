function plot_attitude_comparison(t_plot, eul_gt, eul_est)
%PLOT_ATTITUDE_COMPARISON Plot Euler angles for estimate vs ground truth.
figure('Name','Attitude Estimate vs ABB Ground Truth','Color','w');
subplot(3,1,1);
plot(t_plot, rad2deg(eul_gt(3,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_est(3,:)), '--', 'LineWidth', 1.5, 'Color', [0.9 0.3 0.2]);
ylabel('Roll (deg)');
legend({'ABB ground truth','UKF estimate'}, 'Location', 'best');
title('Roll (X-axis rotation)');

subplot(3,1,2);
plot(t_plot, rad2deg(eul_gt(2,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_est(2,:)), '--', 'LineWidth', 1.5, 'Color', [0.9 0.3 0.2]);
ylabel('Pitch (deg)');
legend({'ABB ground truth','UKF estimate'}, 'Location', 'best');
title('Pitch (Y-axis rotation)');

subplot(3,1,3);
plot(t_plot, rad2deg(eul_gt(1,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
hold on; grid on;
plot(t_plot, rad2deg(eul_est(1,:)), '--', 'LineWidth', 1.5, 'Color', [0.9 0.3 0.2]);
ylabel('Yaw (deg)');
xlabel('Time (s)');
legend({'ABB ground truth','UKF estimate'}, 'Location', 'best');
title('Yaw (Z-axis rotation)');
sgtitle('UKF Attitude vs. ABB Ground Truth (ZYX yaw-pitch-roll)');
end
