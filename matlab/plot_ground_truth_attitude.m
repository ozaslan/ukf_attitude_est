function plot_ground_truth_attitude(t_plot, eul_gt)
%PLOT_GROUND_TRUTH_ATTITUDE Plot ABB ground truth Euler angles.
figure('Name','ABB Ground Truth Attitude','Color','w');
subplot(3,1,1);
plot(t_plot, rad2deg(eul_gt(3,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
grid on;
ylabel('Roll (deg)');
title('Roll (X-axis rotation)');

subplot(3,1,2);
plot(t_plot, rad2deg(eul_gt(2,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
grid on;
ylabel('Pitch (deg)');
title('Pitch (Y-axis rotation)');

subplot(3,1,3);
plot(t_plot, rad2deg(eul_gt(1,:)), 'LineWidth', 1.25, 'Color', [0.2 0.6 0.2]);
grid on;
ylabel('Yaw (deg)');
xlabel('Time (s)');
title('Yaw (Z-axis rotation)');
sgtitle('ABB ground truth attitude (ZYX yaw-pitch-roll)');
end
