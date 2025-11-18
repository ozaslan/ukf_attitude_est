function plot_euler_angle_errors(t_plot, angle_error_deg)
%PLOT_EULER_ANGLE_ERRORS Plot Euler angle errors over time.
figure('Name','Euler Angle Errors','Color','w');
plot(t_plot, angle_error_deg(3,:), 'LineWidth', 1.5, 'Color', [0.2 0.3 0.7]);
hold on; grid on;
plot(t_plot, angle_error_deg(2,:), 'LineWidth', 1.5, 'Color', [0.9 0.5 0.1]);
plot(t_plot, angle_error_deg(1,:), 'LineWidth', 1.5, 'Color', [0.5 0.1 0.6]);
legend({'Roll error','Pitch error','Yaw error'}, 'Location', 'best');
xlabel('Time (s)');
ylabel('Angle error (deg)');
title('Euler angle error (estimate - ground truth)');
end
