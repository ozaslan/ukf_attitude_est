function plot_relative_orientation_angle(t_plot, relative_angle_deg)
%PLOT_RELATIVE_ORIENTATION_ANGLE Plot geodesic angle between quaternions.
figure('Name','Relative Orientation Angle','Color','w');
plot(t_plot, relative_angle_deg, 'LineWidth', 1.6, 'Color', [0.1 0.5 0.8]);
grid on;
xlabel('Time (s)');
ylabel('Geodesic angle (deg)');
title('SO(3) relative angle between estimate and ABB quaternion');
end
