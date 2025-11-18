function plot_raw_imu_data(t, acc_data, gyro_data, mag_data)
%PLOT_RAW_IMU_DATA Plot accelerometer, gyroscope, and magnetometer signals.
component_colors = [
    0.85 0.33 0.10; ... % x
    0.10 0.60 0.30; ... % y
    0.20 0.30 0.80  ... % z
    ];
component_labels = {'X', 'Y', 'Z'};

figure('Name','Raw IMU Sensor Data','Color','w');

subplot(3,1,1);
plot(t, acc_data(1,:), 'LineWidth', 1.2, 'Color', component_colors(1,:));
hold on; grid on;
plot(t, acc_data(2,:), 'LineWidth', 1.2, 'Color', component_colors(2,:));
plot(t, acc_data(3,:), 'LineWidth', 1.2, 'Color', component_colors(3,:));
ylabel('Accel (m/s^2)');
legend(component_labels, 'Location', 'best');
title('Accelerometer');

subplot(3,1,2);
plot(t, gyro_data(1,:), 'LineWidth', 1.2, 'Color', component_colors(1,:));
hold on; grid on;
plot(t, gyro_data(2,:), 'LineWidth', 1.2, 'Color', component_colors(2,:));
plot(t, gyro_data(3,:), 'LineWidth', 1.2, 'Color', component_colors(3,:));
ylabel('Gyro (rad/s)');
legend(component_labels, 'Location', 'best');
title('Gyroscope');

subplot(3,1,3);
plot(t, mag_data(1,:), 'LineWidth', 1.2, 'Color', component_colors(1,:));
hold on; grid on;
plot(t, mag_data(2,:), 'LineWidth', 1.2, 'Color', component_colors(2,:));
plot(t, mag_data(3,:), 'LineWidth', 1.2, 'Color', component_colors(3,:));
ylabel('Mag');
xlabel('Time (s)');
legend(component_labels, 'Location', 'best');
title('Magnetometer');
sgtitle('Raw IMU Sensor Measurements');
end
