clc;
clear;
close all;

% Simulation parameters
fs = 100;          % sampling frequency (Hz)
dt = 1/fs;         % sampling interval
t = 0:dt:20;       % time vector (20 s)

% True angle motion: A*sin(2*pi*f*t)
theta_true = 20 * sin(2*pi*0.2*t);   % amplitude = 20 deg, frequency = 0.2 Hz

plot(t, theta_true)
xlabel('Time (seconds)')
ylabel('Angle (degrees)')
title('True Angle Motion')
grid on

% True angular velocity (derivative of angle)
angular_velocity_true = 20 * 2*pi*0.2 * cos(2*pi*0.2*t);

% Gyroscope bias (slow drift)
gyro_bias = 0.5 + 0.01*t;   % deg/s

% Gyroscope measurement noise
gyro_noise = 0.2 * randn(size(t));

% Gyroscope output = true velocity + bias + noise
gyro_output = angular_velocity_true + gyro_bias + gyro_noise;

figure
plot(t, angular_velocity_true, 'b')
hold on
plot(t, gyro_output, 'r')
legend('True Angular Velocity','Gyro Measured Output')
xlabel('Time (seconds)')
ylabel('Angular Velocity (deg/s)')
title('Gyroscope Simulation')
grid on

% Accelerometer noise (higher than gyro)
accel_noise = 1.5 * randn(size(t));

% Accelerometer angle measurement
accel_output = theta_true + accel_noise;

figure
plot(t, theta_true, 'b', 'LineWidth', 1.5)
hold on
plot(t, accel_output, 'g')
legend('True Angle','Accelerometer Output')
xlabel('Time (seconds)')
ylabel('Angle (deg)')
title('Accelerometer Simulation')
grid on

%% Step 4: Individual sensor estimates

% Gyro angle estimation via integration
gyro_angle_estimate = cumsum(gyro_output) * dt;

figure
plot(t, theta_true, 'b', 'LineWidth', 1.5)
hold on
plot(t, gyro_angle_estimate, 'r')
legend('True Angle','Gyro Only Estimate')
xlabel('Time (seconds)')
ylabel('Angle (deg)')
title('Gyro Only Angle Estimation')
grid on

% Accelerometer angle estimate
accel_angle_estimate = accel_output;

figure
plot(t, theta_true, 'b', 'LineWidth', 1.5)
hold on
plot(t, accel_angle_estimate, 'g')
legend('True Angle','Accelerometer Only Estimate')
xlabel('Time (seconds)')
ylabel('Angle (deg)')
title('Accelerometer Only Angle Estimation')
grid on

% Step 5: Complementary filter fusion
alpha = 0.98;   % gyro weight

complementary_angle = zeros(size(t));
complementary_angle(1) = theta_true(1);

for k = 2:length(t)
    gyro_integration = complementary_angle(k-1) + gyro_output(k)*dt;
    complementary_angle(k) = alpha * gyro_integration + ...
                             (1-alpha) * accel_output(k);
end

figure
plot(t, theta_true, 'b', 'LineWidth', 1.5)
hold on
plot(t, complementary_angle, 'm')
legend('True Angle','Complementary Filter Output')
xlabel('Time (seconds)')
ylabel('Angle (deg)')
title('Complementary Filter Estimation')
grid on

% RMSE error comparison
rmse_gyro = sqrt(mean((theta_true - gyro_angle_estimate).^2));
rmse_accel = sqrt(mean((theta_true - accel_angle_estimate).^2));
rmse_complementary = sqrt(mean((theta_true - complementary_angle).^2));

disp(['Gyro RMSE: ', num2str(rmse_gyro)])
disp(['Accel RMSE: ', num2str(rmse_accel)])
disp(['Complementary RMSE: ', num2str(rmse_complementary)])

% FFT analysis
N = length(t);
f = (0:N-1)*(fs/N);

fft_accel = abs(fft(accel_angle_estimate))/N;
fft_complementary = abs(fft(complementary_angle))/N;
fft_true = abs(fft(theta_true))/N;

figure
plot(f, fft_accel, 'r','LineWidth',1.5)
hold on
plot(f, fft_complementary, 'm','LineWidth',1.5)
plot(f, fft_true, 'b', 'LineWidth',1.5)

xlim([0 5])
legend('Accel FFT','Complementary FFT','True FFT')
xlabel('Frequency (Hz)')
ylabel('Magnitude')
title('FFT Comparison')
grid on