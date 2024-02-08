% Init
clc
clear
close all

% Connect to arduino
a = arduino();

% Connect to imu
imu = lsm9ds1(a,"Bus",1);

% Init hist
time_hist = [];
accel_hist = [];
gyro_hist = [];
magneto_hist = [];

% Read values in a loop
figure(1);
hold on;
while true
    
    % Read sensor
    [accelReadings, timestamp] = readAcceleration(imu); % Measures 9.81 m:s^2 upward
    gyroReadings = readAngularVelocity(imu);
    magnetoReadings = readMagneticField(imu);

    % Add total acceleration
    accelReadings = [accelReadings abs(sum(accelReadings))];
    
    % Save data
    time_hist = [time_hist timestamp];
    accel_hist = [accel_hist; accelReadings];
    gyro_hist = [gyro_hist; gyroReadings];
    magneto_hist = [magneto_hist; magnetoReadings];
    
    % Plot acceleration
    subplot(1, 4, 1)
    hold on;
    plot(time_hist, accel_hist(:,1), 'color', 'r')
    plot(time_hist, accel_hist(:,2), 'color', 'g')
    plot(time_hist, accel_hist(:,3), 'color', 'b')
    title('Linear accelerations [m/s^2]')
    xlabel('Time')
    ylabel('Acceleration [m/s^2]')
    legend('ax', 'ay', 'az')

    % Plot gravitation
    subplot(1, 4, 2)
    hold on;
    plot(time_hist, accel_hist(:,4), 'color', 'k')
    title('Total acceleration [m/s^2]')
    xlabel('Time')
    ylabel('Total acceleration [m/s^2]')
    legend('a')
    
    % Plot angular velocities
    subplot(1, 4, 3)
    hold on;
    plot(time_hist, gyro_hist(:,1), 'color', 'r')
    plot(time_hist, gyro_hist(:,2), 'color', 'g')
    plot(time_hist, gyro_hist(:,3), 'color', 'b')
    title('Angular velocities [deg/s]')
    xlabel('Time')
    ylabel('Angular velocities [deg/s]')
    legend('gx', 'gy', 'gz')

    % Plot magnetic fields
    subplot(1, 4, 4)
    hold on;
    plot(time_hist, magneto_hist(:,1), 'color', 'r')
    plot(time_hist, magneto_hist(:,2), 'color', 'g')
    plot(time_hist, magneto_hist(:,3), 'color', 'b')
    title('Magnetic field [uT]')
    xlabel('Time')
    ylabel('Magnetic field [uT]')
    legend('mx', 'my', 'mz')
    pause(0.01)
    
end