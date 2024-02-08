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
pitch_hat_acc_hist = []; 
roll_hat_acc_hist = [];
yaw_hat_acc_hist = [];

% Read values in a loop
figure(1);
while true
    
    % Read sensor
    [accelReadings, timestamp] = readAcceleration(imu);
    gyroReadings = readAngularVelocity(imu);
    magnetoReadings = readMagneticField(imu);

    % Get accelerations (m/s^2)
    Ax = accelReadings(1);
    Ay = accelReadings(2);
    Az = accelReadings(3);

    % Get magnetic field values (uT)
    Mx = magnetoReadings(1);
    My = magnetoReadings(2);
    Mz = magnetoReadings(3);
 
    % Get angles with accelerations (rad)
    pitch_hat_acc = atan2(Ax , sqrt(Ay .^ 2 + Az .^ 2));
    roll_hat_acc = atan2(Ay , sqrt(Ax .^ 2 + Az .^ 2));

    % Yaw
    mag_x = Mx; %* cos(pitch_hat_acc) + My * sin(roll_hat_acc)*sin(pitch_hat_acc) + Mz * cos(roll_hat_acc)*sin(pitch_hat_acc);
    mag_y = My; %* cos(roll_hat_acc) - Mz * sin(roll_hat_acc);
    yaw_hat_acc = 0.0; %atan2(mag_y, mag_x);
    
    % Convert to degrees (deg)
    pitch_hat_acc_deg = pitch_hat_acc * 180.0 / pi;
    roll_hat_acc_deg = roll_hat_acc * 180.0 / pi;
    yaw_hat_acc_deg = yaw_hat_acc * 180.0 / pi;
    
    % Convert to quaternion [w, x, y, z]
    q = eul2quat([roll_hat_acc, -pitch_hat_acc, yaw_hat_acc], 'XYZ');
    
    % Save in hist
    time_hist = [time_hist timestamp];
    pitch_hat_acc_hist = [pitch_hat_acc_hist pitch_hat_acc_deg];
    roll_hat_acc_hist = [roll_hat_acc_hist roll_hat_acc_deg];
    yaw_hat_acc_hist = [yaw_hat_acc_hist yaw_hat_acc_deg];
    
    % Plot angles 
    subplot(1, 2, 1, 'align');
    hold on;
    plot(time_hist, pitch_hat_acc_hist, 'color', 'r')
    plot(time_hist, roll_hat_acc_hist, 'color', 'g')
    plot(time_hist, yaw_hat_acc_hist, 'color', 'b')
    title('Angles [deg]')
    xlabel('Time')
    ylabel('Angles [deg]')
    legend('pitch', 'roll')
    
    % Plot frame
    subplot(1, 2, 2, 'align');
    plotTransforms([0 0 0; 0 0 0],[1 0 0 0; q]);
    title('Rotation visualisation')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid()
    axis([-1 1 -1 1 -1 1])
    pause(eps);
    
end