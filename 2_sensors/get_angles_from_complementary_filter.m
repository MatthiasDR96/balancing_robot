% Init
clc
clear
close all

% Connect to arduino
a = arduino();

% Connect to imu
imu = lsm9ds1(a,"Bus",1);
%imu = mpu6050(a, 'SampleRate',100,'SamplesPerRead',1,'ReadMode','Latest');

% Init hist
time_hist = [];
pitch_hat_comp_hist = [];
yaw_hat_comp_hist = [];
roll_hat_comp_hist = [];

% Calibrate gyro
N = 1000;
bxg = 0.0;
byg = 0.0;
bzg = 0.0;
fprintf("Start calibration...")
for i=1:length(N)
    
    % Read sensor
    [gyroReadings,timestamp] = readAngularVelocity(imu);

    % Get angular velocities (rad/s)
    Gx = gyroReadings(1);
    Gy = gyroReadings(2);
    Gz = gyroReadings(3);
    bxg = bxg + Gx;
    byg = byg + Gy;
    bzg = bzg + Gz;
    pause(0.1)
    
end
bxg = bxg/N;
byg = byg/N;
bzg = bzg/N;
fprintf(['\n', 'Calibration done.'])

% Read values in a loop
figure(1);
start_time = datetime('now');
pitch_hat_gyr = 0.0;
roll_hat_gyr = 0.0;
while true
    
    % Read sensor
    [accelReadings, timestamp] = readAcceleration(imu);
    gyroReadings = readAngularVelocity(imu);
    magnetoReadings = readMagneticField(imu);
    
    % Get accelerations (m/s^2)
    Ax = accelReadings(1);
    Ay = accelReadings(2);
    Az = accelReadings(3);
    
    % Get angular velocities (rad/s)
    Gx = gyroReadings(1) - bxg;
    Gy = gyroReadings(2) - byg;
    Gz = gyroReadings(3) - bzg;

    % Get magnetic field values (uT)
    Mx = magnetoReadings(1);
    My = magnetoReadings(2);
    Mz = magnetoReadings(3);
    
    % Calculate time difference (s)
    dt = seconds(diff([start_time, datetime('now')]));
    start_time = datetime('now');
    
    % Get angles with accelerations (rad)
    pitch_hat_acc = atan2(Ax , sqrt(Ay .^ 2 + Az .^ 2));
    roll_hat_acc = atan2(-Ay , sqrt(Ax .^ 2 + Az .^ 2));

    % Get angular velocities in the inertial frame (rad/s)
    pitch_dot = cos(pitch_hat_gyr) * Gy - sin(pitch_hat_gyr) * Gz;
    roll_dot = Gx + sin(pitch_hat_gyr) * tan(roll_hat_gyr) * Gy + cos(pitch_hat_gyr) * tan(roll_hat_gyr) * Gz;
    
    % Get angles using integration of angular velocities (rad)
    pitch_hat_gyr = pitch_hat_gyr + dt * pitch_dot;
    roll_hat_gyr = roll_hat_gyr + dt * roll_dot;
 
    % Complementary estimation (rad)
    alpha = 0.5; % If alpha = 1 -> pure acceleration, if alpha = 0 -> pure gyroscope
    pitch_hat_comp   = (1 - alpha) * pitch_hat_gyr + alpha * pitch_hat_acc;
    roll_hat_comp = (1 - alpha) * roll_hat_gyr + alpha * roll_hat_acc;

    % Yaw
    %mag_x = Mx * cos(pitch_hat_comp) + My * sin(roll_hat_comp)*sin(pitch_hat_comp) + Mz * cos(roll_hat_comp)*sin(pitch_hat_comp);
    %mag_y = My * cos(roll_hat_comp) - Mz * sin(roll_hat_comp);
    yaw_hat_comp = 0.0; %atan2(-mag_y,mag_x);
    
    % Convert to degrees (deg)
    pitch_hat_comp_deg = pitch_hat_comp * 180.0 / pi;
    roll_hat_comp_deg = roll_hat_comp * 180.0 / pi;
    yaw_hat_comp_deg = yaw_hat_comp * 180.0 / pi;
    
    % Convert to quaternion [w, x, y, z]
    q = eul2quat([roll_hat_comp, -pitch_hat_comp, yaw_hat_comp], 'XYZ');
    
    % Save in hist
    time_hist = [time_hist timestamp];
    pitch_hat_comp_hist = [pitch_hat_comp_hist pitch_hat_comp_deg];
    yaw_hat_comp_hist = [yaw_hat_comp_hist yaw_hat_comp_deg];
    roll_hat_comp_hist = [roll_hat_comp_hist roll_hat_comp_deg];
    
    % Plot angles
    subplot(1, 2, 1, 'align');
    hold on;
    plot(time_hist, pitch_hat_comp_hist, 'color', 'r')
    plot(time_hist, yaw_hat_comp_hist, 'color', 'g')
    plot(time_hist, roll_hat_comp_hist, 'color', 'b')
    title('Angles [deg]')
    xlabel('Time')
    ylabel('Angles [deg]')
    legend('pitch', 'yaw', 'roll')
    
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