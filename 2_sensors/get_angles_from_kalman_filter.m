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
pitch_hat_kalm_hist = [];
roll_hat_kalm_hist = [];

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
state_estimate = [0 0 0 0]';
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
    
    % System dynamics
    A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];
    B = [dt 0 0 0; 0 0 dt 0]';
    C = [1 0 0 0; 0 0 1 0];
    P = eye(4);
    Q = eye(4);
    R = eye(2);

    % Predict
    pitch_dot = Gy; %cos(pitch_hat_gyr) * Gy - sin(pitch_hat_gyr) * Gz;
    roll_dot = Gx; % + sin(pitch_hat_gyr) * tan(roll_hat_gyr) * Gy + cos(pitch_hat_gyr) * tan(roll_hat_gyr) * Gz;
    state_estimate = A * state_estimate + B * [pitch_dot, roll_dot]';
    P = A * P * A' + Q;
    
    % Get angles with accelerations (rad)
    pitch_hat_acc = atan2(Ax , sqrt(Ay .^ 2 + Az .^ 2));
    roll_hat_acc = atan2(-Ay , sqrt(Ax .^ 2 + Az .^ 2));

    % Update
    measurement = [pitch_hat_acc roll_hat_acc]';
    y_tilde = measurement - C * state_estimate;
    S = R + C * P * C';
    K = P * C' * (S^-1);
    state_estimate = state_estimate + K * y_tilde;
    P = (eye(4) - K * C) * P;
    pitch_hat_kalm = state_estimate(1);
    roll_hat_kalm = state_estimate(3);
    
    % Convert to degrees (deg)
    pitch_hat_kalm_deg = pitch_hat_kalm * 180.0 / pi;
    roll_hat_kalm_deg = roll_hat_kalm * 180.0 / pi;
    
    % Convert to quaternion [w, x, y, z]
    q = eul2quat([roll_hat_kalm, -pitch_hat_kalm, 0.0], 'XYZ');
    
    % Save in hist
    time_hist = [time_hist timestamp];
    pitch_hat_kalm_hist = [pitch_hat_kalm_hist pitch_hat_kalm_deg];
    roll_hat_kalm_hist = [roll_hat_kalm_hist roll_hat_kalm_deg];
    
    % Plot angles
    subplot(1, 2, 1, 'align');
    hold on;
    plot(time_hist, pitch_hat_kalm_hist, 'color', 'r')
    plot(time_hist, roll_hat_kalm_hist, 'color', 'b')
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