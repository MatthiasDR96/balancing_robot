%% Init
clc
clear
close all
    
% Add paths
addpath('1_system_models')
addpath('4_controllers')

%% Solve ode

% Load params
l = 0.159; % Pendulum length (m)

% Define model
model = @dynamic_model;

% Define controller
controller = @pid_controller;

% Define solve params
tspan = [0, 5]; % Seconds
x0 = [0.1;  0.0; 0.0; 0.0]; % [theta; theta_dot, x, xdot]

% Solve ode
[t, x] = ode45(@(t, x) model(t, x, controller(x)), tspan, x0);

%% Animate
%filename = ['videos/simulation_output_controller.avi'];
%v = VideoWriter(filename,'Motion JPEG AVI');
%v.Quality = 95;
%open(v)
u_hist = zeros(length(t), 1);
figure(1);
pbaspect([1 1 1]);
for i=1:length(t)
    
    % Plot pendulum
    plot([x(i,3) x(i,3)+l*sin(x(i,1))], [0.0 l*cos(x(i,1))],'LineWidth',2);
    axis([-2*l 2*l -2*l 2*l]);
    grid on;
    
    % Title
    title(['Pendulum state at t=', num2str(round(t(i), 0)), 's']);
    
    % Control signal
    u_hist(i) = controller(x(i, :)');
    
    % Loop plot
    drawnow;
    pause(0.001);

    % Capture
    %frame = getframe(gcf);
    %writeVideo(v,frame);
    
end

% Save data
%inputdata = [t x];
%outputdata  = [t u_hist];
%inputdata(:, 2:end) = awgn(inputdata(:, 2:end), 30);
%save('inputdata.mat', 'inputdata')
%save('outputdata.mat', 'outputdata')

% Plot
figure(1);
subplot(3,1,1); 
plot(t,x(:,1));
xlabel('Time (s)'); 
ylabel('theta (rad)');
title('Angle');
grid on;
subplot(3,1,2); 
plot(t,x(:,2));
xlabel('Time (s)'); 
ylabel('dtheta (rad/s)');
title('Angular velocity')
grid on;
subplot(3,1,3);
plot(t, u_hist);
axis([0, t(end), -12.0, 12.0]);
xlabel('Time (s)');
ylabel('u [V]');
title("Control signal");
grid on;
