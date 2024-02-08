%% Init
clc
clear

% Add paths
addpath('1_system_models')
addpath('4_controllers')

% State sampling
n = 50;

% Define model
model = @dynamic_model;

% Define controller
controller = @constant_controller;

%% Compute differential field

% State limits
xLim = [-2*pi,2*pi];
yLim = [-7,7];

% State arrays
x = linspace(xLim(1),xLim(2),n); % Theta
y = linspace(yLim(1),yLim(2),n); % Theta dot

% State grid
[xx,yy] = ndgrid(x,y);

% Compute derivative for each state
dxx = zeros(length(x), length(y));
dyy = zeros(length(x), length(y));
for i=1:length(x)
    for j=1:length(y)
        dxdt = model(0, [x(i); y(j); 0; 0], controller([x(i); y(j); 0; 0]));
        dxx(i, j) = dxdt(1);
        dyy(i, j) = dxdt(2);
    end
end

%% Compute particular solution

% Example solution
tspan = [0, 10]; % Integration time span [s]
x0 = [0.7;  0.0; 0.0; 0.0]; % Initial state [rad; rad/s]
[t, x] = ode45(@(t, x) model(t, x, controller(x)), tspan, x0);

% Plot
figure(1); 
hold on;
quiver(xx,yy,dxx,dyy);
plot(x(:,1), x(:,2), 'LineWidth', 3)
plot(x(1,1), x(1,2), '.', 'MarkerSize',30)
title('Differential field');
grid on;
xlabel('theta (rad)');
ylabel('dtheta (rad/s)');

