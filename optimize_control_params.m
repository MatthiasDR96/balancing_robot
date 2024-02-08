%% Init
clc
clear
close all
    
% Add paths
addpath('1_system_models')
addpath('4_controllers')

% Define model
model = @dynamic_model_simplified;

% Define controller
controller = @pid_controller;

% Optimization params
nvars = 2; % Amount of control parameters in the control algorithm
lb = [0, 0]; % Lower bounds, length needs to equal nvars
ub = [60, 60]; % Upper bounds, length needs to equal nvars

% Optimize
sol = optimize(model, controller, nvars, lb, ub);
disp(['Optimal control parameters: ', num2str(sol)]);

function [best_params] = optimize(model, control, nvars, lb, ub)

    % GA solver
    options = optimoptions('ga','ConstraintTolerance',1e-6,'PlotFcn', ...
        {@gaplotbestf,@gaplotstopping}, 'MaxGenerations', 20, 'MaxStallGenerations', ...
        5, 'PopulationSize', 10);
    best_params = ga(@(p) obj_fun(p, model, control), nvars,[],[],[],[],lb,ub,[],[],options);

end

function [error] = obj_fun(params, model, controller)

    % Simulate
    tspan = [0, 5]; % Seconds
    x0 = [0.5;  0.0; 0.0; 0.0]; % [theta; theta_dot]
    [~, x] = ode45(@(t, x) model(t, x, controller(x, params)), tspan, x0);
    
    % Fitness
    x_err = norm(x(1, :));
    error = x_err;

end
