function [dxdt] = linearized_model(~, x, u)

    % Load params
    p = load('params.mat');
    m = p.m; % Pendulum mass
    M = p.M; % Cart mass
    l = p.l; % Pendulum length
    r = p.r; % Wheel radius
    g = p.g; % Gravitation
    
    % Set continuous-time linear state space dynamics form
    A = [[0.0 1.0]; [g/l -c/m*l^2]];
    B = [0.0; -1.0];

    % System matrices
    %A = [[0.0 1.0 0.0 0.0]; [g/l -c 0.0 0.0]; [0.0 0.0 0.0 1.0]; [0.0 0.0 0.0 -c]]; % system matrix
    %B = [0.0; -2*k/(r*R*l*(M+m)); 0.0; 2*k/(r*R*(M+m))]; % control matrix
    
    % Calculate dxdt
    dxdt = A * x + B * u;
    
end