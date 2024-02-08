function [u] = lqr_controller(x)

    % Load params
    p = load('params.mat');
    A = p.A; % State matrix
    B = p.B; % Actuator matrix

    % Control parameters, Q = state penalty, R = control penalty
    Q = eye(4); Q(1,1) = 1000; Q(3,3) = 5000; 
    R = eye(1);

    % LQR control, compute optimal control gain
    
    K = lqr(A, B, Q, R);

    % Compute control signal
    u = -K * x;
    
    % Limit control signal
    if u < 0
        u = max(-p.V_max, u);
    else
        u = min(p.V_max, u);
    end
    
end