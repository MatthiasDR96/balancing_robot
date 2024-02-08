function [u] = pid_controller(x)

    % Load params
    %p = load('params.mat');
    
    % Optional PID params
    Kp_theta = 40;
    Kd_theta = 5;
    Kp_x = 8;
    Kd_x = 10;
    
    % Get params
    th = x(1); % Current theta
    dth = x(2); % Current angular velocity
    x_ = x(3); % Current theta
    dx = x(4); % Current angular velocity
    
    % Apply constant control signal
    u = th * Kp_theta + dth * Kd_theta + x_ * Kp_x + dx * Kd_x;
    
    % Limit control signal
    %if u < 0
        %u = max(-p.V_max, u);
    %else
        %u = min(p.V_max, u);
    %end
    
end