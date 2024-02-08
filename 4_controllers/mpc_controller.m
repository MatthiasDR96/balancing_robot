function [u] = mpc_controller(x)

    % Load params
    p = load('params.mat');
    A = p.A;
    B = p.B;
    dt = p.dt;
    
    % State space to discrete
    A = eye(4) + A * dt;
    B = B * dt;
    
    % Objective function
    function [J] = obj_func(u, x)
        
        % Get objective values
        Q = eye(4); Q(1,1) = 1000; Q(3,3) = 5000; 
        R = eye(1);

        % Solve ode with given control values
        x_hist_ = zeros(length(x), length(u));
        x_hist_(:,1) = x;
        for i=1:length(u)
             x_hist_(:, i+1) = A * x_hist_(:, i) + B * u(i);
        end
        
        % Compute cost
        J = sum(sum(x_hist_' * Q * x_hist_)) + sum(sum(u' * R * u)) ;     
        
    end

    % Optimize control values
    h = 50; % Optimization horizon
    u0 = zeros(1, h);
    lb = -ones(12, h);
    ub = ones(12, h);
    opts = optimoptions('fmincon','Display','off','Algorithm','sqp');
    opt_u = fmincon(@(u) obj_func(u, x), u0, [], [], [], [], [], [], [], opts);
   
    % Get first control signal
    u = opt_u(1);
    
    % Limit control signal
    if u < 0
        u = max(-p.V_max, u);
    else
        u = min(p.V_max, u);
    end
    
end