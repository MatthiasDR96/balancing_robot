function [u] = pole_placement_controller(x)

    % Load params
    p = load('params.mat');
    A = p.A;
    B = p.B;

    % Required poles
    poles = [-20, -10, -5, -1];

    % Place poles
    K = place(A,B,poles);
    u = -K * x;
    
    % Limit control signal
    if u < 0
        u = max(-p.V_max, u);
    else
        u = min(p.V_max, u);
    end
    
end