function [dxdt] = dynamic_model(~, x, u)

    % Load params
    m = 1.267; % Pendulum mass (kg)
    M = 0.558; % Cart mass (kg)
    l = 0.159; % Pendulum length (m)
    r = 0.033; % Wheel radius (m)
    g = 9.81; % Gravitational acceleration (m/s^2)
    c = 0.5; % Damping (Ns/m)

    % Calculate derivatives (pendulum has theta zero in upward position and
    % has positive theta clockwise)
    ddx = (u/(r*(M+m))) - c*x(4);
    ddth = (g / l) * sin(x(1)) - (ddx / l) * cos(x(1)) - c*x(2);

    % Calculate dxdt
    dxdt = zeros(4, 1);
	dxdt(1) = x(2);
	dxdt(2) = ddth;
    dxdt(3) = x(4);
    dxdt(4) = ddx;

end