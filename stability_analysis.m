% Init
clc
clear

% Load params
m = 1.267; % Pendulum mass (kg)
l = 0.159; % Pendulum length (m)
g = 9.81; % Gravitational acceleration (m/s^2)
c = 0.5; % Damping (Ns/m)

% Linearized model
A = [[0.0 1.0]; [g/l -c/m*l^2]];
B = [0.0; -1.0];

% Check stability of the system by calculating eigenvalues and eigenvectors, mention positive eigenvalues (unstable)
[~, evals] = eig(A);
fprintf(['\n', 'System dynamics (positive eigenvalue => instable):'])
fprintf(['\n\tEigenvalues of A: ', num2str(reshape(evals', 1, []))])

% Controllability
fprintf(['\n\t', 'Rank of ctrb(A,B): ', num2str(rank(ctrb(A, B)))])

%% PID control
fprintf(['\n\n', 'PID control (negative/imaginary eigenvalues => stable/oscillating):'])

% Define K from used PID params [Kp_theta Kd_theta]
K = [60, 1];
fprintf(['\n\t', 'K = ', num2str(reshape(K, 1, []))])

% Verification of Eigen values of A-BK
[~, evals] = eig(A - B * K);
fprintf(['\n\t', 'Eigenvalues of A-BK: ', num2str(reshape(evals', 1, []))])

%% Pole placement control
fprintf(['\n\n', 'Pole placement control (negative/real eigenvalues => stable, no oscillation):'])

% Required poles
p = [-20, -10];

% Place poles
K = place(A,B,p);
fprintf(['\n\t', 'K = ', num2str(reshape(K, 1, []))])

% Verification of Eigen values of A-BK
[~, evals] = eig(A - B * K);
fprintf(['\n\t', 'Eigenvalues of A-BK: ', num2str(reshape(evals', 1, []))])

%% LQR control
fprintf(['\n\n', 'LQR control (negative/real eigenvalues => stable, no oscillation):'])

% Control parameters, Q = state penalty, R = control penalty
Q = eye(2);
R = eye(1);

% LQR control, compute optimal control gain
K = lqr(A, B, Q, R);
fprintf(['\n\t', 'K = ', num2str(reshape(K, 1, []))])

% Verification of Eigen values of A-BK
[~, evals] = eig(A - B * K);
fprintf(['\n\t', 'Eigenvalues of A-BK: ', num2str(reshape(evals', 1, []))])