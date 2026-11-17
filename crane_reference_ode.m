function [t, F, x, v, theta_deg, extras] = crane_reference_ode(p)
%CRANE_REFERENCE_ODE  Nonlinear ODE plant for the open-loop gantry crane.
%
% Solves with ode45 (rod mass neglected):
%   (M+m)*xdd + m*L*(thdd*cos(th) - thd^2*sin(th)) = F - b*xd
%   L*thdd + xdd*cos(th) + g*sin(th) + (c/(m*L))*thd = 0
%
% Force profile matches the Simulink Step block: F = F_step for t < t_step,
% then F = 0 (drop to zero).
%
% Sign convention: theta = 0 hangs straight down; theta > 0 means the load
% is on the +x side of the pivot.

    arguments
        p.M (1,1) double = 10
        p.m (1,1) double = 2
        p.L (1,1) double = 2
        p.b (1,1) double = 20
        p.c (1,1) double = 0.5
        p.g (1,1) double = 9.81
        p.F_step (1,1) double = 10
        p.t_step (1,1) double = 1
        p.t_stop (1,1) double = 40
    end

    M = p.M; m = p.m; L = p.L; b = p.b; c = p.c; g = p.g;
    F_step = p.F_step; t_step = p.t_step; t_stop = p.t_stop;

    opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-12);
    odefun = @(t, y) localRhs(t, y, M, m, L, b, c, g, F_step, t_step);
    [t, y] = ode45(odefun, [0 t_stop], [0; 0; 0; 0], opts);

    x = y(:, 1);
    v = y(:, 2);
    theta = y(:, 3);
    theta_deg = theta * (180/pi);
    F = F_step * double(t < t_step);

    extras = struct();
    extras.omega = y(:, 4);
    extras.A_lin = localLinearA(M, m, L, b, c, g);
    extras.eigs = eig(extras.A_lin);
end

function dydt = localRhs(t, y, M, m, L, b, c, g, F_step, t_step)
    xd = y(2);
    th = y(3);
    thd = y(4);
    F = F_step * double(t < t_step);

    % Unknowns: [xdd; thdd]
    A = [(M + m), m * L * cos(th); ...
         cos(th), L];
    rhs = [F - b * xd + m * L * thd^2 * sin(th); ...
           -g * sin(th) - (c / (m * L)) * thd];
    acc = A \ rhs;

    dydt = [xd; acc(1); thd; acc(2)];
end

function A = localLinearA(M, m, L, b, c, g)
    % Linearization about x=xd=th=thd=0 (rod neglected).
    A = [0, 1, 0, 0; ...
         0, -b/M, m*g/M, c/(M*L); ...
         0, 0, 0, 1; ...
         0, b/(M*L), -(g/L)*(1 + m/M), -(c/L^2)*(1/M + 1/m)];
end
