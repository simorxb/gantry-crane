%CRANE_PARAMS  Open-loop gantry crane physical and simulation parameters.
%
% Cart-pendulum plant: cart mass M, point-mass load m on a rigid massless
% rope of length L, viscous friction b on the cart, rotational viscous
% friction c at the pivot, gravity g.
%
% The Multibody model may include a very light visualization rod; the ODE
% plant neglects that rod mass.

%% Plant
M = 10;          % cart mass [kg]
m = 2;           % load mass [kg]
L = 2;           % rope length [m]
b = 10;          % cart viscous friction [N*s/m]
c = 0.1;         % pivot rotational viscous friction [N*m*s/rad]
g = 9.81;        % gravity [m/s^2]

%% Force step input (F = F_step for t < t_step, then 0)
F_step = 10;     % force amplitude before the drop [N]
t_step = 3;      % time when force drops to 0 [s]
t_stop = 40;     % simulation stop time [s]

%% Visualization (inertial properties: cart M, load m; rod neglected in ODE)
cart_size = [0.4 0.3 0.3];   % Brick Solid dimensions [m]
load_radius = 0.1;           % Spherical Solid radius [m]
rod_radius = 0.02;           % Cylindrical Solid radius [m]
m_rod = 1e-3;                % visualization rod mass [kg] (neglected in ODE)

%% Model / logging
model_name = 'crane_open_loop';
csv_name = 'crane_step.csv';
