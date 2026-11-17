%RUN_STEP_RESPONSE  Simulate open-loop Multibody crane vs ODE reference.
%
% Uses the existing crane_open_loop.slx (edit that model with the agentic
% toolkit / model_edit — this script does not rebuild it). Force profile:
% F = F_step until t_step, then 0. Compares against crane_reference_ode
% (rod neglected), plots Simscape vs ODE, and writes crane_step.csv.

clearvars;
close all;
clc;

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);
addpath(thisDir);

crane_params;

if ~exist([model_name '.slx'], 'file')
    error('Model %s.slx not found. Open/create it before running this script.', model_name);
end

%% Reference ODE (neglects visualization rod)
[t_ode, F_ode, ~, v_ode, th_ode_deg] = crane_reference_ode( ...
    M=M, m=m, L=L, b=b, c=c, g=g, F_step=F_step, t_step=t_step, t_stop=t_stop);

%% Simscape Multibody simulation
assignin('base', 'M', M);
assignin('base', 'm', m);
assignin('base', 'L', L);
assignin('base', 'b', b);
assignin('base', 'c', c);
assignin('base', 'g', g);
assignin('base', 'F_step', F_step);
assignin('base', 't_step', t_step);
assignin('base', 'cart_size', cart_size);
assignin('base', 'load_radius', load_radius);
assignin('base', 'rod_radius', rod_radius);
assignin('base', 'm_rod', m_rod);

open_system(model_name);
set_param(model_name, 'StopTime', num2str(t_stop));
set_param(model_name, 'SimMechanicsOpenEditorOnUpdate', 'on');

fprintf('Simulating Multibody model "%s"...\n', model_name);
simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');

t_sm = simOut.x_ts.Time;
x_sm = simOut.x_ts.Data(:);
v_sm = simOut.v_ts.Data(:);
th_sm_deg = simOut.theta_ts.Data(:);
F_sm = simOut.F_ts.Data(:);

%% Export CSV (Simscape): t, F, x, v, theta_deg
Tcsv = table(t_sm, F_sm, x_sm, v_sm, th_sm_deg, ...
    'VariableNames', {'t', 'F', 'x', 'v', 'theta_deg'});
writetable(Tcsv, csv_name);
fprintf('Wrote %s (%d samples).\n', csv_name, height(Tcsv));

%% Plot: F, cart velocity, theta — Simscape solid, ODE dashed
figure('Name', 'Gantry crane open-loop step response', 'Color', 'w');
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t_sm, F_sm, 'b-', 'LineWidth', 1.2); hold on;
plot(t_ode, F_ode, 'r--', 'LineWidth', 1.0);
ylabel('F [N]');
grid on;
legend('Simscape', 'ODE', 'Location', 'best');
title(tl, 'Open-loop gantry crane: force step response');

nexttile;
plot(t_sm, v_sm, 'b-', 'LineWidth', 1.2); hold on;
plot(t_ode, v_ode, 'r--', 'LineWidth', 1.0);
ylabel('cart velocity [m/s]');
grid on;

nexttile;
plot(t_sm, th_sm_deg, 'b-', 'LineWidth', 1.2); hold on;
plot(t_ode, th_ode_deg, 'r--', 'LineWidth', 1.0);
ylabel('\theta [deg]');
xlabel('t [s]');
grid on;

linkaxes(findall(gcf, 'Type', 'axes'), 'x');
xlim([0 t_stop]);

fprintf('Done.\n');
