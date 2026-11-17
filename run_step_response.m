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
[t_ode, F_ode, x_ode, v_ode, th_ode_deg] = crane_reference_ode( ...
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

%% Simscape vs ODE: largest difference on the Simscape time grid
x_ode_i = interp1(t_ode, x_ode, t_sm);
v_ode_i = interp1(t_ode, v_ode, t_sm);
th_ode_i = interp1(t_ode, th_ode_deg, t_sm);
fprintf('Max |Simscape - ODE|: x %.2e m, v %.2e m/s, theta %.2e deg\n', ...
    max(abs(x_sm - x_ode_i)), max(abs(v_sm - v_ode_i)), max(abs(th_sm_deg - th_ode_i)));

%% Figures for the README and the carousel
imgDir = fullfile(thisDir, 'images');
if ~exist(imgDir, 'dir')
    mkdir(imgDir);
end
figPos = [100 100 1400 1000];
fontSize = 16;

% Figure 1: Simscape (solid) vs ODE (dashed) — the plant check
fig1 = figure('Name', 'Gantry crane: Simscape vs ODE', 'Color', 'w', 'Position', figPos);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');

nexttile;
plot(t_sm, F_sm, 'b-', 'LineWidth', 2); hold on;
plot(t_ode, F_ode, 'r--', 'LineWidth', 2);
ylabel('F [N]');
grid on;
lg = legend('Simscape', 'ODE');
lg.Layout.Tile = 'east';   % outside the axes, same width for all tiles
title(tl, 'Open-loop gantry crane: force pulse response', 'FontSize', fontSize + 2);

nexttile;
plot(t_sm, v_sm, 'b-', 'LineWidth', 2); hold on;
plot(t_ode, v_ode, 'r--', 'LineWidth', 2);
ylabel('cart velocity [m/s]');
grid on;

nexttile;
plot(t_sm, th_sm_deg, 'b-', 'LineWidth', 2); hold on;
plot(t_ode, th_ode_deg, 'r--', 'LineWidth', 2);
ylabel('\theta [deg]');
xlabel('t [s]');
grid on;

set(findall(fig1, 'Type', 'axes'), 'FontSize', fontSize);
linkaxes(findall(fig1, 'Type', 'axes'), 'x');
xlim([0 t_stop]);
exportgraphics(fig1, fullfile(imgDir, 'simscape_vs_ode.png'), 'Resolution', 150);

% Figure 2: Simscape response only — force, cart position, load angle
fig2 = figure('Name', 'Gantry crane: pulse response', 'Color', 'w', 'Position', figPos);
tl2 = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');

nexttile;
plot(t_sm, F_sm, 'b-', 'LineWidth', 2);
ylabel('F [N]');
grid on;
title(tl2, 'Open-loop gantry crane: force pulse response', 'FontSize', fontSize + 2);

nexttile;
plot(t_sm, x_sm, 'b-', 'LineWidth', 2);
ylabel('cart position x [m]');
grid on;

nexttile;
plot(t_sm, th_sm_deg, 'b-', 'LineWidth', 2);
ylabel('\theta [deg]');
xlabel('t [s]');
grid on;

set(findall(fig2, 'Type', 'axes'), 'FontSize', fontSize);
linkaxes(findall(fig2, 'Type', 'axes'), 'x');
xlim([0 t_stop]);
exportgraphics(fig2, fullfile(imgDir, 'pulse_response.png'), 'Resolution', 150);

fprintf('Exported figures to %s.\nDone.\n', imgDir);
