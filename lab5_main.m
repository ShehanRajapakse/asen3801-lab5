%% RunAircraftSim.m
% Example script to call PlotAircraftSim

clear; clc; close all;

% 
% Example simulation data setup
% 
t = linspace(0, 20, 1000)';  % Time vector [s]

% Example aircraft states (12 states)
% [xE, yE, zE, phi, theta, psi, uE, vE, wE, p, q, r]
xE = 50 * sin(0.1*t);
yE = 50 * cos(0.1*t);
zE = -10 * sin(0.05*t);
phi = 0.05 * sin(0.5*t);
theta = 0.03 * sin(0.4*t);
psi = 0.1 * t;
uE = 15 + 0.5 * sin(0.3*t);
vE = 0.2 * sin(0.6*t);
wE = 0.3 * cos(0.3*t);
p = 0.1 * sin(0.8*t);
q = 0.05 * cos(0.9*t);
r = 0.08 * sin(0.7*t);

x = [xE yE zE phi theta psi uE vE wE p q r];

% 
% Example control input setup
% 
% Control inputs: [elevator, aileron, rudder, throttle]
elevator = deg2rad(2 * sin(0.5*t));  % small deflection in radians
aileron  = deg2rad(3 * cos(0.4*t));
rudder   = deg2rad(1 * sin(0.7*t));
throttle = 0.6 + 0.2*sin(0.3*t);     % varies between 0.4–0.8

cInp = [elevator aileron rudder throttle];

% 
% Call plotting function
% 
fig = 1;           % Figure handle 
col = 'b';         % Line color/style

PlotAircraftSim_lab5(t, x, cInp, fig, col);

