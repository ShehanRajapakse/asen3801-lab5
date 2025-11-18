%% RunAircraftSim.m
% Example script to call PlotAircraftSim

clear; clc; close all;


% Example simulation data setup

t = linspace(0, 20, 1000)';  % Time vector [s]

% The aircraft initial state and control inputs are at the following trim values

xE1 = 0; yE1 = 0; zE1 = -1800; % meters
phi1 = 0; theta1 = deg2rad(1.59282267); psi1 = 0; % degrees
uE1 = 20.99; vE1 = 0; wE1 = 0.5837; % m/s
p1 = 0; q1 = 0; r1 = 0; % deg/s

x1 = [xE1 yE1 zE1 phi1 theta1 psi1 uE1 vE1 wE1 p1 q1 r1];

elevator1 = 0.1079;
aileron1  = 0;
rudder1   = 0;
throttle1 = 0.3182;

controls1 = [elevator1 aileron1 rudder1 throttle1];

% For initial aircraft state and control surface inputs

xE2 = 0; yE2 = 0; zE2 = -1800; % meters
phi2 = deg2rad(15); theta2 = deg2rad(-12); psi2 = deg2rad(270); % degrees
uE2 = 19; vE2 = 3; wE2 = -2; % m/s
p2 = deg2rad(0.08); q2 = deg2rad(-0.2); r2 = 0; % deg/s

x2 = [xE2 yE2 zE2 phi2 theta2 psi2 uE2 vE2 wE2 p2 q2 r2];

elevator2 = deg2rad(5);
aileron2  = deg2rad(2);
rudder2   = deg2rad(-13);
throttle2 = 0.3;

controls2 = [elevator2 aileron2 rudder2 throttle2];

fig = 1; 
col = 'b'; 

PlotAircraftSim_lab5(t, x1, controls1, fig, col);

