% %% RunAircraftSim.m
% % Example script to call PlotAircraftSim
% 
% clear; clc; close all;
% 
% % 
% % Example simulation data setup
% % 
% t = linspace(0, 20, 1000)';  % Time vector [s]
% 
% % Example aircraft states (12 states)
% % [xE, yE, zE, phi, theta, psi, uE, vE, wE, p, q, r]
% xE = 50 * sin(0.1*t);
% yE = 50 * cos(0.1*t);
% zE = -10 * sin(0.05*t);
% phi = 0.05 * sin(0.5*t);
% theta = 0.03 * sin(0.4*t);
% psi = 0.1 * t;
% uE = 15 + 0.5 * sin(0.3*t);
% vE = 0.2 * sin(0.6*t);
% wE = 0.3 * cos(0.3*t);
% p = 0.1 * sin(0.8*t);
% q = 0.05 * cos(0.9*t);
% r = 0.08 * sin(0.7*t);
% 
% x = [xE yE zE phi theta psi uE vE wE p q r];
% 
% % 
% % Example control input setup
% % 
% % Control inputs: [elevator, aileron, rudder, throttle]
% elevator = deg2rad(2 * sin(0.5*t));  % small deflection in radians
% aileron  = deg2rad(3 * cos(0.4*t));
% rudder   = deg2rad(1 * sin(0.7*t));
% throttle = 0.6 + 0.2*sin(0.3*t);     % varies between 0.4–0.8
% 
% cInp = [elevator aileron rudder throttle];
% 
% % 
% % Call plotting function
% % 
% fig = 1;           % Figure handle 
% col = 'b';         % Line color/style
% 
% PlotAircraftSim_lab5(t, x, cInp, fig, col);
% 
% 
% 
%% main_AircraftSim.m
% Main script to simulate aircraft motion using AircraftEOM
% and visualize results with PlotAircraftSim

clear; clc; close all;

%% --- Simulation setup ---
tspan = [0 20];   % simulation time [s]

% No wind for now
wind_inertial = [0; 0; 0];

%% --- Aircraft parameters (example placeholders; update with your data) ---
aircraft_parameters.m  = 13.5;      % mass [kg]
aircraft_parameters.g  = 9.81;      % gravity [m/s^2]
aircraft_parameters.Ix = 0.8244;    % moment of inertia [kg*m^2]
aircraft_parameters.Iy = 1.135;     
aircraft_parameters.Iz = 1.759;     
aircraft_parameters.Ixz = 0.1204;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aircraft geometry parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.S = 0.6282; %[m^2]
aircraft_parameters.b = 3.067; %[m]
aircraft_parameters.c = 0.208; %[m]
aircraft_parameters.AR = aircraft_parameters.b*aircraft_parameters.b/aircraft_parameters.S;

aircraft_parameters.m = 5.74; %[kg]
aircraft_parameters.W = aircraft_parameters.m*aircraft_parameters.g; %[N]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inertias from Solidworks model of Tempest
% These need to be validated, especially for Ttwistor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SLUGFT2_TO_KGM2 = 14.5939/(3.2804*3.2804);
aircraft_parameters.Ix = SLUGFT2_TO_KGM2*4106/12^2/32.2; %[kg m^2]
aircraft_parameters.Iy = SLUGFT2_TO_KGM2*3186/12^2/32.2; %[kg m^2]
aircraft_parameters.Iz = SLUGFT2_TO_KGM2*7089/12^2/32.2; %[kg m^2]
aircraft_parameters.Ixz = SLUGFT2_TO_KGM2*323.5/12^2/32.2; %[kg m^2]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drag terms determined by curve fit to CFD analysis performed by Roger
% Laurence. Assumes general aircraft drag model
%       CD = CDmin + K(CL-CLmin)^2
% or equivalently
%       CD = CD0 + K1*CL + K*CL^2
% where
%       CD0 = CDmin + K*CLmin^2
%       K1  = -2K*CLmin
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CDmin = 0.0240;
aircraft_parameters.CLmin = 0.2052;
aircraft_parameters.K = 0.0549;
aircraft_parameters.e = 1/(aircraft_parameters.K*aircraft_parameters.AR*pi);

aircraft_parameters.CD0 = aircraft_parameters.CDmin+aircraft_parameters.K*aircraft_parameters.CLmin*aircraft_parameters.CLmin;
aircraft_parameters.K1 = -2*aircraft_parameters.K*aircraft_parameters.CLmin;
aircraft_parameters.CDpa = aircraft_parameters.CD0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Engine parameters, assuming model from Beard and Mclain that gives zero
% thrust for zero throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.Sprop = 0.0707;
aircraft_parameters.Cprop = 1;
aircraft_parameters.kmotor = 30;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Zero angle of attack aerodynamic forces and moments
% - some sources (like text used for ASEN 3128) define the body
%   coordinate system as the one that gives zero total lift at 
%   zero angle of attack
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CL0 = 0.2219;
aircraft_parameters.Cm0 = 0.0519;

aircraft_parameters.CY0 = 0;
aircraft_parameters.Cl0 = 0;
aircraft_parameters.Cn0 = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longtidunal nondimensional stability derivatives from AVL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CLalpha = 6.196683; 
aircraft_parameters.Cmalpha = -1.634010; 
aircraft_parameters.CLq = 10.137584; 
aircraft_parameters.Cmq = -24.376066;

% Neglected parameters, check units below if incorporated later
aircraft_parameters.CLalphadot = 0; 
aircraft_parameters.Cmalphadot = 0; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lateral-directional nondimensional stability derivatives from AVL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_parameters.CYbeta = -0.367231; 
aircraft_parameters.Clbeta = -0.080738; 
aircraft_parameters.Cnbeta = 0.080613; 
aircraft_parameters.CYp = -0.064992;
aircraft_parameters.Clp = -0.686618;
aircraft_parameters.Cnp = -0.039384;
aircraft_parameters.Clr = 0.119718;
aircraft_parameters.Cnr = -0.052324;
aircraft_parameters.CYr = 0.213412;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control surface deflection parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Elevator
  aircraft_parameters.CLde =   0.006776;
  aircraft_parameters.Cmde =  -0.06; 

  % Aileron
  aircraft_parameters.CYda =  -0.000754;
  aircraft_parameters.Clda =  -0.02; 
  aircraft_parameters.Cnda =  -0.000078;
 
  % Rudder
  aircraft_parameters.CYdr =   0.003056;
  aircraft_parameters.Cldr =   0.000157;
  aircraft_parameters.Cndr =  -0.000856;

  

%% --- Initial Conditions (Trim and Disturbed) ---
% Trim condition (steady level flight)
x0_trim = [ ...
    0; 0; -1800; ...                  % position [m]
    0; 0.02780; 0; ...                % Euler angles [rad]
    20.99; 0; 0.5837; ...             % velocity [m/s]
    0; 0; 0 ];                        % angular rates [rad/s]

aircraft_surface = rad2deg([ ...
    0.1079; 0; 0; 0.3182 ]);           % [elevator, aileron, rudder, throttle]

% Disturbed initial condition
x0_dist = [ ...
    0; 0; -1800; ...
    deg2rad(15); deg2rad(-12); deg2rad(270); ...
    19; 3; -2; ...
    deg2rad(0.08); deg2rad(-0.2); 0 ];

u0_dist = [ ...
    deg2rad(5); deg2rad(2); deg2rad(-13); 0.3 ];

%% --- Simulation #1: Trim Condition ---
[t_trim, x_trim] = ode45(@(t,x) AircraftEOM(t, x, aircraft_surface, wind_inertial, aircraft_parameters), tspan, x0_trim);

% Save control history (constant here)
cInp_trim = repmat(aircraft_surface', length(t_trim), 1);

%% --- Simulation #2: Disturbed Condition ---
[t_dist, x_dist] = ode45(@(t,x) AircraftEOM(t, x, u0_dist, wind_inertial, aircraft_parameters), tspan, x0_dist);

% Save control history (constant here)
cInp_dist = repmat(u0_dist', length(t_dist), 1);

%% --- Plot Results ---
% Plot trim case
PlotAircraftSim_lab5(t_trim, x_trim, cInp_trim, 1, 'b');

% Plot disturbed case (overlayed for comparison)
PlotAircraftSim_lab5(t_dist, x_dist, cInp_dist, 1, 'r--');

legend('Trim Case', 'Disturbed Case');


doublet_time = 0.25;
doublet_size = 15;
tspan_doub = [0 3];

[t_dist_doub, x_dist_doub] = ode45(@(t,x) AircraftEOMDoublet(t, x, aircraft_surface,doublet_size,doublet_time, wind_inertial, aircraft_parameters), tspan_doub, x0_trim);

cInp_doub = repmat(aircraft_surface', length(t_dist_doub), 1);

PlotAircraftSim_lab5(t_dist_doub, x_dist_doub, cInp_doub, 1, 'r--');