% Creation Date: 11/1/2025
% Assignment: 3801 Lab 4

clc
clear
close all

%{ 
Plot Numbering Scheme
    1st Digit = Task
    2nd Digit = 0
    3rd Digit = Question #
    4th Digit = Subquestion # (0 if none)
    5th Digit = 0
    6th Digit = Figure #
%}

%% Constants
g = 9.81; % m/s^2

m = 0.068; % kg
d = 0.060; % m
km = 0.0024; % Nm/N
I = [5.8E-5, 0,      0;
     0,      7.2E-5, 0;
     0,      0,      1E-4]; % kg*m^2
nu = 1E-3; % N/(m/s)^2
mu = 2E-6; % N*m/(rad/s)^2


%% Task 1

%% Part 2-3
% Motor forces at trim
motor_forces = g.*m./4 .* ones(1,4); % N
timespan = [0,10]; % seconds
init_cond = zeros(12,1);

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);

figure_numbering = [103001;103002;103003;103004;103005;103006];
PlotAircraftSim(t, var', [ones(1,41)*-sum(motor_forces);zeros(1,41);zeros(1,41);zeros(1,41)], figure_numbering, "b-")

%% Part 4a
% Motor forces at trim for constant velocity at 5 m/s East, 0 yaw
phi1_4_a_func = @(x) -nu*5*5*cos(x)./m+g*sin(x);
phi1_4_a = fzero(phi1_4_a_func,0); 
motor_forces = (nu.*5.*5.*sin(phi1_4_a)+m.*g.*cos(phi1_4_a))./4 .* ones(1,4); % N

timespan = [0,10]; % seconds

% Initial conditions pulled from result of FBD calculation
init_cond = zeros(12,1) + [0;0;0;phi1_4_a;0;0;0;5*cos(phi1_4_a);-5*sin(phi1_4_a);0;0;0];

% Call nonlinear EOM simulation
[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);

% Plot figures
figure_numbering = [104101;104102;104103;104104;104105;104106];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "b-")

%% Part 4b
% Motor forces at trim for constant velocity at 5 m/s East, 90 yaw
theta1_4_b_func = @(x) nu*5*-5*cos(x)./m+g*sin(x);
theta1_4_b = fzero(theta1_4_b_func,0); 
motor_forces = (nu.*5.*5.*sin(theta1_4_b)+m.*g.*cos(theta1_4_b))./4 .* ones(1,4); % N

timespan = [0,10]; % seconds

% Initial conditions pulled from result of FBD calculation
init_cond = zeros(12,1) + [0;0;0;0;-theta1_4_b;deg2rad(90);5*cos(theta1_4_b);0;-5*sin(theta1_4_b);0;0;0];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);

figure_numbering = [104201;104202;104203;104204;104205;104206];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "b-")

%% Task 2

%% Part 1-2
% Motor forces are = force of gravity
motor_forces = g.*m./4 .* ones(1,4); % N

timespan = [0,10]; % seconds

% Set control forces and moments to 0
deltaFc = zeros(3,1);
deltaGc = zeros(3,1);

% Part 1a
% 5 deg roll
init_cond = [0;0;0;deg2rad(5);0;0;0;0;0;0;0;0];

% Call linear and nonlinear EOM simulation
[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc), timespan, init_cond);

% Plot figures
figure_numbering = [201101;201102;201103;201104;201105;201106];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [ones(1,length(tL))*-sum(motor_forces);zeros(1,length(tL));zeros(1,length(tL));zeros(1,length(tL))], figure_numbering, "g-")

% Part 1b
% 5 deg pitch
init_cond = [0;0;0;0;deg2rad(5);0;0;0;0;0;0;0];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc), timespan, init_cond);
figure_numbering = [201201;201202;201203;201204;201205;201206];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [ones(1,length(tL))*-sum(motor_forces);zeros(1,length(tL));zeros(1,length(tL));zeros(1,length(tL))], figure_numbering, "g-")

% Part 1c
% 5 deg yaw
init_cond = [0;0;0;0;0;deg2rad(5);0;0;0;0;0;0];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc), timespan, init_cond);
figure_numbering = [201301;201302;201303;201304;201305;201306];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [ones(1,length(tL))*-sum(motor_forces);zeros(1,length(tL));zeros(1,length(tL));zeros(1,length(tL))], figure_numbering, "g-")

% Part 1d
% 0.1 rad/sec roll
init_cond = [0;0;0;0;0;0;0;0;0;0.1;0;0];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc), timespan, init_cond);
figure_numbering = [201401;201402;201403;201404;201405;201406];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [ones(1,length(tL))*-sum(motor_forces);zeros(1,length(tL));zeros(1,length(tL));zeros(1,length(tL))], figure_numbering, "g-")

% Part 1e
% 0.1 rad/sec pitch
init_cond = [0;0;0;0;0;0;0;0;0;0;0.1;0];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc), timespan, init_cond);
figure_numbering = [201501;201502;201503;201504;201505;201506];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [ones(1,length(tL))*-sum(motor_forces);zeros(1,length(tL));zeros(1,length(tL));zeros(1,length(tL))], figure_numbering, "g-")

% Part 1f
% 0.1 rad/sec yaw
init_cond = [0;0;0;0;0;0;0;0;0;0;0;0.1];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc), timespan, init_cond);
figure_numbering = [201601;201602;201603;201604;201605;201606];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [ones(1,length(tL))*-sum(motor_forces);zeros(1,length(tL));zeros(1,length(tL));zeros(1,length(tL))], figure_numbering, "g-")

%% Part 3
% Motor forces are = force of gravity
motor_forces = g.*m./4 .* ones(1,4); % N

timespan = [0,10]; % seconds

% Part 3d
% 0.1 rad/sec roll
init_cond = [0;0;0;0;0;0;0;0;0;0.1;0;0];

% Call nonlinear EOM simulation with and without set rate feedback
[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tC, varC] = ode45(@(t, var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu), timespan, init_cond);

% Recalculate control forces and moments from rate feedback function
Fc = zeros(3, length(varC));
Gc = zeros(3, length(varC));
for i = 1:length(varC)
    [Fc(:,i), Gc(:,i)] = RotationDerivativeFeedback(varC(i,:), m, g);
end

% Plot figures
figure_numbering = [203101;203102;203103;203104;203105;203106];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "m-")
PlotAircraftSim(tC, varC', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "c-")

% Part 3e
% 0.1 rad/sec pitch
init_cond = [0;0;0;0;0;0;0;0;0;0;0.1;0];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tC, varC] = ode45(@(t, var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu), timespan, init_cond);

Fc = zeros(3, length(varC));
Gc = zeros(3, length(varC));
for i = 1:length(varC)
    [Fc(:,i), Gc(:,i)] = RotationDerivativeFeedback(varC(i,:), m, g);
end

figure_numbering = [203201;203202;203203;203204;203205;203206];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "m-")
PlotAircraftSim(tC, varC', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "c-")

% Part 3f
% 0.1 rad/sec yaw
init_cond = [0;0;0;0;0;0;0;0;0;0;0;0.1];

[t, var] = ode45(@(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), timespan, init_cond);
[tC, varC] = ode45(@(t, var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu), timespan, init_cond);

Fc = zeros(3, length(varC));
Gc = zeros(3, length(varC));
for i = 1:length(varC)
    [Fc(:,i), Gc(:,i)] = RotationDerivativeFeedback(varC(i,:), m, g);
end

figure_numbering = [203301;203302;203303;203304;203305;203306];
PlotAircraftSim(t, var', [ones(1,length(t))*-sum(motor_forces);zeros(1,length(t));zeros(1,length(t));zeros(1,length(t))], figure_numbering, "m-")
PlotAircraftSim(tC, varC', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "c-")

%% Task 3

%% Part 1
% Calculate K Values
k1Long = -22.*I(2,2);
k2Long = 40.*I(2,2);

k1Lat = -22.*I(1,1);
k2Lat = 40.*I(1,1);

%% Part 3-4
timespan = [0,10]; % seconds

% Part 3a
% 5 deg roll
init_cond = [0;0;0;deg2rad(5);0;0;0;0;0;0;0;0];

% Call linear and nonlinear EOM simulation with gains for Euler angles and rates
[t, var] = ode45(@(t, var) QuadrotorEOMwithInnerControl(t, var, g, m, I, nu, mu), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOMwithInnerControl_Linearized(t, var, g, m, I), timespan, init_cond);

% Recalculate control forces and moments from InnerLoopFeedback function
Fc = zeros(3, length(var));
Gc = zeros(3, length(var));
for i = 1:length(var)
    [Fc(:,i), Gc(:,i)] = InnerLoopFeedback(var(i,:));
end
FcL = zeros(3, length(varL));
GcL = zeros(3, length(varL));
for i = 1:length(varL)
    [FcL(:,i), GcL(:,i)] = InnerLoopFeedback(varL(i,:));
end

% Plot figures
figure_numbering = [303101;303102;303103;303104;303105;303106];
PlotAircraftSim(t, var', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [FcL(3,:);GcL(1,:);GcL(2,:);GcL(3,:)], figure_numbering, "b-")

% Part 3b
% 5 deg pitch
init_cond = [0;0;0;0;deg2rad(5);0;0;0;0;0;0;0];

[t, var] = ode45(@(t, var) QuadrotorEOMwithInnerControl(t, var, g, m, I, nu, mu), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOMwithInnerControl_Linearized(t, var, g, m, I), timespan, init_cond);

Fc = zeros(3, length(var));
Gc = zeros(3, length(var));
for i = 1:length(var)
    [Fc(:,i), Gc(:,i)] = InnerLoopFeedback(var(i,:));
end
FcL = zeros(3, length(varL));
GcL = zeros(3, length(varL));
for i = 1:length(varL)
    [FcL(:,i), GcL(:,i)] = InnerLoopFeedback(varL(i,:));
end

figure_numbering = [303201;303202;303203;303204;303205;303206];
PlotAircraftSim(t, var', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [FcL(3,:);GcL(1,:);GcL(2,:);GcL(3,:)], figure_numbering, "b-")

% Part 3c
% 0.1 rad/sec roll
init_cond = [0;0;0;0;0;0;0;0;0;0.1;0;0];

[t, var] = ode45(@(t, var) QuadrotorEOMwithInnerControl(t, var, g, m, I, nu, mu), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOMwithInnerControl_Linearized(t, var, g, m, I), timespan, init_cond);

Fc = zeros(3, length(var));
Gc = zeros(3, length(var));
for i = 1:length(var)
    [Fc(:,i), Gc(:,i)] = InnerLoopFeedback(var(i,:));
end
FcL = zeros(3, length(varL));
GcL = zeros(3, length(varL));
for i = 1:length(varL)
    [FcL(:,i), GcL(:,i)] = InnerLoopFeedback(varL(i,:));
end

figure_numbering = [303301;303302;303303;303304;303305;303306];
PlotAircraftSim(t, var', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [FcL(3,:);GcL(1,:);GcL(2,:);GcL(3,:)], figure_numbering, "b-")

% Part 3d
% 0.1 rad/sec pitch
init_cond = [0;0;0;0;0;0;0;0;0;0;0.1;0];

[t, var] = ode45(@(t, var) QuadrotorEOMwithInnerControl(t, var, g, m, I, nu, mu), timespan, init_cond);
[tL, varL] = ode45(@(t, var) QuadrotorEOMwithInnerControl_Linearized(t, var, g, m, I), timespan, init_cond);

Fc = zeros(3, length(var));
Gc = zeros(3, length(var));
for i = 1:length(var)
    [Fc(:,i), Gc(:,i)] = InnerLoopFeedback(var(i,:));
end
FcL = zeros(3, length(varL));
GcL = zeros(3, length(varL));
for i = 1:length(varL)
    [FcL(:,i), GcL(:,i)] = InnerLoopFeedback(varL(i,:));
end

figure_numbering = [303401;303402;303403;303404;303405;303406];
PlotAircraftSim(t, var', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "r-")
PlotAircraftSim(tL, varL', [FcL(3,:);GcL(1,:);GcL(2,:);GcL(3,:)], figure_numbering, "b-")

%% Part 7
% Part 7a (Longitudinal Translation, setpoint at x=1 meter)
% Start at steady hover
init_cond = zeros(12,1);

% Call nonlinear EOM simulation with velocity, Euler angle, and angular rate feedback 
[t, var] = ode45(@(t, var) QuadrotorEOMwithLongControl(t, var, g, m, I, nu, mu), timespan, init_cond);

% Recalculate control forces and moments from VelocityReferenceFeedback function
Fc = zeros(3, length(var));
Gc = zeros(3, length(var));
for i = 1:length(var)
    [Fc(:,i), Gc(:,i)] = VelocityReferenceFeedback(t(i), var(i,:));
end

% Plot figures
figure_numbering = [307101;307102;307103;307104;307105;307106];
PlotAircraftSim(t, var', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "g-")

% Part 7b (Lateral Translation, setpoint at y=1 meter)
% Start at steady hover
init_cond = zeros(12,1);

[t, var] = ode45(@(t, var) QuadrotorEOMwithLatControl(t, var, g, m, I, nu, mu), timespan, init_cond);

Fc = zeros(3, length(var));
Gc = zeros(3, length(var));
for i = 1:length(var)
    [Fc(:,i), Gc(:,i)] = VelocityReferenceFeedback(t(i), var(i,:));
end

figure_numbering = [307201;307202;307203;307204;307205;307206];
PlotAircraftSim(t, var', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "g-")

% Part 7c (Full Translation, setpoint at x=1 meter y=1 meter)
init_cond = zeros(12,1);

[t, var] = ode45(@(t, var) QuadrotorEOMwithFullControl(t, var, g, m, I, nu, mu), timespan, init_cond);

Fc = zeros(3, length(var));
Gc = zeros(3, length(var));
for i = 1:length(var)
    [Fc(:,i), Gc(:,i)] = VelocityReferenceFeedback(t(i), var(i,:));
end

figure_numbering = [307301;307302;307303;307304;307305;307306];
PlotAircraftSim(t, var', [Fc(3,:);Gc(1,:);Gc(2,:);Gc(3,:)], figure_numbering, "g-")
