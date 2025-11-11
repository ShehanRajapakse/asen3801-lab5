function PlotAircraftSim_lab5(t, x, cInp, fig, col)
% PlotAircraftSim - Plots aircraft simulation states and control inputs
%
% Inputs:
%   t      - time vector [s]
%   x      - aircraft state matrix over time
%   cInp   - control input matrix over time
%            [elevator, aileron, rudder, throttle]
%   fig    - figure handle or number
%   col    - plot color or line style

%---------------------------
% Split state vector
%---------------------------
xE = x(:,1);  % inertial X
yE = x(:,2);  % inertial Y
zE = x(:,3);  % inertial Z
phi = x(:,4); % roll
thta = x(:,5);% pitch
psi = x(:,6); % yaw
uE = x(:,7);  % X velocity
vE = x(:,8);  % Y velocity
wE = x(:,9);  % Z velocity
p = x(:,10);  % roll rate
q = x(:,11);  % pitch rate
r = x(:,12);  % yaw rate

%---------------------------
% Convert angles to degrees
%---------------------------
phi  = rad2deg(phi);
thta = rad2deg(thta);
psi  = rad2deg(psi);
p = rad2deg(p);
q = rad2deg(q);
r = rad2deg(r);

%---------------------------
% Split control inputs
%---------------------------
% cInp = [elevator, aileron, rudder, throttle]
elevator = rad2deg(cInp(:,1));  % [deg]
aileron  = rad2deg(cInp(:,2));  % [deg]
rudder   = rad2deg(cInp(:,3));  % [deg]
throttle = cInp(:,4);           % [0–1] fraction

%---------------------------
% Plot 1: Inertial Position
%---------------------------
figure('Name', 'Inertial Position');
subplot(3,1,1); plot(t, xE, col); ylabel('X [m]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,2); plot(t, yE, col); ylabel('Y [m]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,3); plot(t, zE, col); ylabel('Z [m]'); xlabel('Time [s]'); grid on; hold on;

%---------------------------
% Plot 2: Euler Angles
%---------------------------
figure('Name', 'Euler Angles');
subplot(3,1,1); plot(t, phi, col); ylabel('Roll \phi [deg]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,2); plot(t, thta, col); ylabel('Pitch \theta [deg]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,3); plot(t, psi, col); ylabel('Yaw \psi [deg]'); xlabel('Time [s]'); grid on; hold on;

%---------------------------
% Plot 3: Inertial Velocities
%---------------------------
figure('Name', 'Inertial Velocity');
subplot(3,1,1); plot(t, uE, col); ylabel('u_E [m/s]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,2); plot(t, vE, col); ylabel('v_E [m/s]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,3); plot(t, wE, col); ylabel('w_E [m/s]'); xlabel('Time [s]'); grid on; hold on;

%---------------------------
% Plot 4: Angular Velocities
%---------------------------
figure('Name', 'Angular Velocity');
subplot(3,1,1); plot(t, p, col); ylabel('p [deg/s]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,2); plot(t, q, col); ylabel('q [deg/s]'); xlabel('Time [s]'); grid on; hold on;
subplot(3,1,3); plot(t, r, col); ylabel('r [deg/s]'); xlabel('Time [s]'); grid on; hold on;

%---------------------------
% Plot 5: Control Inputs
%---------------------------
figure('Name', 'Control Inputs');
subplot(4,1,1);
plot(t, elevator, col, 'LineWidth', 1.2);
ylabel('Elevator [deg]');
xlabel('Time [s]');
grid on; hold on;

subplot(4,1,2);
plot(t, aileron, col, 'LineWidth', 1.2);
ylabel('Aileron [deg]');
xlabel('Time [s]');
grid on; hold on;

subplot(4,1,3);
plot(t, rudder, col, 'LineWidth', 1.2);
ylabel('Rudder [deg]');
xlabel('Time [s]');
grid on; hold on;

subplot(4,1,4);
plot(t, throttle, col, 'LineWidth', 1.2);
ylabel('Throttle [0–1]');
xlabel('Time [s]');
grid on; hold on;

%---------------------------
% Plot 6: 3D Path
%---------------------------
figure('Name', '3D Path of Aircraft');
plot3(xE, yE, -zE, col, 'LineWidth', 1.2); hold on;
scatter3(xE(1), yE(1), -zE(1), 60, 'g', 'filled');
scatter3(xE(end), yE(end), -zE(end), 60, 'r', 'filled');
xlabel('X_E [m]');
ylabel('Y_E [m]');
zlabel('Altitude [m]');
title('3D Flight Path');
grid on; axis equal;

end
