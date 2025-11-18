function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

xE    = aircraft_state(1);
yE    = aircraft_state(2);
zE    = aircraft_state(3);
phi   = aircraft_state(4);
theta = aircraft_state(5);
psi   = aircraft_state(6);
uE     = aircraft_state(7);
vE     = aircraft_state(8);
wE     = aircraft_state(9);
p     = aircraft_state(10);
q     = aircraft_state(11);
r     = aircraft_state(12);

R_bi = FromInertialToBody(phi, theta, psi)

altitude = -zE;         
density = stdatmo(altitude);

[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

uEdot = r*vE - q*wE + aircraft_parameters.m*aircraft_parameters.g*(-sin(theta));
vEdot = p*wE - r*uE + aircraft_parameters.m*aircraft_parameters.g*cos(theta)*sin(phi);
wEdot = q*uE - p*vE + aircraft_parameters.m*aircraft_parameters.g*cos(theta)*cos(phi);

Ixx = aircraft_parameters.Ix;
Iyy = aircraft_parameters.Iy;
Izz = aircraft_parameters.Iz;
Ixz = aircraft_parameters.Ixz;

I = [ Ixx  0   -Ixz;
      0    Iyy  0;
     -Ixz  0    Izz ];
     
omega = [p; q; r];
omega_dot = I \ (aero_moments - cross(omega, I*omega));

pdot = omega_dot(1);
qdot = omega_dot(2);
rdot = omega_dot(3);

vel_body = [uE; vE; wE];
vel_inertial = R_bi * vel_body;

xEdot = vel_inertial(1);
yEdot = vel_inertial(2);
zEdot = vel_inertial(3);

phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

xdot = [xEdot;
        yEdot;
        zEdot;
        phi_dot;
        theta_dot;
        psi_dot;
        uEdot;
        vEdot;
        wEdot;
        pdot;
        qdot;
        rdot];


end



