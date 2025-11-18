function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, aircraft_parameters)

de = aircraft_surfaces(1);
da = aircraft_surfaces(2);
dr = aircraft_surfaces(3);
dt = aircraft_surfaces(4);

if time <= doublet_time
    de = de + doublet_size;
elseif time <= 2*doublet_time
    de = de - doublet_size;
end

u_ctrl = [de; da; dr; dt];
xdot = AircraftEOM(time, aircraft_state, u_ctrl, wind_inertial, aircraft_parameters);

end
