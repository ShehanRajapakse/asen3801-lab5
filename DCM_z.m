function [Q] = DCM_z(psi)

Q = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];

end