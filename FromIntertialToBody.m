function [Q] = FromIntertialToBody(phid, thetad, psid)

phi = deg2rad(phid); theta = deg2rad(thetad); psi = deg2rad(psid);

Rphi = [1 0 0; 0 cos(deg2rad(phi)) sin(phi); 0 -sin(phi) cos(phi)];
Rtheta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
Rpsi = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];

Q = Rpsi * Rtheta * Rphi;

end