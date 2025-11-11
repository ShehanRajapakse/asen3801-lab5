function [Q] = DCM_y(theta)

Q = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];

end