function R=Rot(axis, angle)
%Homogenous transformation for a rotation around "axis" for the amount of "angle"
axis = upper(axis);
%angle = deg2rad(angle);
if (axis == 'X')
R=[1 0 0 0; 0 cos(angle) -sin(angle) 0; 0 sin(angle) cos(angle) 0; 0 0 0 1];
end
if (axis == 'Y')
R=[cos(angle) 0 sin(angle) 0; 0 1 0 0; -sin(angle) 0 cos(angle) 0; 0 0 0 1];
end
if (axis =='Z')
R=[cos(angle) -sin(angle) 0 0; sin(angle) cos(angle) 0 0; 0 0 1 0; 0 0 0 1];
end
end