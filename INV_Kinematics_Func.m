function [theta1, theta2, dist] = INV_Kinematics_Func(x, y, z)
% here are our constant distances in mm
L1 = 310; L2 = 300;
% now we start by finding first joint variable based on alghorithm
s1 = y/(x^2+y^2)^0.5;
c1 = x/(x^2+y^2)^0.5;
theta1 = rad2deg(atan2(s1,c1));
% now we define the position vector for endeffector in world frame 
% then we transform it into second frame defind by 1 index (x1-z1)
p0_0 = [x;y;z;1];
H1_0 = Rot('Z', -theta1) * Trans('Z', -L1);
p1_1 = H1_0 * p0_0;
% now we find second joint variable
x1 = p1_1(1,1);
z1 = p1_1(3,1);
s2 = z1/(x1^2+z1^2)^0.5;
c2 = x1/(x1^2+z1^2)^0.5;
theta2 = rad2deg(atan2(s2, c2));
% and finally the last one
dist = (x1^2+z1^2)^0.5 - L2;
end