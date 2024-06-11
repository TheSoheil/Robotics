clc;clear;
% this is developed by Soheil Hekmat respect to 3rd Question
% in this problem we will calc the jacobian of rrp robot like other two
% question and then we wanna find out what is the max velocity
% related to each joint's motor.

% first section: calculation of jacobian
syms theta1 theta2 d
dh2 = [theta1,310,0,pi/2;pi/2+theta2,0,0,pi/2;0,300+d,0,0];
q2 = [theta1;theta2;d];
[Jv2,Jw2] = JACOB(dh2,q2);
J2 = Jv2;
% second section: velocity and joint variables assignment
% we use desired steps for joint variables (units:mm/s,mm,degree)
v_end_eff = [150;100;50];
theta1_desired = 0:deg2rad(5):deg2rad(80);
theta2_desired = deg2rad(-45):deg2rad(5):deg2rad(45);
d_desired = 100:5:200;
% 3rd section: we use 3 loops to check each states velocities and assign
% them to a matrix to get the maximum v of each motor, based on lectures
% 11 and 12 we should use inverse jacobian and velocity vecctor to get
% theta capital_dot(which is Q_dot in code variables)
v1_motor=[];v2_motor=[];v3_motor=[];
for i=1:length(theta1_desired)
    for j=1:length(theta2_desired)
        for k=1:length(d_desired)
            J2_valued = subs(J2,[theta1;theta2;d],[theta1_desired(i);theta2_desired(j);d_desired(k)]);
            Q_dot = inv(J2_valued) * v_end_eff;
            v1_motor = [v1_motor Q_dot(1,1)];
            v2_motor = [v2_motor Q_dot(2,1)];
            v3_motor = [v3_motor Q_dot(3,1)];
        end
    end
end
fprintf('maximum velocity for 1st motor is: %.3f (deg/s)\n', rad2deg(max(v1_motor)))
fprintf('maximum velocity for 2nd motor is: %.3f(deg/s)\n', rad2deg(max(v2_motor)))
fprintf('maximum velocity for 3rd motor is: %.3f(mm/s)\n', max(v3_motor))