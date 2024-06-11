clc;clear;
% this is developed by Soheil Hekmat respect to 2nd Question
% we wanna find singular points in each robot so we will use the method
% mentioned in lecture 14, so we have to calculate det for jacobian of each
% robot. note that because our two robots have Transitional displacement we
% only need to find det of v's jacobian. after that we should solve
% equation det(Jv) = 0 to find singular points.

% at first we go exactly like Q1 to get Jacobian 
% rpp's jacobian 
syms theta d1 d2
dh1 = [theta,d1,0,0;pi/2,22.5,45,pi/2;0,30+d2,0,0];
q1 = [theta;d1;d2];
[Jv1,Jw1] = JACOB(dh1,q1);
J1 = Jv1;
% rrp's jacobian
syms theta1 theta2 d
dh2 = [theta1,310,0,pi/2;pi/2+theta2,0,0,pi/2;0,300+d,0,0];
q2 = [theta1;theta2;d];
[Jv2,Jw2] = JACOB(dh2,q2);
J2 = Jv2;
% now to find singular points
% rpp's singular points
singular1 = solve(det(J1) == 0 , [theta;d1;d2]);
disp('rpp:')
disp(singular1)
% rrp's singular points
singular2 = solve(det(J2) == 0 , [theta1;theta2;d]);
disp('rrp:')
fprintf('first one:\ntheta1 is: %.2f\n',singular2.theta1(1,1));
fprintf('theta2 is: %.2f\n',singular2.theta2(1,1));
fprintf('d is: %.2f\n',singular2.d(1,1));
fprintf('second one:\ntheta1 is: %.2f\n',singular2.theta1(2,1));
fprintf('theta2 is: %.2f\n',singular2.theta2(2,1));
fprintf('d is: %.2f\nthetas are in rad and d is in mm unit',singular2.d(2,1));

