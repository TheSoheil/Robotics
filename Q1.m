clc;clear;
% this is developed by Soheil Hekmat respect to 1st Question
% we will pass DH table and joint variables (Q) of our two robots (rrp and 
% rpp) to our JACOB function and get its jacobian matrix.

% first rpp robot related to 2nd assignment
syms theta d1 d2
dh1 = [theta,d1,0,0;pi/2,22.5,45,pi/2;0,30+d2,0,0];
q1 = [theta;d1;d2];
[Jv1,Jw1,H1] = JACOB(dh1,q1);
J1 = [simplify(Jv1);simplify(Jw1)];
disp('H for rpp robot is:')
disp(simplify(H1))
disp('jacobian of rpp robot is:')
disp(simplify(J1))
disp('***************************')
% then rrp robot related to 3rd assignment

syms theta1 theta2 d
dh2 = [theta1,310,0,pi/2;pi/2+theta2,0,0,pi/2;0,300+d,0,0];
q2 = [theta1;theta2;d];
[Jv2,Jw2,H2] = JACOB(dh2,q2);
J2 = [Jv2;Jw2];
disp('H for rrp robot is:')
disp(simplify(H2))
disp('jacobian of rrp robot is:')
disp(simplify(J2))
