clc; clear;
% These are EndEffector's position in mm, we have six different series
% the first 3 series are results of question1 and other 3 are given in q2
X = [-202.567,211.535,0,68.4,298.5,184.9];
Y = [241.41,122.13,516.831,12,-250.5,139.3];
Z = [365.569,601.98,498.112,703.9,535,86.3];
% Here we Begin solving in a for loop for all 6 states given in table
% in this loop we pass each states x y z to our inverse function
for i = 1:6
    [theta1,theta2,d] = INV_Kinematics_Func(X(i), Y(i), Z(i));
    fprintf('our joint variables for state number %i would be as followed:\n', i);
    fprintf('theta1: %.3f\n', theta1);
    fprintf('theta2: %.3f\n', theta2);
    fprintf('d: %.3f\n\n', d);
end