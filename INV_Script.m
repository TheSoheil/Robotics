clc; clear;
% These are EndEffector's position in mm, we have two different series
% the first serie is related to initial pos and second one is for final pos
X = [284.829,91.203];
Y = [-164.446,340.373];
Z = [190.293,605.682];
% Here we Begin solving in a for loop for all 2 states given in table
% in this loop we pass each states x y z to our inverse function
for i = 1:2
    [theta1,theta2,d] = INV_Kinematics_Func(X(i), Y(i), Z(i));
    fprintf('our joint variables for state number %i would be as followed:\n', i);
    fprintf('theta1: %.1f deg\n', theta1);
    fprintf('theta2: %.1f deg \n', theta2);
    fprintf('d: %.1f mm \n\n', d);
end
[theta1_i,theta2_i,d_i] = INV_Kinematics_Func(X(1), Y(1), Z(1));
[theta1_f,theta2_f,d_f] = INV_Kinematics_Func(X(2), Y(2), Z(2));
initial = [deg2rad(theta1_i),deg2rad(theta2_i),d_i * 0.001];
final = [deg2rad(theta1_f),deg2rad(theta2_f),d_f * 0.001];
% in this section we wanna find the coefficients for trajectory
% we do that based on lecture 16 and it's formulas, we have T = 1.
syms a_2 a_3
for i = 1:3
    eq1 = initial(i) - final(i) + a_2 + a_3;
    eq2 = 2*a_2 + 3*a_3;
    res = solve([eq1, eq2], [a_2, a_3]);
    fprintf('our trajectory function coefficients for joint number %i would be as followed:\n', i);
    fprintf('a0: %.3f\n', initial(i));
    fprintf('a1: 0\n');
    fprintf('a2: %.3f\n', res.a_2);
    fprintf('a3: %.3f\n\n', res.a_3);
end