clc; clear;
% This script is developed by Soheil Hekmat to initialize parameters needed
% in Robot Control Assignment
% we start assigning the data
% Q2
R = 50;       % This is Resistance(OHM)
n = 50;       % This is Transmission Ratio(no unit)(in doc it's defined as r) i named it like lectures
Bm = 0.01;    % This is Viscose Friction Coefficient at the Motor and Transmission(Nms)
Km = 7.5;     % This is Motor Constant((N.m)/amp)
Kb = Km;      % This is Back EMF Constant(which is equal to Km)
Jm = 0.1;     % This is Angular Inertia of Armature(kgm^2)
Jg = 0.05;    % This is Angular Inertia of Gears(kgm^2)
JL = 0;       % This is Angular Inertia of the Load(kgm^2) 
L = 0;        % This is Armature Inductance
voltage = 48; % This is Input Voltage(V)
Va = voltage; % This is Armature Voltage(V)
%%%%%
% now we should calculate other parameters by the ones above
J = Jm + Jg + (1/n^2)*JL;  % This is Total Angular Inertia
B = Bm + (Km*Kb)/R;        % This is Total Friction 
%%%%%
% Q3
ts = 1;       % This is Setteling time = 1 sec
Mp = 4/100;   % This is Overshoot
temp = sqrt(log(Mp)^2 / (pi^2 + log(Mp)^2));
%%%%%
Kd = (9.2*J/ts) - B + 1.9;          % This is Derivative Coefficient
Kp = J/(temp*ts/4.6)^2 + 38;      % This is Proportional Coefficient
Ki_max = Kp * (B + Kd)/J;         % This is  maximum Integrative Coefficient
Ki = 1;                         % This is Integrative Coefficient which we choose between 0 and Ki_max