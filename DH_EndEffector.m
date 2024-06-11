clc; clear;
% These are the constant distances in mm
L1 = 45; L2 = 22.5; Theta = [-90,30,150,-18,56,280];
d1 = [150,120,30,100,30,220]; d2 = [100,80,0,0,170,110]; 
% Here we Begin Transforming in a for loop for all 6 states given in table
for i = 1:6
    H = Rot('Z',Theta(i));
    H = H * Trans('Z',d1(i));
    H = H * Rot('Z',90);
    H = H * Trans('Z',L2);
    H = H * Trans('X',L1);
    H = H * Rot('X',90);
    H = H * Trans('Z',d2(i) + 30);
    % Our final homogenous transformation will be as followed:
    % disp(H);
    % Now we will use vector P1 to get the position
    P1 = [0;0;0;1];
    Pos = H * P1;
    % Position of square will be as followed:
    %  disp(Pos);
    % Note that the last elemnt of the matrix (1) is based on definition and its not envolved in position.
    PurePos = [Pos(1,1);Pos(2,1);Pos(3,1)];
    fprintf('acctual position vector for state number %i would be as followed:\n', i);
    fprintf('x: %.3f\n', PurePos(1,1));
    fprintf('y: %.3f\n', PurePos(2,1));
    fprintf('z: %.3f\n\n', PurePos(3,1));
    % disp(PurePos);
end
disp('Results are based on mm unit.')