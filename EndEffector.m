clc; clear;
% These are the constant distances
a = 20; b = a; c = 40;
d = 15; e = d; s = 5; 
% Here we Begin Transforming
H = Trans('Y',a + b);
H = H * Trans('Z',e + s);
H = H * Trans('X',s);
H = H * Rot('Y',-90);
disp('Our final homogenous transformation will be as followed:')
disp(H);
% Now we will use vector P1 to get the position
P1 = [0;0;0;1];
Pos = H * P1;
disp('Position of square will be as followed:')
disp(Pos);
disp('Note that the last elemnt of the matrix (1) is based on definition and its not envolved in position.')
PurePos = [Pos(1,1);Pos(2,1);Pos(3,1)];
disp('acctual position vector would be like this:')
disp(PurePos);
disp('Results are based on cm unit.')