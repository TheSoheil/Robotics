clc; clear;
% This program is developed by  Soheil Hekmat respect to Question3
% in this script we will use our path generation function to build a path
% for our robot. we start by creating matrix B and our obstacles
% B is a 13*4 matrix. each row is start and end coordinates of one side 
% of a polygone which is a simply a line. line 7 is for the square line 8
% for triangle and line 9 for pentagone and line 10 for new line obstacle
B = [4,12,6,10;6,10,12,16;12,16,10,18;10,18,4,12;
    9,10,6,4;6,4,12,4;12,4,9,10;
   11,12,15,8;15,8,19,12;19,12,17,16;17,16,13,16;13,16,11,12;
   17.5,10,17.5,8];
eta = 1;
Xs = [1;10];
Xf = [22;12];
P = Path_generator (Xs, Xf, eta, B);
% now that we have our path. we can plot the obstacles and path (whole
% workspace). first we visualize the obstacles
% first obstacle the square:
x1 = [6,4,10,12,6];
y1 = [10,12,18,16,10];
plot(x1, y1)
hold on
% second obstacle the triangle:
x2 = [9,6,12,9];
y2 = [10,4,4,10];
plot(x2, y2)
% third obstacle the pentagone:
x3 = [11,15,19,17,13,11];
y3 = [12,8,12,16,16,12];
plot(x3,y3)
% fourth obstacle the line:
x4 = [17.5,17.5];
y4 = [10,8];
plot(x4,y4)
% now we plot the path
P_trans = transpose(P);
scatter(Xs(1), Xs(2));
scatter(Xf(1), Xf(2));
plot(P_trans(:, 1), P_trans(:, 2));
axis([0 25 0 25])
hold off
