function res = Collision(X,X_temp,B)
% in this function we want to check whether if our new point of the path
% collides with obstacles or not
res = 0;
for i=1:13
A = [B(i,1);B(i,2)];
C = [B(i,3);B(i,4)];
D = [X_temp - X,-(A - C)];
alfa = D \ (C - X);
if alfa(1,1) >= 0 && alfa(1,1) <= 1 && alfa(2,1) >= 0 && alfa(2,1) <= 1 
    res = 1;
    break
end
end
end
