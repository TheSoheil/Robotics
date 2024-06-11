% This script is developed by Soheil Hekmat to calculate rms error needed
% in Robot Control Assignment
n = length(out.x_8);
m = length(out.x_5);
%disp(m)
%disp(n)
% rmse is root mean square error based on the formula we calc it in a loop
% using our outputs from two robots(5th assignment and this one with controling)
sum = 0;
for i = 1:n 
    err = [out.x_8(i);out.y_8(i);out.z_8(i)]-[out.x_5(i);out.y_5(i);out.z_5(i)];
    err = norm(err);
    sum = sum + err^2;
end
rms = sqrt(sum / n);
fprintf('rmse is: %0.3f', rms)
