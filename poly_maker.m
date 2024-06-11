function poly_obstac = poly_maker(obstac_lines)
% this function is developed by Soheil Hekmat
% in this function we wanna detect obstacles as polygones. we have 3 
% obstacles in this problem. we give coordinates of each polygon to
% it's specific matrixes and then we use polyshape func to build the
% obstacles
pol_sides = []; temp1 = 0; temp2 = 0; first_ind = 1; sec_ind = 1;
for n=1:length(obstac_lines)
    temp1 = obstac_lines(first_ind,1);
    temp2 = obstac_lines(sec_ind,2);
    if temp1 == obstac_lines(n,3) && temp2 == obstac_lines(n,4)
        pol_sides = [pol_sides, n];
        first_ind = n + 1;
        sec_ind = n + 1;
    end
end
x_p1 = []; y_p1 = [];
x_p2 = []; y_p2 = [];
x_p3 = []; y_p3 = [];
a1 = 1; a2 = 1; a3 = 1;
for i=1:length(obstac_lines)
    if i <= pol_sides(1)
        x_p1(a1) = obstac_lines(i,1);
        y_p1(a1) = obstac_lines(i,2);
        a1 = a1 + 1;
    end
    if i <= pol_sides(2) && i >= (pol_sides(1) + 1)
        x_p2(a2) = obstac_lines(i,1);
        y_p2(a2) = obstac_lines(i,2);
        a2 = a2 + 1;
    end
    if i >= pol_sides(2) + 1
        x_p3(a3) = obstac_lines(i,1);
        y_p3(a3) = obstac_lines(i,2);
        a3 = a3 + 1;
    end
end
poly_obstac = polyshape({x_p1,x_p2,x_p3},{y_p1,y_p2,y_p3});
end