% This program is developed by Soheil Hekmat for Second problem
% as same as Q1 at first we should get the data from a txt file
clc; clear;
f = 'input.txt';
% we need to read the data from the file so we use 'r' for file permission
f_identifier = fopen(f, 'r');
formatSpec = '%d';
% we use textscan to read the data. we use ',' for delimeter because we have
% a csv file
C = textscan(f_identifier, formatSpec, 'Delimiter', ',');
% because textscan's output is a cell we change it to matrix for ease
mat = cell2mat(C);
% we get all values we need from the file and categorize them
N = mat(1);
obstac_coor = mat(2:49);
finish = mat(52:53);
m = mat(54);
% robot vert is a 1*2m matrix including polygonal robot's vertices 
robot_vert = mat(55:length(mat));
% splitting polygonal robot's vertices into x's and y's
splitted_robot_vert = []; s = 1;
for j=1:length(robot_vert)
    if rem(j,2) == 1
        splitted_robot_vert(s,1) = robot_vert(j);
    else
        splitted_robot_vert(s,2) = robot_vert(j);
        s = s + 1;
    end
end
% start in this problem is the first vertice of polygonal robot
start = [splitted_robot_vert(1,1);splitted_robot_vert(1,2)];
% we wanna remove duplicate coordinates in 'obstac_coor' to get graph nodes
obs_vertices = [];
count = 0; index = 1;
for i=1:48
    % in this for loop we wanna get values from indexes i=1,2,5,6,....45,46
    if count ~= 2 && count ~= 3
        obs_vertices(1,index) = obstac_coor(i);
        count = count + 1;
        index = index + 1;
    else 
        count = count + 1;
    end
    if count == 4
        count = 0;
    end
end
% splitting x's and y's by odd and even indexes
splitted_obs_vertices = []; temp = 1;
for k=1:24
    if rem(k,2) == 1
        splitted_obs_vertices(temp,1) = obs_vertices(k);
    else
        splitted_obs_vertices(temp,2) = obs_vertices(k);
        temp = temp + 1;
    end
end
% now we should make a obstacle matrix it includes all lines of polygones
obstac_lines = []; a = 2;
for i=1:12
    obstac_lines(i,1) = mat(a);
    obstac_lines(i,2) = mat(a+1);
    obstac_lines(i,3) = mat(a+2);
    obstac_lines(i,4) = mat(a+3);
    a = a + 4;
end
% now we wanna make our polygonal obstacles
poly_obstac = poly_maker(obstac_lines);
% to solve this problem we need to use algorithm convex hull which was
% teached in lectures 21-22. based on the algorithm we should build vectors
% from all vertices of robot reaching out the first vertice
% now to build vectors we use a loop to make and save vectors starting from
% other vertices and ending in first vertice
for i=1:m-1
    built_vect{i} = start - [splitted_robot_vert(i+1,1);splitted_robot_vert(i+1,2)];
end
% now we wanna findout how many sides does each polygonal obstacle have.
each_poly_len = []; len_pol = 0;
for i=1:length(poly_obstac.Vertices)
    if isnan(poly_obstac.Vertices(i,1)) == 1
        each_poly_len = [each_poly_len, len_pol];
        len_pol = 0;
        continue
    end
    len_pol = len_pol + 1;
end
each_poly_len = [each_poly_len, len_pol];
% now that we have every thing needed we start performing convex hull
new_obstac_lines = []; while_counter = 1; in1 = 1; in2 = 1;
while while_counter <= poly_obstac.NumRegions(1)
    x_pol = []; y_pol = [];
    % adding new obstacle vertices coordinates by those built vectors
    for i=1:each_poly_len(while_counter)
        for j=1:length(built_vect)
            x_pol = [x_pol, splitted_obs_vertices(in1,1) + built_vect{j}(1)];
            y_pol = [y_pol, splitted_obs_vertices(in1,2) + built_vect{j}(2)];
        end
        in1 = in1 + 1;
    end
    % adding original obstacle vertices coordinates
    for k=1:each_poly_len(while_counter)
        x_pol = [x_pol, splitted_obs_vertices(in2,1)];
        y_pol = [y_pol, splitted_obs_vertices(in2,2)];
        in2 = in2 + 1;
    end
    % performing convex hull to coordinates of new obstacle vertices
    c_h = convhull(x_pol, y_pol);
    % now we extract lines of new obstacles as our polygonal's lines we
    % had before in previous problem
    for m=1:length(c_h) - 1 % this subtraction is because of output of conv_hull which last element is repetitious
    new_obstac_lines = [new_obstac_lines; x_pol(c_h(m)) y_pol(c_h(m)) x_pol(c_h(m+1)) y_pol(c_h(m+1))];
    end
    while_counter = while_counter + 1;
end
% now that we have convexed obstacles we need to build up the graph
% vertices and links and then dijkstra exactly as Q1
split_gr_vert = [];
% adding start point to nodes
split_gr_vert(1,1) = start(1,1);
split_gr_vert(1,2) = start(2,1);
for i=2:(length(new_obstac_lines) + 1)
    split_gr_vert(i,1) = new_obstac_lines(i-1,1);
    split_gr_vert(i,2) = new_obstac_lines(i-1,2);
end
% adding end point to nodes
split_gr_vert(length(new_obstac_lines) + 2,1) = finish(1,1);
split_gr_vert(length(new_obstac_lines) + 2,2) = finish(2,1);
% now we wanna make our new and convexed polygonal obstacles with polyshape
conv_poly_obstac = poly_maker(new_obstac_lines);
% we will build the graph 
graph_links = distance_collision(split_gr_vert,conv_poly_obstac);
% now to perform dijkstra algorithm for road mapping we define a function.
road = dijkstra(graph_links, length(split_gr_vert));
% now we wanna plot the obstacles graph and road which we generated
% 1. original obstacles and convexed ones
%plot(conv_poly_obstac,'FaceColor', 'blue','EdgeColor', 'red')
plot(poly_obstac,'FaceColor', 'red','EdgeColor', 'red')
hold on
% 2. graph links
interv = length(split_gr_vert);
for i = 1:interv
    for j = 1:interv
        if graph_links(i,j) ~= inf
            % we don't wanna draw collided links (inf values)
            % extracting and splitting x's and y's for plotting
            x_links = [split_gr_vert(i,1), split_gr_vert(j,1)];
            y_links = [split_gr_vert(i,2), split_gr_vert(j,2)];
            % plotting graph links as dashed black lines
            plot(x_links, y_links, ':k');
        end
    end
end
% 3. generated road by road mapping
x_road = []; y_road = [];
for i = 1:length(road)
    % extracting and splitting x's and y's for plotting the road
    x_road = [x_road, split_gr_vert(road(i), 1)];
    y_road = [y_road, split_gr_vert(road(i), 2)];
end
plot(x_road,y_road, 'b','LineWidth', 1.5)
scatter(x_road,y_road,'filled')
% 4. start and end point
% plot(start(1), start(2), 'o','MarkerSize', 8);
% plot(finish(1), finish(2), 'o','MarkerSize', 8);
text(start(1), start(2) - 0.5, 'start')
text(finish(1), finish(2) - 0.5, 'end')
% 5. the robot
robot_shape = polyshape(splitted_robot_vert(:,1),splitted_robot_vert(:,2));
plot(robot_shape,'LineWidth', 1.5)
hold off
