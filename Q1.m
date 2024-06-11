% This program is developed by Soheil Hekmat for First problem
% at first we should get the data from a txt file(part a)
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
start = mat(50:51);
finish = mat(52:53);
% we wanna remove duplicate coordinates in 'obstac_coor' to get graph nodes
graph_vertices = [];
% adding start point to nodes
graph_vertices(1,1) = start(1);
graph_vertices(1,2) = start(2);
count = 0; index = 3;
for i=1:48
    % in this for loop we wanna get values from indexes i=1,2,5,6,....45,46
    if count ~= 2 && count ~= 3
        graph_vertices(1,index) = obstac_coor(i);
        count = count + 1;
        index = index + 1;
    else 
        count = count + 1;
    end
    if count == 4
        count = 0;
    end
end
% adding end point to nodes
graph_vertices(1,27) = finish(1);
graph_vertices(1,28) = finish(2);
% splitting x's and y's by odd and even indexes
splitted_graph_vertices = []; temp = 1;
for k=1:28
    if rem(k,2) == 1
        splitted_graph_vertices(temp,1) = graph_vertices(k);
    else
        splitted_graph_vertices(temp,2) = graph_vertices(k);
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
% we will build the graph or road map
graph_links = distance_collision(splitted_graph_vertices,poly_obstac);
% now to perform dijkstra algorithm for road mapping we define a function.
road = dijkstra(graph_links, length(splitted_graph_vertices));
% now we wanna plot the obstacles graph and road which we generated(part b)
% 1.obstacles
plot(poly_obstac)
hold on
% 2. graph links
interv = length(splitted_graph_vertices);
for i = 1:interv
    for j = 1:interv
        if graph_links(i,j) ~= inf
            % we don't wanna draw collided links (inf values)
            % extracting and splitting x's and y's for plotting
            x_links = [splitted_graph_vertices(i,1), splitted_graph_vertices(j,1)];
            y_links = [splitted_graph_vertices(i,2), splitted_graph_vertices(j,2)];
            % plotting graph links as dashed black lines
            plot(x_links, y_links, ':k');
        end
    end
end
% 3. generated road by road mapping
x_road = []; y_road = [];
for i = 1:length(road)
    % extracting and splitting x's and y's for plotting the road
    x_road = [x_road, splitted_graph_vertices(road(i), 1)];
    y_road = [y_road, splitted_graph_vertices(road(i), 2)];
end
plot(x_road,y_road, 'r')
scatter(x_road,y_road,'filled')
% 4. start and end point
% plot(start(1), start(2), 'o','MarkerSize', 8);
% plot(finish(1), finish(2), 'o','MarkerSize', 8);
text(start(1), start(2) + 1, 'start')
text(finish(1), finish(2) + 1, 'end')
hold off
