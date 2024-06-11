function graph_links = distance_collision(splitted_graph_vertices,poly_obstac)
% this function is developed by Soheil Hekmat
% in this function we wanna calculate each graph vertice distance to other
% vertices and check whether if this distance collides any obstacle 
interval = length(splitted_graph_vertices);
graph_links = [];
for i = 1:interval
    for j = 1:interval
        if i == j
            % this means we are checking just one point with itself
            graph_links(i,j) = 0;
            continue
        end
        % we split the line into 10 points to check collision
        count = norm(splitted_graph_vertices(i,:) - splitted_graph_vertices(j,:)) * 10;
        x_points = linspace(splitted_graph_vertices(i,1), splitted_graph_vertices(j,1), count);
        y_points = linspace(splitted_graph_vertices(i,2), splitted_graph_vertices(j,2), count);
        [inpol, onbond] = isinterior(poly_obstac, x_points, y_points);
        % if our point is on boundary of obstacle collision has not occured
        inside = inpol & ~onbond;
        if sum(inside) == 0
            % this means we don't have collision and this is ok to count as
            % a graph link
            graph_links(i,j) = norm(splitted_graph_vertices(i,:) - splitted_graph_vertices(j,:));
        else 
            % this means we have a collision with obstacles
            graph_links(i,j) = inf;
        end
    end    
end