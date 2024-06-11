function road = dijkstra_99106074(graph_links ,num_vertices)
% this function is developed by Soheil Hekmat
% we wanna perform dijkstra algorithm to find a road or path for our robot
% we get graph links and number of vertices as input. we will develop a
% program exactly based on lectures 19-20
% we assign a distance from final point to each vertice. last one is zero
% others infinite(at first).
dist_vert = [];
for i=1:num_vertices
    if i == num_vertices
        dist_vert(i) = 0;
    else
        dist_vert(i) = inf;
    end
end
active_vert = num_vertices;
% now we should create not visited vertices matrix. if an elements value is
% 0 it's not visited and if it's 1 it is visited.
not_visited_vert = [];
for i=1:num_vertices
    if i == num_vertices
        not_visited_vert(i) = 1;
    else
        not_visited_vert(i) = 0;
    end
end
% we will start iteration to find all minimum distances based on algorithm
% we will be finished when we have visited all vertices(all elements in
% matrix should have 1 as value).
while sum(not_visited_vert) ~= num_vertices
    % we should give new distances to vertices based on algorithm
    for k=1:num_vertices
        if (not_visited_vert(k) == 0) && (dist_vert(k) > dist_vert(active_vert) + graph_links(active_vert, k) )
            % first condition is to make sure it's not a visited vertice
            % and second one is to check if the new distance is lower than 
            % current one
            dist_vert(k) = dist_vert(active_vert) + graph_links(active_vert, k);
        end
    end
    % now to find the nearest not visited neighbor we will set all visited
    % vertices to inf value in another dummy matrix. then we change the
    % active node. finally we will change the active vertice to visited
    % (1 value). and then we iterate.
    dummy_dist = dist_vert;
    for i = 1:num_vertices
        if not_visited_vert(i) == 1
            dummy_dist(i) = inf;
        end
    end
    [value, vertice] = min(dummy_dist);
    active_vert = vertice;
    not_visited_vert(active_vert) = 1;
end
% after using up all vertices now we should generate the road by algorithm
% in lectures 19-20. we start by assuming start vertice as active one and
% then we will check distances and changing active vertice to reach the
% final vertice.
road = []; active_one = 1;
while active_one ~= num_vertices
    for i=1:num_vertices
        if (graph_links(active_one,i) ~= inf) && (dist_vert(active_one) == dist_vert(i) + graph_links(active_one,i))
            active_one = i;
            road = [road, active_one];
        end
    end
end
end