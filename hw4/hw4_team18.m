% HW4 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

%% MAIN METHOD %%%%%%%%%%%%%%%%%%%%%

function hw4_team18(serPort, world_file, start_goal_file)

    % create the world
    [start, goal] = readStartGoalFromFile(start_goal_file);
    [wall, obstacles] = readWorldFromFile(world_file);
    [grown_obstacles] = growObstacles(obstacles);

    % plot the world
    initializePlots();
    plotPoints([start;goal], [1 0 0]);         % start and goal in red
    plotObstacle(wall, [0 0 0.5]);             % wall in dark blue
    plotObstacles(obstacles, [0 0.4 1]);       % obstacles in blue
    plotObstacles(grown_obstacles, [0, 0, 0]); % grown obstacles in black
    
    % find path to take
    [verticies, edges] = createVisibilityGraphFake(start, goal, grown_obstacles);
    edges = addEdgeCosts(verticies, edges);
    shortest_path = findShortestPath(verticies, edges, start, goal);

    % plot the paths
    plotPaths(verticies, edges, [0 0.5 0]); % all paths in dark green
    plotPath(shortest_path, [0 1 0]);       % shortest path in bright green
    
end



%% SETTING UP ENVIRONMENT %%%%%%%%%%%%%%%%%%%%%

function [wall, obstacles] = readWorldFromFile(file)

    try
        % open file
        fid = fopen(file);

        % first line is number of obstacles
        line = fgetl(fid);    
        numObstacles = str2double(line);

        % first obstacle is actually the wall
        wall = readObstacle(fid);

        % and the rest are obstacles
        obstacles = cell(1, numObstacles - 1);
        for i = 1:length(obstacles)
             obstacles{1,i} = readObstacle(fid);
        end

        % always close file even if error
        finishup = onCleanup(@() fclose(fid));

    catch
        disp('Failed to read obstacles from file');
        obstacles = zeros(0,1);
        wall = zeros(0,2);
    end

end

function obstacle = readObstacle(fid)

    % first line is number of verticies
    line = fgetl(fid);
    numVerticies = str2double(line);

    obstacle = zeros(numVerticies, 2);

    for j = 1:numVerticies

        % one vertex per line
        line = fgetl(fid);
        vertex = strsplit(line, ' ');

        obstacle(j,:) = [str2double(vertex(1)), str2double(vertex(2))];

    end

end

function [start, goal] = readStartGoalFromFile(file)

    try
        % open file
        fid = fopen(file);

        % first line is start
        line = fgetl(fid);
        point = strsplit(line, ' ');
        start = [str2double(point(1)), str2double(point(2))];

        % second line is goal
        line = fgetl(fid);
        point = strsplit(line, ' ');
        goal = [str2double(point(1)), str2double(point(2))];
        
        % always close file even if error
        finishup = onCleanup(@() fclose(fid));

    catch
        disp('Failed to read start and goal from file');
        start = [];
        goal = [];
    end

end

function grown_obstacles = growObstacles(obstacles)

    function [x0,y0] = centroid(x,y)
        [m1,n1] = size(x);
        n = max(m1,n1);
        x = x(:); y = y(:);
        x2 = [x(2:n);x(1)];
        y2 = [y(2:n);y(1)];
        a = 1/2*sum (x.*y2-x2.*y);
        x0 = 1/6*sum((x.*y2-x2.*y).*(x+x2))/a;
        y0 = 1/6*sum((x.*y2-x2.*y).*(y+y2))/a;
    end

    grown_obstacles = cell(1, length(obstacles));

    for i = 1:length(obstacles)
        
        obstacle = obstacles{i};
        
        % all verticies of original and grown obstacle
        verticies = zeros(length(obstacle)*2, 2);
        verticies(1:length(obstacle),:) = obstacle;
        
        % push all points out from centroid
        [centroid_x, centroid_y] = centroid(obstacle(:,1), obstacle(:,2));
        for j = 1:length(obstacle)
            new_x = obstacle(j, 1) + (obstacle(j, 1) - centroid_x)/3;
            new_y = obstacle(j, 2) + (obstacle(j, 2) - centroid_y)/3;
            verticies(j+length(obstacle),:) = [new_x,new_y];
        end
        
        % find grown obstacle from new set of verticies
        grown_obstacles{1,i} = computeConvexHull(verticies);
        
    end
    
end

function grown_obstacle = computeConvexHull(verticies)

    num_verticies = size(verticies, 1);

    % 1. find the lowest point (rightmost tiebreaks), label it p0
    p0 = verticies(1,:);
    for i=2:num_verticies
        vertex = verticies(i,:);
        if (vertex(2) < p0(2) || (vertex(2) == p0(2) && vertex(1) > p0(1)))
            p0 = vertex;
        end
    end

    % 2. sort pts angularly about p0 (closeness tiebreaks), label p1..pn-1
    sorted_points = zeros(num_verticies-1, 4);
    idx = 1;
    for i=1:num_verticies
        vertex = verticies(i,:);
        if (vertex == p0)
            continue;
        end
        ang = atan2(vertex(2)-p0(2),vertex(1)-p0(1));
        dist = pdist([p0;vertex]);
        sorted_points(idx,:) = [vertex,ang,dist];
        idx = idx + 1;
    end
    sorted_points = sortrows(sorted_points, [3, 4]); 

    % 3. push pn-1 and p0 onto stack
    stack = zeros(num_verticies + 1, 2);
    stack(1,:) = sorted_points(num_verticies-1,1:2);
    stack(2,:) = p0;
    stack_cur = 2;
    
    % 4. set i = 1
    i = 1;
    
    % 5. while i < n
    %      if pi strictly left of line from stack_snd to stack_top
    %      then push pi onto stack and increment i
    %      else pop stack
    while (i < num_verticies)
        point = sorted_points(i,1:2);
        top = stack(stack_cur,:);
        snd = stack(stack_cur-1,:);
        
        % look at angles to determine "strictly left"
        ang_stack = mod(atan2(top(2)-snd(2),top(1)-snd(1)), 2*pi);
        ang_point = mod(atan2(point(2)-snd(2),point(1)-snd(1)), 2*pi);
        ang_diff = mod(ang_point - ang_stack, 2*pi);

        if (ang_diff > 0 && ang_diff < pi)
            stack_cur = stack_cur + 1;  % push pi onto stack
            stack(stack_cur,:) = point;
            i = i + 1;                  % increment i
        else
            stack_cur = stack_cur - 1;  % pop stack            
        end
    end
    
    % 6. stack contains convex hull (with redundant point on top stack)
    grown_obstacle = stack(2:stack_cur,:);
            
end



%% FINDING SHORTEST PATH %%%%%%%%%%%%%%%%%%%%%

function [verticies, edges] = createVisibilityGraphFake(start, goal, obstacles)

    edges = zeros(0,2);
    verticies = [goal;start];
    
    top_left_idx = 2;
    bottom_right_idx = 2;
    for i=1:length(obstacles)
        obstacle = obstacles{i};
        verticies = [verticies;obstacle(1,:);obstacle(length(obstacle),:)];
        edges = [edges;[top_left_idx,size(verticies,1)-1];[bottom_right_idx,size(verticies,1)]];
        top_left_idx = size(verticies,1)-1;
        bottom_right_idx = size(verticies,1);
    end
    edges = [edges;[top_left_idx,1];[bottom_right_idx,1]];    
    
    verticies = [verticies;-2,1; 1,3; 3,1; 4,3];
    s = size(verticies,1);
    edges = [edges;[2,s-3];[s-3,s-2];[s-2,s-1];[s-1,s];[s,1]];
    
end

function visibility_graph = createVisibilityGraph(start, goal, obstacles)
%{
    Input. A set S of disjoint polygonal obstacles.
    Output. The visibility graph Gvis(S).
        1. Initialize a graph G = (V,E) 
           where V is the set of all vertices of thepolygons in S 
           and E = /0.
        2. for all vertices v ? V
        3.     W ? VISIBLEVERTICES(v,S)
        4.     For every vertex w ? W, add the arc (v,w) to E.
        5. return G        
%}

    % add start-goal to obstacle set
    obstacles{length(obstacles)} = [start;goal];

    % initialize g = v,e where v=all veriticies, e = empty
    num_veriticies;
    for i=1:length(obstacles)
        num_verticies = num_verticies + length(obstacles{i});
    end    
    V = zeros(num_verticies);
    for i=1:length(obstacles)
        obstacle = obstacles{i};
        for j=1:length(obstacle)
            V(j) = obstacle(j,:);
        end
    end
    E = [];

    % do algorithm
    for i=1:length(V)
        vertex = V(i);
        visible = visibleVerticies(vertex, obstacles);
        for j=1:length(visible)
            E = [E;[visible(i),vertex]];
        end
    end
    
    visibility_graph = cell(1,2);
    visibility_graph{1} = V;
    visibility_graph{2} = E;
end

function visible = visibleVerticies(p, obstacles)
%{
    Input. A set S of polygonal obstacles 
           and a point p that does not lie in theinterior of any obstacle.
    Output. The set of all obstacle vertices visible from p.
        1. Sort the obstacle vertices according to the clockwise angle 
           that the half-line from p to each vertex makes with the 
           positive x-axis. In case of ties, vertices closer to p should 
           come before vertices farther from p. 
           Let w1,...,wn be the sorted list.
        2. Let ? be the half-line parallel to the positive x-axis 
           starting at p. Findthe obstacle edges that are properly 
           intersected by ?, and store them in abalanced search tree 
           T in the order in which they are intersected by ?.
        3. W ? /0
        4. for i ? 1 to n
        5.     if VISIBLE(wi) then Add wi to W.
        6.     Insert into T the obstacle edges incident to wi that lie on 
               the clock-wise side of the half-line from p to wi.
        7.     Delete from T the obstacle edges incident to wi that lie 
               on thecounterclockwise side of the half-line from p to wi.
        8. return W
%}

    visible = [];

end

function visible()
%{
    1. if pwi intersects the interior of the obstacle of which wi 
       is a vertex, locally at wi
    2.     return false
    3. else if i = 1 or wi-?1 is not on the segment pwi
    4.     Search in T for the edge e in the leftmost leaf.
    5.     if e exists and pwi intersects e
    6.         return false
    7.     else return true
    8. else if wi-?1 is not visible
    9.     return false
    10. else Search in T for an edge e that intersects wi?1wi.
    11.     if e exists
    12.         return false
    13.     else return true
%}

end

function edges_with_cost = addEdgeCosts(verticies, edges)

    edges_with_cost = zeros(length(edges), 3);
    for i=1:length(edges)
        start_v = edges(i, 1);
        end_v = edges(i, 2);
        dist = pdist([verticies(start_v,:);verticies(end_v,:)]);
        edges_with_cost(i,:) = [start_v,end_v,dist];
    end
    
end

function path = findShortestPath(verticies, edges, start, goal)
% verticies is a list of (x, y) pairs
% edges is a list of (vert_idx1, vert_idx2, cost)
% start is vert_idx of start
% goal is vert_idx of goal
% shortest_path is list of vert_idx hops

    % if start and goal are same vertex, done
    if (start == goal)
        path = start;
        return;
    end

    num_verticies = length(verticies);      % number of verticies
    settled = cell(num_verticies, 1);       % checked nodes
    unsettled = cell(1);                    % remaining nodes
    dist = inf(1, num_verticies);           % distance from start to vertex
    predecessors = zeros(1, num_verticies); % link for recreating path

    % convert start and goal into vertex indicies
    start_idx = 0;
    goal_idx = 0;
    for i=1:num_verticies
        if (start == verticies(i,:))
            start_idx = i;
        end
        if (goal == verticies(i,:))
            goal_idx = i;
        end        
    end
    
    % initialize
    dist(start_idx) = 0;
    unsettled{1} = start_idx;
    
    while (~isempty(unsettled))
        
        % get minimum unsettled
        node = -1;
        idx = 0;
        for i=1:length(unsettled)
            vertex = unsettled(i);
            vertex = vertex{1};
            if (node == -1 || dist(vertex) < dist(node))
                node = vertex;
                idx = i;
            end
        end
        
        settled{node} = 1;
        unsettled(idx) = [];
        for i=1:length(edges)
            edge = edges(i,:);
            if (edge(1) == node && isempty(settled{edge(2)}))        
                neighbor = edge(2);                
                newDist = dist(node) + edge(3);
                if (dist(neighbor) > newDist)
                    dist(neighbor) = newDist;
                    predecessors(neighbor) = node;
                    unsettled{length(unsettled)+1} = neighbor;
                end
            end
        end
        
    end

    % now get path from start to goal
    if (~predecessors(goal_idx))
        path = [];
    else
        shortest_path = zeros(1, num_verticies);
        i = 1;
        current = goal_idx;
        while (current)
            shortest_path(i) = current;
            current = predecessors(current);
            i = i+1;
        end
        shortest_path = fliplr(shortest_path(:,1:i-1));

        path = zeros(length(shortest_path), 2);
        for j=1:length(shortest_path)
            path(j,:) = verticies(shortest_path(j),:);
        end

    end
    
end




%% PLOTTING %%%%%%%%%%%%%%%%%%%%%

function initializePlots()
    % clear figure 1
    figure(1);
    clf;
end

function plotObstacles(obstacles, color)

    for i = 1:length(obstacles)
        plotObstacle(obstacles{i}, color);
    end

end

function plotObstacle(obstacle, color)

    figure(1);
    hold on;

    fill(obstacle(:,1), obstacle(:,2),...
        color,'FaceColor','none','EdgeColor',color,'LineWidth',2);    
end

function plotPoints(points, color)

    for i = 1:length(points)
        plotPoint(points(i,:), color);
    end
    
end

function plotPoint(point, color)

    figure(1);
    hold on;

    plot(point(1), point(2), 'Color', color, 'Marker', '.');
    plot(point(1), point(2), 'Color', color, 'Marker', 'o');
    
end

function plotPaths(verticies, edges, color)

    figure(1);
    hold on;

    for i=1:length(edges)
        node1 = verticies(edges(i,1),:);
        node2 = verticies(edges(i,2),:);
        plot([node1(1);node2(1)],[node1(2);node2(2)],'Color',color);
    end

end

function plotPath(path, color)

    figure(1);
    hold on;
    
    plot(path(:,1),path(:,2),'Color',color);

end
