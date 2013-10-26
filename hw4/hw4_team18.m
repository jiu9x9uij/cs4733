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
    visibility_graph = createVisibilityGraph(start, goal, obstacles);
    shortest_path = findShortestPath(start, goal, visibility_graph);
    
    % plot the paths
    plotPaths(visibility_graph, [0 0.5 0]); % all paths in dark green
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

    catch err
        disp('Failed to read obstacles from file');
        disp(err);
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
        
        % copy the obstacle
        grown_obs = obstacles{i};
        
        % push all points out from centroid
        [centroid_x, centroid_y] = centroid(grown_obs(:,1), grown_obs(:,2));
        for j = 1:length(grown_obs)
            grown_obs(j, 1) = grown_obs(j, 1)...
                                + (grown_obs(j, 1) - centroid_x)/3;
            grown_obs(j, 2) = grown_obs(j, 2)...
                                + (grown_obs(j, 2) - centroid_y)/3;

        end
        
        grown_obstacles{1,i} = grown_obs;
        
    end
    
end



%% FINDING SHORTEST PATH %%%%%%%%%%%%%%%%%%%%%

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

    fake_path = [start; -2.2428,1.1882; 1,2; 3,1; 4,3; goal];

    top_left_path = zeros(length(obstacles) + 2, 2);
    bottom_right_path = zeros(length(obstacles) + 2, 2);

    top_left_path(1,:) = start;
    top_left_path(length(top_left_path),:) = goal;
    bottom_right_path(1,:) = start;
    bottom_right_path(length(bottom_right_path),:) = goal;
    
    for i=1:length(obstacles)
        obstacle = obstacles{i};
        top_left_path(i+1,:) = obstacle(1,:);
        bottom_right_path(i+1,:) = obstacle(length(obstacle),:);
    end
    
    visibility_graph = cell(1, 3);
    visibility_graph{1} = fake_path;
    visibility_graph{2} = top_left_path;
    visibility_graph{3} = bottom_right_path;
    
end

function visibleVerticies()
%{
    Input. A set S of polygonal obstacles 
           and a point p that does not lie in theinterior of any obstacle.
    Output. The set of all obstacle vertices visible from p.
        1. Sort the obstacle vertices according to the clockwise angle 
           that the half-line from p to each vertex makes with the 
           positive x-axis. In case ofties, vertices closer to p should 
           come before vertices farther from p. 
           Letw1,...,wn be the sorted list.
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

function path = findShortestPath(start, goal, visibility_graph)

    path = visibility_graph{1};
    
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

function plotPaths(paths, color)

    for i = 1:length(paths)
        plotPath(paths{i}, color);
    end

end

function plotPath(path, color)

    figure(1);
    hold on;
    
    plot(path(:,1),path(:,2),'Color',color);

end

