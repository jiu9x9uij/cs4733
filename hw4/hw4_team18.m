% HW4 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

%% MAIN METHOD %%%%%%%%%%%%%%%%%%%%%

function hw4_team18(serPort, world_file, start_goal_file)

    % constants
    robot_num_verts = 4;
    robot_mult = 1.1;
    robot_radius = 0.1675;

    % create the world
    [start, goal] = readStartGoalFromFile(start_goal_file);
    [wall, obstacles] = readWorldFromFile(world_file);
    robot_pts = buildRobot(robot_radius*robot_mult, robot_num_verts);
    grown_obstacles = growObstacles(obstacles, robot_pts);

    % plot the world
    initializePlots();
    plotPoints([start;goal], [1 0 0]);         % start and goal in red
    plotObstacle(wall, [0 0 0.5]);             % wall in dark blue
    plotObstacles(obstacles, [0 0.4 1]);       % obstacles in blue
    plotObstacles(grown_obstacles, [0, 0, 0]); % grown obstacles in black
    
    % find path to take
    [verticies, edges] = createVisibilityGraph(start, goal, grown_obstacles, wall);
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

function robot_pts = buildRobot(radius, num_verticies)

    degree = 2*pi/num_verticies;
    
    robot_pts = zeros(num_verticies, 2);
    for i=1:num_verticies
        ang = (i-1)*degree;
        robot_pts(i,:) = [radius*cos(ang),radius*sin(ang)];
    end

end



%% GROW OBSTACLES %%%%%%%%%%%%%%%%%%%%%

function grown_obstacles = growObstacles(obstacles, robot_pts)

    num_robot_pts = size(robot_pts,1);
    num_obstacles = size(obstacles,2);

    % "reflect" robot
    reference_point = robot_pts(1,:);
    for i=1:num_robot_pts
        robot_pts(i,:) = reference_point - robot_pts(i,:);
    end
    
    grown_obstacles = cell(1, num_obstacles);

    for i = 1:num_obstacles
        % grow verticies
        verticies = growVerticies(obstacles{i}, robot_pts);

        % grown obstacle is convex hull of new set of verticies
        grown_obstacles{1,i} = computeConvexHull(verticies);
    end
    
end

function verticies = growVerticies(obstacle, robot_pts)

    num_robot_pts = size(robot_pts,1);
    num_obs_verts = size(obstacle,1);

    % now stick reference point at every vertex, and grab set of all 
    verticies = zeros(num_robot_pts*num_obs_verts, 2);
    for i=1:num_obs_verts
        for j=1:num_robot_pts
            verticies((i-1)*num_robot_pts+j,:) = obstacle(i,:)+robot_pts(j,:);
        end
    end
    
    % remove any dup verticies
    verticies = unique(verticies, 'rows');

end

% TODO matlab has this built in: convhull(verticies(1:,),verticies(2,:))
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
        ang_stack = atan2(top(2)-snd(2),top(1)-snd(1));
        ang_point = atan2(point(2)-snd(2),point(1)-snd(1));
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



%% VISIBILITY GRAPH %%%%%%%%%%%%%%%%%%%%%

function [verticies, edges] = createVisibilityGraph(start, goal, obstacles, wall)

    num_obs = size(obstacles,2);

    num_verts = 2;
    for i=1:num_obs
        num_verts = num_verts + size(obstacles{i},1);
    end
    
    verticies = zeros(num_verts,2);
    verticies(1,:) = start;
    verticies(2,:) = goal;

    intra_obs_edges = zeros(num_verts, num_verts);
    
    idx = 2;
    for i=1:num_obs
        obstacle = obstacles{i};
        first_idx = idx+1;
        for j=1:size(obstacle,1)
            idx = idx+1;
            verticies(idx,:) = obstacle(j,:);
        end
        for k=first_idx:idx
            for l=k+2:idx
                intra_obs_edges(k,l) = 1;
            end
        end
        intra_obs_edges(first_idx, idx) = 0;
    end
    
    inside_obstacle = zeros(1,num_verts);
    for i=1:num_verts
        p1 = verticies(i,:);
        inside_obstacle(i) = 0;
        for k=1:num_obs
            if (insideObstacle(p1, obstacles{k}))
                inside_obstacle(i) = 1;
                break;
            end
        end
        if (~inside_obstacle(i))
            inside_obstacle(i) = ~insideObstacle(p1, wall);
        end
    end

    edges = zeros(num_verts*(num_verts-1)/2,3);
    idx = 0;
    for i=1:num_verts

        if (inside_obstacle(i))
            continue;
        end
        
        p1 = verticies(i,:);
        
        for j=i+1:num_verts
            
            if (inside_obstacle(j) || intra_obs_edges(i,j))
                continue;
            end

            p2 = verticies(j,:);
          
            intersects_obs = false;
            for k=1:num_obs
                if (intersectsObstacle(p1, p2, obstacles{k}))
                    intersects_obs = true;
                    break;
                end
            end
            
            if (~intersects_obs)
                idx = idx+1;
                edges(idx,:) = [i,j,pdist([p1;p2])];
            end
        end
    end
      
    % remove empty rows
    edges = edges(any(edges,2),:);
end

function [p, intersects] = intersectLines(p1, p2, p3, p4)
    t1 = p1(1) - p2(1);
    t2 = p3(1) - p4(1);
    t3 = p1(2) - p2(2);
    t4 = p3(2) - p4(2);
    t5 = p1(1)*p2(2) - p1(2)*p2(1);
    t6 = p3(1)*p4(2) - p3(2)*p4(1);
    t7 = t1*t4 - t3*t2;
    
    p = [(t5*t2 - t1*t6)/t7, (t5*t4 - t3*t6)/t7];
    intersects = t7 ~= 0;
end

function [p, intersects] = intersectSegments(p1, p2, p3, p4)
    
    if (isequal(p1, p3) || isequal(p1, p4))
        intersects = true;
        p = p1;
        return;
    end
    
    if (isequal(p2, p3) || isequal(p2, p4))
        intersects = true;
        p = p2;
        return;
    end

    [p, intersects] = intersectLines(p1, p2, p3, p4);

    if (intersects)
        d1 = pdist([p1;p2]);
        d2 = pdist([p3;p4]);

        if (d1 < pdist([p;p1]) || d1 < pdist([p;p2]) || ...
            d2 < pdist([p;p3]) || d2 < pdist([p;p4]))
            intersects = false;
        end
    end
end

function intersects = intersectsObstacle(p1, p2, obstacle)

    for j=1:size(obstacle,1)-1
        obs1 = obstacle(j,:);
        obs2 = obstacle(j+1,:);
        
        % can move along edge
        if ((isequal(p1, obs1) && isequal(p2, obs2)) ||...
            (isequal(p1, obs2) && isequal(p2, obs1)))
            intersects = false;
            return;
        end
        
        [p, intersects] = intersectSegments(p1, p2, obs1, obs2);
        
        if (intersects && ~isequal(p, p1) && ~isequal(p, p2))
            return;
        end
    end
   
    obs1 = obstacle(1,:);
    obs2 = obstacle(size(obstacle,1),:);
    if ((isequal(p1, obs1) && isequal(p2, obs2)) ||...
        (isequal(p1, obs2) && isequal(p2, obs1)))
        intersects = false;
        return;
    end
    [p, intersects] = intersectSegments(p1, p2, obs1, obs2);
    if (intersects && ~isequal(p, p1) && ~isequal(p, p2))
        return;
    end

    intersects = false;     

end

function inside = insideObstacle(p1, obstacle)

    [in, on] = inpolygon(p1(1),p1(2),obstacle(:,1),obstacle(:,2));

    inside = in && ~on;

end



%% DIJKSTRA %%%%%%%%%%%%%%%%%%%%%

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
            % check edge one way
            if (edge(1) == node && isempty(settled{edge(2)}))        
                neighbor = edge(2);                
                newDist = dist(node) + edge(3);
                if (dist(neighbor) > newDist)
                    dist(neighbor) = newDist;
                    predecessors(neighbor) = node;
                    unsettled{length(unsettled)+1} = neighbor;
                end
            % check edge other way
            elseif (edge(2) == node && isempty(settled{edge(1)}))        
                neighbor = edge(1);                
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


