% HW4 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

%% MAIN METHOD %%%%%%%%%%%%%%%%%%%%%

function hw4_team18(serPort, world_file, start_goal_file)

    % constants
    robot_num_verts = 8;
    robot_mult = 1.25;
    robot_radius = 0.1675;

    % create the world
    [start, goal] = readStartGoalFromFile(start_goal_file);
    [wall, obstacles] = readWorldFromFile(world_file);
    robot_pts = buildRobot(robot_radius*robot_mult, robot_num_verts);
    grown_obstacles = growObstacles(obstacles, robot_pts);

    % plot the world
    initializePlots();
    plotPoint(start, [1 0 0]);                 % start in red
    plotPoint(goal, [0 0 1]);                  % goal in blue
    plotObstacle(wall, [0 0 0.5]);             % wall in dark blue
    plotObstacles(obstacles, [0 0.4 1]);       % obstacles in blue
    plotObstacles(grown_obstacles, [0, 0, 0]); % grown obstacles in black
    
    % find path to take
    [verticies, edges] = createVisibilityGraph(start, goal, grown_obstacles, wall);
    shortest_path = findShortestPath(verticies, edges, start, goal);

    disp(shortest_path);
    
    % plot the paths
    plotPaths(verticies, edges, [0 0.5 0]); % all paths in dark green
    plotPath(shortest_path, [1 0 0], 2);    % shortest path in red
    
    % run robot if there is a serPort
    if (serPort ~= 0)
        runRobot(serPort, shortest_path, obstacles);
    end
    
end



%% SETTING UP ENVIRONMENT %%%%%%%%%%%%%%%%%%%%%

function [wall, obstacles] = readWorldFromFile(file)
% reads obstacles and wall from file into usable matricies

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
% read one obstacle from file into a useable matrix

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
% read start and goal points from file

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
% construct set of points that represent robot, generally circular
% for example if num_verticies is 8, will create an octagon

    degree = 2*pi/num_verticies;
    
    robot_pts = zeros(num_verticies, 2);
    for i=1:num_verticies
        ang = (i-1)*degree;
        robot_pts(i,:) = [radius*cos(ang),radius*sin(ang)];
    end

end



%% GROW OBSTACLES %%%%%%%%%%%%%%%%%%%%%

function grown_obstacles = growObstacles(obstacles, robot_pts)
% reflection algorithm to grow obstacles
% picks right-most robot point as reference point

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
        verticies = growVerticies(obstacles{i}, robot_pts, reference_point);

        % grown obstacle is convex hull of new set of verticies
        grown_obstacles{1,i} = computeConvexHull(verticies);
    end
    
end

function verticies = growVerticies(obstacle, robot_pts, ref_pt)
% for one specific object, use reflection algorithm to grow verticies

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

function grown_obstacle = computeConvexHull(verticies)
% given set of any verticies, computes convex hull polygon

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
% given start and goal points, set of obstacles and wall, creates
% visibility graph, which represents all points that can "see" each
% other in free space.

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
            
            if (~intersects_obs && ~intersectsObstacle(p1, p2, wall))
                idx = idx+1;
                edges(idx,:) = [i,j,pdist([p1;p2])];
            end
        end
    end
      
    % remove empty rows
    edges = edges(any(edges,2),:);
end

function [p, intersects] = intersectLines(p1, p2, p3, p4)
% check to see if two lines intersect

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
% check to see if two line segments intersect   

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

        myEps = 1e-12; % matlab "eps" is too small
        
        % use eps to account for floating point rounding errors
        if (pdist([p;p1]) - d1 > myEps || pdist([p;p2]) - d1 > myEps || ...
            pdist([p;p3]) - d2 > myEps || pdist([p;p4]) - d2 > myEps)
            intersects = false;
        end
    end
end

function intersects = intersectsObstacle(p1, p2, obstacle)
% check to see if line from p1 to p2 intersects the obstacle

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
% check to see if point p1 is inside the obstacle

    [in, on] = inpolygon(p1(1),p1(2),obstacle(:,1),obstacle(:,2));
    inside = in && ~on;
end



%% DIJKSTRA %%%%%%%%%%%%%%%%%%%%%

function path = findShortestPath(verticies, edges, start, goal)
% dijkstra's shortest path algorithm
% verticies is a list of (x, y) pairs
% edges is a list of (vert_idx1, vert_idx2, cost)
% start and goal are (x, y) points

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

function plotPath(path, color, lineWidth)

    figure(1);
    hold on;
    
    plot(path(:,1),path(:,2),'Color',color,'LineWidth',lineWidth);

end



%% ROBORACE %%%%%%%%%%%%%%%%%%%%%

function runRobot(serPort, points, obstacles)
% points has 2 columns and some number of rows
% first row is current point
% following rows are points to go to, in order
% last row is goal point

    % Clear the cache
    clc;                                                          

    % Poll for bump Sensors to avoid getting NaN values and
    % clear the distance and angle sensors
    BumpsWheelDropsSensorsRoomba(serPort);
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort); 
    
    % current position and orientation
    pos = [points(1,1), points(1,2), 0];
    qGoal = points(size(points,1),:);
    %turn to face the first point
    [pos(3),~] = turnToFacePoint(serPort, pos, points(2,:));
    
    %%loop variables
    atgoal = false;
    status = 1; % drivin straight
    currentStraightDist = 0;
    currentDeltaAngle = 0;
    idx = 3;
    recentLeaveDist = 0;
    distToLine = 0; % if we bump we need this guy
    compensateTurn = 1; %same deal, for bump stuff
    
    %%first calculation
    [data, recentLeaveDist] = computeDistAndAngle(serPort, points(idx-2,:), ...
        points(idx-1,:), points(idx,:), recentLeaveDist, pos(3));
    
    %%begin main loop
    while (~atgoal)

        Dist = DistanceSensorRoomba(serPort);
        Angle = AngleSensorRoomba(serPort);  
        Bump = checkForBump(serPort);
        
        % update odometry and plot position
        pos(3) = mod(pos(3) + Angle, 2*pi);
        pos(1) = pos(1) + Dist * cos(pos(3));
        pos(2) = pos(2) + Dist * sin(pos(3));
        
        % print position
        fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));
        
        if (Bump && ~(status==4))
            disp('oh no bumped!!');
            stopRobot(serPort);
            a = points(idx-2,:);
            b = points(idx-1,:);
            c = points(idx,:);
            pos(1,1:2) = getClosestPoint(obstacles,pos);
            [whereIProbablyShouldBe, distToLine] = pointOnLine(a,b,pos(1,1:2));
            %tricky because we have to get back into the system
            [pos(3), compensateTurn] = turnToFacePoint(serPort, pos, whereIProbablyShouldBe);
            currentStraightDist = 0;
            status = 4;
            AngleSensorRoomba(serPort); %just to clear this shizzle cuz who knows
            DistanceSensorRoomba(serPort);
            disp('CHANGE TO STATE 4');
            Bump = checkForBump(serPort);
        end
        
        breakEarlyAngCompensate = 5*pi/180;
        breakEarlyDistCompensate = 0.1;
        
        switch status
                
            case 1 % driving straight
                moveForward(serPort);
                
                currentStraightDist = currentStraightDist + Dist;
                if(currentStraightDist >= data(1) - breakEarlyDistCompensate)
                   currentStraightDist = 0;
                   status = 2;
                   disp('CHANGE TO STATE 2');
                end
                
            case 2 % turning
                
                %how far we've turned in this particular arc
                currentDeltaAngle = currentDeltaAngle + Angle;
                
                if (abs(currentDeltaAngle) >= abs(data(2)*(pi/180))-breakEarlyAngCompensate)
                    moveForward(serPort);
                    currentDeltaAngle = 0;
                    idx = idx + 1;
                    %this means we have only to head straight to the goal
                    if (idx > size(points,1))
                        status = 3;
                        data(1) = getDistance(pos,qGoal);
                        disp('CHANGE TO STATE 3');
                    else
                        % calculate data with current pos as first point
                        [data, recentLeaveDist] = computeDistAndAngle(serPort, pos, points(idx-1,:), points(idx,:), recentLeaveDist, pos(3));
                        status = 1;
                        disp('CHANGE TO STATE 1');
                    end
                else
                    positive = data(2) > 0;
                    turnRobot(serPort, positive);
                end
            case 3 %driving straight for some distance to the goal
                moveForward(serPort);
                currentStraightDist = currentStraightDist + Dist;
                %might as well limit our distance
                if(currentStraightDist >= data(1))
                   atgoal = true; 
                end
            case 4 %try and get to the line we should be on
                moveForward(serPort);
                currentStraightDist = currentStraightDist + Dist;
                if(Bump)
                    stopRobot(serPort);
                    turnRadians(serPort, compensateTurn/4);
                    AngleSensorRoomba(serPort); %clear it so the pos(3) never gets updated
                end
                if(currentStraightDist >= distToLine)
                   stopRobot(serPort);
                   status = 5;
                   disp('CHANGE TO STATE 5');
                end
            case 5 %turn to face "point b", drive to point b, turn to c
                pos(3) = turnToFacePoint(serPort, pos, b);
                quickDist = driveDistance(serPort, norm(b-pos(1,1:2)));
                pos(1) = pos(1) + quickDist * cos(pos(3));
                pos(2) = pos(2) + quickDist * sin(pos(3));
                pos(3) = turnToFacePoint(serPort, pos, c);
                idx = idx + 1;
                %this means we have only to head straight to the goal
                if (idx > size(points,1))
                    status = 3;
                    data(1) = getDistance(pos,qGoal);
                    disp('CHANGE TO STATE 3');
                else
                    % calculate data with current pos as first point
                    [data, recentLeaveDist] = computeDistAndAngle(serPort,...
                        pos, points(idx-1,:), points(idx,:), 0, pos(3));
                    status = 1;
                    disp('CHANGE TO STATE 1');
                end
        end
    end
    
    disp('DONE!');
    stopRobot(serPort);

end

function Bump = checkForBump(serPort)
% simply reads the bump sensors
%
% Input:
% serPort - for robot access
%
% Output: 
% whether or not any of the bump sensors are true
    
    [BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

    % handle possible NaN
    while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
        [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
            = BumpsWheelDropsSensorsRoomba(serPort);
    end

    Bump = BumpRight || BumpLeft || BumpFront;
end

function turnRobot(serPort, positive) 
% turns the robot
%
% Input:
% serPort - for robot access
% positive - whether to turn positive or negative
%
% Output: 
% none
    
    turnSpeed = .3; % in rads/s
    turnFwdSpeed = .1; % in rads/s
    if(positive)
        SetFwdVelAngVelCreate(serPort,turnFwdSpeed,turnSpeed);
    else
        SetFwdVelAngVelCreate(serPort,turnFwdSpeed,-turnSpeed);
    end

end

function distGone = driveDistance(serPort, totalDist)
% drives a certain distance and stops
%
% Input:
% serPort - for robot access
% totalDist - distance to travel before stopping
%
% Output: 
% actual distance traveled    

    done = false;
    distGone = 0;
    while(~done)
        dist = DistanceSensorRoomba(serPort);
        distGone = distGone + dist;
        moveForward(serPort); 
        if(distGone >= totalDist)
            done = true;
        end
    end
    stopRobot(serPort);
end

function stopRobot(serPort)
% stops the robot
%
% Input:
% serPort - for robot access
%
% Output: 
% none
    
    SetFwdVelAngVelCreate(serPort,0.00001,0.00001);
end

function moveForward(serPort)
% drive straight with compensation
%
% Input:
% serPort - for robot access
%
% Output:
% none
    
    angSpeedCompensate = .02713;
    fwdSpeed = .3;
    if isa(serPort,'CreateRobot')
        angSpeedCompensate = 0;
        fwdSpeed = .4;
    end
    SetFwdVelAngVelCreate(serPort,fwdSpeed,angSpeedCompensate);
end

function [data, newRecentLeave] = computeDistAndAngle(serPort, A, B, C, recentLeaveDist, angle)
% Compute dist to travel and amount to turn for corner cutting system
%
% Input:
% serPort for checking if real robot
% A,B,C - points, B is the center point
% recentLeaveDist - how far onto the line A-B we already are
% angle - current Angle
%
% Output:
% data -dist to travel and amount to turn
% newRecentLeave - how far onto new line we will end up

% we can break this up into a series of distances and angles
% the distances represent driving straight, the angles are
% using our "standard" turn radius which can be set
% depending on our turn radius, the distances changes

    %stuff for the real robot
    measuredTurnDiameter = .98; %old.. 1.1938
    robotDiameter = .335;
    turnRadius = (measuredTurnDiameter-robotDiameter)/2;
    
    if isa(serPort,'CreateRobot')
        %set the constants for simulator only
        turnSpeed = .3; % must be same as above in turnrobot
        turnFwdSpeed = .1; % must be same as above in turnrobot
        turnRadius = (turnFwdSpeed * ((2*pi)/turnSpeed))/pi/2;
    end
    
    %because we might not be facing exactly the right way, we have to
    %modify our point B to B' that we will actually hit given our
    %trajectory

    p = A(1,1:2);
    q = C;
    r = [cos(angle), sin(angle)];
    s = B - C;
    
    %find t and u such that p + t r = q + u s
    %t = (q ? p) × s / (r × s)
    t = cross2d((q-p), (s / cross2d(r,s)));
    
    newB = p + t*r;
    
    data = zeros(1,2);

    myangle = computeAngleBetweenPoints(A,newB,C);
    data(2) = myangle;

    mydist = getDistance(A,newB);
    leaveDist = computeLeaveLineDist(myangle,turnRadius);
    mydist = mydist - leaveDist - recentLeaveDist;
    newRecentLeave = leaveDist;

    data(1) = mydist;
end

function result = cross2d(v1,v2)
    result = v1(1) * v2(2) - v1(2)*v2(1);
end

function angle = computeAngleBetweenPoints(A, B, C)
% Find angle between three points
%
% Input:
% A,B,C - points, B is the center point
%
% Output:
% angle - angle in radians

    ABrise = B(2)-A(2);
    ABrun = B(1)-A(1);
    ABangle = atan2d(ABrise,ABrun);
    
    BCrise = C(2)-B(2);
    BCrun = C(1)-B(1);
    BCangle = atan2d(BCrise,BCrun);
    
    angle = BCangle-ABangle;
    if(angle > 180)
       angle = 180 - angle; 
    elseif(angle < -180)
        angle = 360 + angle;
    end

end

function dist = computeLeaveLineDist(angle, radius)
    angle = abs(angle/2);
    dist = tand(angle)*radius;
end

function mylikelypoint = getClosestPoint(obstacles,pos)
% Find likely collision vertex
%
% Input:
% obstacles - coordinates of the obstacles
% pos - where we thought we were
%
% Output:
% mylikelypoint - closest vertex in obstacles to pos
    
    bestDistance = 1000000;
    mylikelypoint = [0,0];
    for i= 1:size(obstacles,2)
        obstacle = obstacles{i};
        for j=1:size(obstacle,1)
            vertex = obstacle(j,:);
            currentDist = getDistance(pos,vertex);
            if(currentDist < bestDistance)
                bestDistance = currentDist;
                mylikelypoint = vertex;
            end
        end
    end
end

function [intersection,distanceToLine] = pointOnLine(lineStart, lineEnd, point)
% Closest point on a line from another point
%
% Input:
% lineStart - first point (x,y) of line
% lineEnd - second point (x,y) of line
% point - the point not on the line
%
% Output:
% intersection - closest point on the line
% distanceToLine - distance from point to intersection

    line = (lineEnd-lineStart);
    line = line/norm(line);
    
    r = lineStart - point;
    v = [line(2), -line(1)]; %perp to line
    
    %make sure v is pointing from point
    currentDist = getDistance(point, lineStart);
    newDist = getDistance(point + v,lineStart);
    if(newDist > currentDist)
        v = v*-1;
    end
    
    distanceToLine = dot(v,r);
    intersection = point + v*distanceToLine;
    distanceToLine = abs(distanceToLine);

end

function [newAng, compensateAng] = turnToFacePoint(serPort, pos, qGoal)
% Turn in place to face goal.
%
% Input:
% serPort - Serial port for communicating with robot
% pos - Current position of robot (x, y, theta)
% qGoal - Goal position (x, y)
%
% Output:
% newAng - Angle of robot after completing turn

    disp('Starting turnToFacePoint');

    % calculate what we need to turn
    compensateAng = atan((pos(2)-qGoal(2))/(pos(1)-qGoal(1))) - pos(3);
        
    % if we're to the right of the goal, turn an extra 180
    if (pos(1) >= qGoal(1))
    	compensateAng = compensateAng+pi;
    end
    
    % if we're at the goal, inverse tan will fail, so manually set to 0
    if (pos(1) == qGoal(1) && pos(2) == qGoal(2))
        compensateAng = 0;
    end
    
    % do the turn and update the angle
    actualTurn = turnRadians(serPort, compensateAng);
    newAng = mod(pos(3) + actualTurn, 2*pi);
    disp('Completed turnToFacePoint');

end

function angTurned = turnRadians(serPort, angToTurn)
% Perform an acurate in place turn.
%
% Input:
% serPort - Serial port for communicating with robot
% angToTurn - Angle to turn, positive is counter-clockwise (rad)
%
% Output:
% angTurned - Actual angle turned (rad)

    % compensate for real robot
    angToTurn = 0.9*angToTurn;

    % constants
    turnSpeed = 0.35; % turn angle speed (rad/s)

    % if turning clockwise, use negative speed and positive angle
    if (angToTurn < 0)
        turnSpeed = -turnSpeed;
        angToTurn = -angToTurn;
    end
    
    % if turning more than half way, turn the other way
    if (angToTurn > pi)
        angToTurn = 2*pi - angToTurn;
        turnSpeed = -turnSpeed;
    end
    
    % reset angle sensor
    angTurned = AngleSensorRoomba(serPort);
    
    % start turning
    SetFwdVelAngVelCreate(serPort, 0, turnSpeed);
    
    % loop until turn complete
    while (abs(angTurned) < angToTurn)
        pause(0.01);
        angTurned = angTurned + AngleSensorRoomba(serPort);
    end
    
    % stop turning
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    % pause in case robot still turning a little
    pause(0.2);
    
    % reset angle sensor and update angle
    angTurned = angTurned + AngleSensorRoomba(serPort);
    
end

function dist = getDistance(point1, point2)
% Euclidean distance between two points.
%
% Input:
% point1 - first point (x,y)
% point2 - second point (x,y)
%
% Output:
% dist - distnace between points (m)

    dist = pdist([point1(1),point1(2);point2(1),point2(2)], 'euclidean'); 

end
