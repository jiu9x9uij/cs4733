% HW4 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

function hw4_team18(serPort, world_file, start_goal_file)

    [wall, obstacles] = readWorldFromFile(world_file);
    [start, goal] = readStartGoalFromFile(start_goal_file);
    
    dispObj(wall, 'WALL');
    dispObs(obstacles);
    dispObj(start, 'START');
    dispObj(goal, 'GOAL');
    
end

function dispObj(obj, name)
    fprintf('********* %s ***********\n', name);
    disp(obj);
end

function dispObs(obstacles)
    fprintf('********* %s ***********\n', 'OBSTACLES');
    for i = 1:length(obstacles)
        disp(obstacles{i});
    end
end

function [wall, obstacles] = readWorldFromFile(file)

% representing a polygon in matlab:
% array of x-coords of verticies, array of y-coords of verticies


% the vertices follow, one per line, each with two coordinates
% you should close the obstacle by linking the last vertex to the first
% the first obstacle in the file actually specifies the wall that encloses the working environment.

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
