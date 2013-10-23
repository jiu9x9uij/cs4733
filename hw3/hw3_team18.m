% HW3 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

function hw3_team18(serPort)
    %% DESCRIPTION %%%%%%%%%%%%%%%%%%%
    % Create a 2-D occupancy grid for robot environment by exploring,
    % and map it out with cells that are either occupied or empty.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%
    %=============================================================%
    % Clear cache & avoid NaN values                              %
    %=============================================================%
    clc;                                                          % Clear the cache
    
    % Poll for bump Sensors to avoid getting NaN values when the 
    % robot first hits a wall
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);
    %=============================================================%
    
    %% CONSTANTS %%%%%%%%%%%%%%%%%%%%%
    gridSizeWidth = 11;             % must be odd!! this is ~5m grid
    gridSizeHeight = 17;            % must be odd!! 
    robotDiameter = .335;           % height and width of each grid square
    status = 1;                     % state machine
    done = 0;                       % whether or not the program is done 
    maxSpiralSpeed = .4;            % max spiral turn speed (rad/s)
    maxFwdVel = .15;                % max speed forward (m/s)
    spiralFwdSpeed = .3;            % fwd vel used for spiral (m/s)
    maxTimeWithoutImproving = 180;  % die after 3 mins without grid change
    wallFollowThreshold = .2;       % how close we have to be for wall follow being back at start
    maxProgramDuration = 15*60;     % max allowed duration is 15 min
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%% REAL ROBOT %%%%%%%%%%%
    
    if isa(serPort,'CreateRobot')
        %constants are fine
    else
        maxSpiralSpeed = .4;            % max spiral turn speed (rad/s)
        maxFwdVel = .1;                % max speed forward (m/s)
        spiralFwdSpeed = .2;            % fwd vel used for spiral (m/s)
    end
    
    
    %% SET UP/LOOP VARIABLES %%%%%%%%%%%%%%%%%%%
    grid = zeros(gridSizeHeight, gridSizeWidth); % 0 unknown, 1 empty, 2 filled
    tStart = tic;                     % total duration marker
    timeSinceNewSquare = tic;         % time when we last changed grid
    lastHitPoint = [0,0];             % hit point for wall follow (x,y)
    pos = [0,0,0];                    % current robot pos (x,y,theta)
    goalPoint = [0,0];                % random point to go to (x,y)
    currentSpiralSpeed = .4;          % spiral needs to know current speed
    initializePlots(size(grid), robotDiameter);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    %% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%
    disp('-------- Begin Cover Algorithm --------');
    while(~done)
        
        % read all the sensors at once
        [BumpRight, BumpLeft, BumpFront, Wall, ~, ~, ...
        ~, ~, ~, ~, ~,~, ~, ~, ~, Dist, Angle, ...
        ~, ~, ~, ~, ~, ~]  = AllSensorsReadRoomba(serPort);
    
        % handle possible NaN
        while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
            Wall = WallSensorReadRoomba(serPort);
        end
        
        % update odometry and plot position
        pos(3) = mod(pos(3) + Angle, 2*pi);
        pos(1) = pos(1) + Dist * cos(pos(3));
        pos(2) = pos(2) + Dist * sin(pos(3));
        plotPosition(pos);
      
        switch status
            
            case 1  % spiral until you find something
                currentSpiralSpeed = Spiral(currentSpiralSpeed,... 
                                            maxSpiralSpeed,...
                                            spiralFwdSpeed, serPort);
                if (BumpRight || BumpLeft || BumpFront)
                    SetFwdVelAngVelCreate(serPort,0,0);
                    if (seenWallBefore(grid, pos, robotDiameter))
                       status = 4; % go to random point
                  
                    else
                        status = 2;  % start wall follow
                        disp('start wall follow');
                    end
                    lastHitPoint = [pos(1), pos(2)];
                    currentSpiralSpeed = maxSpiralSpeed;
                else
                    [grid, marked] = markEmpty(grid, pos, robotDiameter);
                    if (marked)
                        timeSinceNewSquare = tic; % reset timer
                    end
                end
                
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (Wall)
                    [grid, marked] = markFilled(grid, pos, robotDiameter, true);
                    if (marked)
                        timeSinceNewSquare = tic; % reset timer
                    end
                end
                if (getDistance(pos, lastHitPoint) > wallFollowThreshold)
                    disp('wall follow, left threshold');
                    status = 3; % wall follow, left threshold
                end
                
            case 3 % Wall Follow | Left the threshold of the hit point
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort);

                if (Wall)
                    [grid, marked] = markFilled(grid, pos, robotDiameter, true);
                    if (marked)
                        timeSinceNewSquare = tic; % reset timer
                    end
                end
                if(getDistance(pos, lastHitPoint) < wallFollowThreshold)
                   status = 4; % go to random point
                end
                
            case 4 % pick random unseen spot and turn towards it
                SetFwdVelAngVelCreate(serPort,0,0);
                [spot, found] = findEmptySpot(grid, robotDiameter);
                if (~found)
                    done = true;
                    disp('no more empty spots on grid!');
                else
                    pos(3) = turnToFacePoint(serPort, pos, spot);
                    status = 5; % straight to goal point
                    goalPoint = spot;
                end
                
            case 5 % drive straight to goal, unless you hit something or leave the area
                
                isAtPoint = atPoint(pos, goalPoint, toc(tStart));
                if (isAtPoint)
                    status = 1; % spiral
                end
                if (BumpRight || BumpLeft || BumpFront)
                    SetFwdVelAngVelCreate(serPort,0,0);
                    if (seenWallBefore(grid, pos, robotDiameter))
                        status = 6; % m-line wall follow
                    else
                        status = 2; % start wall follow
                    end
                    lastHitPoint = [pos(1), pos(2)];
                else
                    [grid, marked] = markEmpty(grid, pos, robotDiameter);
                    if (marked)
                        timeSinceNewSquare = tic; % reset timer
                    end
                    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);
                end
                
            case 6 % M-Line wall follow before threshold
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (Wall)
                    [grid, marked] = markFilled(grid, pos, robotDiameter, true);
                    if (marked)
                        timeSinceNewSquare = tic; % reset timer
                    end
                end
                % check if we're at the goal point
                isAtGoal = atPoint(pos, goalPoint, toc(tStart));
                if (isAtGoal)
                    status = 4; % go to random point
                end
                if (getDistance(pos, lastHitPoint) > wallFollowThreshold)
                    disp('m-line wall follow, left threshold');
                    status = 7; % m-line wall follow, left threshold
                end
                
            case 7 % M-Line wall follow after threshold
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (Wall)
                    [grid, marked] = markFilled(grid, pos, robotDiameter, true);
                    if (marked)
                        timeSinceNewSquare = tic; % reset timer
                    end
                end
                % check if we're on the M Line
                if (onLine(pos, lastHitPoint, goalPoint))
                    disp('On M Line');
                    % check if we're closer to the goal
                    toGoal = getDistance(pos, goalPoint);
                    original = getDistance(lastHitPoint, goalPoint);
                    if (toGoal < original)
                        pos(3) = turnToFacePoint(serPort, pos, goalPoint);
                        status = 5; % straight to goal point
                    end
                end
                % if we're back at the start, goal is unreachable
                if (getDistance(pos, lastHitPoint) < wallFollowThreshold)
                    disp('circumnavigated/trapped, setting goal to filled');
                    [grid, marked] = markFilled(grid, pos, robotDiameter, false);
                    if (marked)
                        timeSinceNewSquare = tic; % reset timer
                    end
                    status = 4; % go to random point
                end
        end
        
        if (toc(timeSinceNewSquare) > maxTimeWithoutImproving)
            disp('It seems we haven''t improved grid in too long.');
            done = true;
        end
        
        if (toc(tStart) > maxProgramDuration)
            disp('It''s been a long time, so let''s stop.');
            done = true;
        end
        
        drawnow;
    end 
    disp('-------- End Cover Algorithm --------');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % draw full grid now that we're done in case it was closed mid run
    for row = 1:gridSize(1);
        for col = 1:gridSize(2);
            updateGrid(row, col, grid(row, col), robotDiameter, gridSize);
        end
    end
    
end

%% COVER UTILITIES %%%%%%%%%%%%%%%%%%%%%

function seenBefore = seenWallBefore(grid, pos, robotDiameter)
% Check if we've seen a wall at the current position.
%
% Input:
% grid - the current grid matrix
% robotDiameter - the width and height of the grid squares
% pos - current robot position (x, y, angle)
%
% Output:
% seenBefore - true if we're current "close" to known obstacle

    s = size(grid);
    seenBefore = false;
    i = 0;
    
    % check front, left, right and back of robot for known obstacle
    while(i < 4)
        angle = i*(pi/2) + pos(3);
        spot = [pos(1) + (robotDiameter/2)*cos(angle),...
                pos(2) + (robotDiameter/2)*sin(angle)];
        [row, col] = translateCoordGridSpace(grid, spot, robotDiameter);
        if (col > s(2) || row > s(1) || col <= 0 || row <= 0)
            % do nothing
        elseif (grid(row,col)== 2)
            seenBefore = true;
            return;
        end
        i = i+1;
    end
    
    % check one extra point in front of robot, in case odometry is off
    frontOfRobot = [pos(1) + (robotDiameter)*cos(pos(3)),...
                    pos(2) + (robotDiameter)*sin(pos(3))];
    [row,col] = translateCoordGridSpace(grid, frontOfRobot, robotDiameter);
    if (col > s(2) || row > s(1) || col <= 0 || row <= 0)
        % do nothing
    elseif (grid(row,col) == 2)
        seenBefore = true;
    end
    
end

function newRadsPerSecond = Spiral(currentRadsPerSecond, maxRadsPerSecond, maxFwdSpeed, serPort)
% Drive in a spiral.
%
% Input:
% currentRadsPerSecond - new angular velocity (rad/s)
% maxRadsPerSecond - max angular velocity (rad/s)
% maxFwdSpeed - new linear velocity (m/s)
% serPort - Serial port for communicating with robot
%
% Output:
% newRadsPerSecond - new angular velocity (rad/s)

    % pause because spiral is time dependent
    pause(.1);
    rateOfDecrease = .997;
    if isa(serPort,'CreateRobot')
        %constants are fine
    else
        rateOfDecrease = .99;
    end

    % calculate spiral speeds
    newRadsPerSecond = currentRadsPerSecond * rateOfDecrease;
    v = maxFwdSpeed * (1- (currentRadsPerSecond/maxRadsPerSecond));

    % update velocities
    SetFwdVelAngVelCreate(serPort, v, newRadsPerSecond);
    
end

function [grid, marked] = markEmpty(grid, pos, robotDiameter)
% Takes a coordinate position, finds the grid location and if the location 
% isn't already marked as filled it marks it as empty. 
% Marks all 4 corners of the robot, not just robot center.
% 
% Input:
% grid - the current grid matrix
% pos - the x,y coordinates of the position to be resolved
% robotDiameter - the width and height of the grid squares
%
% Output:
% grid - the new grid matrix
% marked - true if grid was updated

    s = size(grid);
    marked = 0;
    i = 0;
    while (i < 4)
        angle = i*(pi/2) + pos(3);
        spot = [pos(1) + (robotDiameter/2)*cos(angle), ...
                pos(2) + (robotDiameter/2)*sin(angle)];
        [row, col] = translateCoordGridSpace(grid, spot, robotDiameter);
        if (col > s(2) || row > s(1) || col <= 0 || row <= 0)
            disp('robot in unknown area (not in grid)!');
            %return;
        else
            if (grid(row,col)== 0)
                grid(row,col) = 1; 
                fprintf('empty!  - > row: %f col: %f\n',row,col);
                marked = true;
                updateGrid(row, col, 1, robotDiameter, s);
            end
        end
        i = i+1;
    end
    
end

function [grid, marked] = markFilled(grid, pos, robotDiameter, following)
% Takes a coordinate position, finds the grid location and marks the spot 
% as filled regardless of if it was previously marked as empty. 
% Marks only the right side of the robot, not the center.
% 
% Input:
% grid - the current grid matrix
% pos - the x,y coordinates of the position to be resolved
% robotDiameter - the width and height of the grid squares
% following - true if wall following, in which case mark point to right
%
% Output:
% grid - the new grid matrix
% marked - true if grid was updated

    s = size(grid);
    marked = false;
    
    % to the right of the robot
    if (following)
        spot = [pos(1) + (robotDiameter/2)*cos(pos(3)-pi/2),...
                pos(2) + (robotDiameter/2)*sin(pos(3)-pi/2)];
    else
        spot = pos;
    end
    
    [row, col] = translateCoordGridSpace(grid, spot, robotDiameter);
    if (col > s(2) || row > s(1) || col <= 0 || row <= 0)
        disp('tried to access unknown part of grid!');
    else
        if (grid(row,col)~=2)
            grid(row,col) = 2;
            fprintf('filled! - > row: %f col: %f\n',row,col);
            marked = true;
            updateGrid(row, col, 2, robotDiameter, s);
        end
    end
end

function [row, col] = translateCoordGridSpace(grid, pos, robotDiameter)
% Takes a coordinate position and resolves it to the grid.
%
% Input:
% grid - the current grid matrix
% pos - the x,y coordinates of the position to be resolved
% robotDiameter - the width and height of the grid squares
%
% Output:
% row - grid row number
% col - grid column number

    height = size(grid,1);
    width = size(grid,2);
    offsetHeight = height/2 + 1;
    offsetWidth = width/2 + 1;
    col = floor(pos(1)/robotDiameter + offsetWidth); % x refers to column
    row = floor(pos(2)/robotDiameter + offsetHeight); % y refers to row
    
end

function spot = translateGridSpaceToCoord(grid, robotDiameter, row, col)
% Takes a grid position and resolves it to coordinates.
%
% Input:
% grid - the current grid matrix
% row - grid row number
% col - grid column number
% robotDiameter - the width and height of the grid squares
%
% Output:
% spot - the x,y coordinates of the position to be resolved
    height = size(grid,1);
    width = size(grid,2);
    offsetHeight = height/2 + 1;
    offsetWidth = width/2 + 1;
    spot = [(col-offsetWidth)*robotDiameter, (row-offsetHeight)*robotDiameter];
    
end

function [spot, found] = findEmptySpot(grid, robotDiameter)
% Finds a random unexplored point on the grid.
%
% Input:
% grid - the current grid matrix
% robotDiameter - the width and height of the grid squares
%
% Output:
% spot - random empty coordinate spot
% found - true if a spot was found

    s = size(grid);
    spot = [0,0];
    found = false;
    
    % first check if any spots are unknown
    for row = 1:s(1);
        for col = 1:s(2);
            if (grid(row, col)==0)
                found = true;
                break;
            end
        end
        if (found)
            break;
        end
    end
    
    if (found)
        successful = false;
        while (~successful)
           row = round(rand(1)*s(1));
           col = round(rand(1)*s(2));
           if (row == 0) 
               row = 1; 
           end
           if (col == 0) 
               col = 1; 
           end
           if (grid(row,col)==0)
              spot = translateGridSpaceToCoord(grid, robotDiameter, row, col);
              successful = true; 
           end
        end
    end
end

function WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort)
% Wall follow from homework 1 solution

    % Constants
    velocity_val = 0.2;
    angular_velocity_val = 0.1;
    if isa(serPort,'CreateRobot')
        %constants are fine
    else
        velocity_val = 0.1;
    end
    
    % Angle Velocity for different bumps
    av_bumpright =  4 * angular_velocity_val;
    av_bumpleft  =  2 * angular_velocity_val;
    av_bumpfront =  3 * angular_velocity_val;
    av_nowall    = -4 * angular_velocity_val;
    
    if BumpRight || BumpLeft || BumpFront
        v = 0;                              % Set Velocity to 0
    elseif ~Wall
        v = 0.25 * velocity_val;            % Set Velocity to 1/4 of the default
    else
        v = velocity_val;                   % Set Velocity to the default
    end

    if BumpRight
    av = av_bumpright;                      % Set Angular Velocity to av_bumpright
    elseif BumpLeft
        av = av_bumpleft;                   % Set Angular Velocity to av_bumpleft
    elseif BumpFront
        av = av_bumpfront;                  % Set Angular Velocity to av_bumpfront
    elseif ~Wall
        av = av_nowall;                     % Set Angular Velocity to av_nowall
    else
        av = 0;                             % Set Angular Velocity to 0
    end
    SetFwdVelAngVelCreate(serPort, v, av );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% BASIC UTILITIES %%%%%%%%%%%%%%%%%%%%%

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

function angTurned = turnRadians(serPort, angToTurn)
% Perform an acurate in place turn.
%
% Input:
% serPort - Serial port for communicating with robot
% angToTurn - Angle to turn, positive is counter-clockwise (rad)
%
% Output:
% angTurned - Actual angle turned (rad)

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

function isAtPoint = atPoint(pos, qGoal, duration)
% Determine if we're at goal position, which depends on program duration.
%
% Input:
% pos - Current position of robot (x, y, theta)
% qGoal - Goal position (x, y)
% duration - How long program has been running (s)
%
% Output:
% isAtPoint - True if robot is considered at goal

    distCushion = 0.2 + (duration/60)*0.04;
    dist = getDistance(pos, qGoal); 
    isAtPoint = dist < distCushion; 
    
end

function isOnLine = onLine(pos, qStart, qGoal)
% Determine if robot on line from start to goal.
%
% Input:
% pos - Current position of robot (x, y, theta)
% qStart - Initial position (x, y)
% qGoal - Goal position (x, y)
%
% Output:
% isOnLine - True if current position on line from start to goal

    % constants
    allowableSlopeDiff = 0.05;

    % calculate slopes
    startToCurSlope = (pos(2) - qStart(2))/(pos(1) - qStart(1));
    curToGoalSlope = (qGoal(2) - pos(2))/(qGoal(1) - pos(1));
    
    % determine if on line
    isOnLine = abs(startToCurSlope - curToGoalSlope) < allowableSlopeDiff;

end

function newAng = turnToFacePoint(serPort, pos, qGoal)
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% PLOTTING %%%%%%%%%%%%%%%%%%%%%

function initializePlots(grid_size, robot_size)
% Set up plot for current odometry and occupancy grid.
%
% Input:
% grid_size - number of grid squares (assume grid is square)
% robot_size - diameter of robot (m)

    width = robot_size*grid_size(2)/2 + 1;
    height = robot_size*grid_size(1)/2 + 1;
    % create position figure
    figure(1);
    hold on;
    axis([-width width -height height]);

    % create occupancy figure
    figure(2);
    hold on;
    clf;
    axis([-width width -height height]);

    % draw initial occupancy grid
    for row = 1:grid_size(1);
        for col = 1:grid_size(2);
            
            % x and y are bottom left corner of rectangle to draw
            x = robot_size * (col - grid_size(2)/2 - 1);
            y = robot_size * (row - grid_size(1)/2 - 1);
            
            % initially everything is unknown, so gray
            rectangle('position',  [x, y, robot_size, robot_size], ...
                      'edgecolor', [0, 0, 0], ...
                      'facecolor', [0.5, 0.5, 0.5]);
        end
    end

end

function plotPosition(pos)
% Prints x,y,theta position in readible format.
% Plots the position in blue and orientation in green.
%
% Input:
% pos - Position to display

    % print position and orientation
    % fprintf('POS: (%.3f, %.3f) ANG: %.3f\n',pos(1),pos(2),pos(3)*(180/pi));

    % draw odometry on figure 1
    figure(1);
    hold on;

    % plot position
    plot(pos(1), pos(2), 'b.');
    
    % plot orientation
    dispOrientation = 0.25;
    plot([pos(1),pos(1)+dispOrientation*cos(pos(3))], ...
         [pos(2),pos(2)+dispOrientation*sin(pos(3))], 'g');
    
end

function updateGrid(row, col, value, robot_size, grid_size)
% Update square in occupancy grid plot.
%
% Input:
% row - row to update
% col - col to update
% value - new value of grid square
% robot_size - diameter of robot (m)
% grid_size - number of grid squares (assume grid is square)

    % draw grid on figure 2
    figure(2);
    hold on;

    color = [0.5, 0.5, 0.5]; % unknown is gray
    if value == 1
        color = [1, 0.5, 0]; % empty is orange
    elseif value == 2
        color = [1, 1, 1];   % filled is white
    end

    % x and y are bottom left corner of rectangle to draw
    x = robot_size * (col - grid_size(2)/2 - 1);
    y = robot_size * (row - grid_size(1)/2 - 1);

    rectangle('position',  [x, y, robot_size, robot_size], ...
              'edgecolor', [0, 0, 0], ...
              'facecolor', color);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
