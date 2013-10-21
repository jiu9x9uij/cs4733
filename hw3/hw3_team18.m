% HW2 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

function hw3_team18(serPort)
    %% DESCRIPTION %%%%%%%%%%%%%%%%%%%
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% CONSTANTS %%%%%%%%%%%%%%%%%%%%%
    % 1 will mark empty, 2 will mark filled
    % 0 means that space was never traversed.
    gridSize = 21;                  %must be odd, robot starts in center ~10 meter grid
    robotDiameter = .335;           %height and width of each grid square
    robotRadius = robotDiameter/2;  % used for marking walls and stuff
    status = 1;                     % state machine!
    done = 0;                       % whether or not the program is done 
    maxSpiralSpeed = .4;            % when starting a spiral, the speed of turn
    currentSpiralSpeed = .4;        % keeping track of current speed
    maxFwdVel = .3;                 % maximium speed forward
    maxTimeWithoutImproving = 120;  % can go 2 minutes without improving grid before we call it off
    wallFollowThreshold = .2;       % how close we have to be for wall follow being back at start
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% SET UP/LOOP VARIABLES %%%%%%%%%%%%%%%%%%%
    grid = createGrid(gridSize);
    tStart = tic;       % total duration marker
    timeSinceNewSquare = tic; %time marking when we've last improved our grid
    lastHitPoint = [0,0];   %variable used for wall follow
    pos = [0,0,0];      % x,y,theta - current position and angle
    goalPoint = [0,0];  % used for going to a random spot on the grid
    isCloserOnMLine = false;    %used for m-line stuff
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    disp('-------- Begin Cover Algorithm --------');
    
    %% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%
    while(~done)
        pause(.1);
        % read all the sensors at once
        [BumpRight, BumpLeft, BumpFront, Wall, ~, ~, ...
        ~, ~, ~, ~, ~,~, ~, ~, ~, Dist, Angle, ...
        ~, ~, ~, ~, ~, pCharge]  = AllSensorsReadRoomba(serPort);
        
         % update angle
        pos(3) = mod(pos(3) + Angle, 2*pi);
        
        % update distance and position
        pos(1) = pos(1) + Dist * cos(pos(3));
        pos(2) = pos(2) + Dist * sin(pos(3));
        
        fprintf('POS: (%.3f,%.3f)  ANG: %.3f\n', pos(1), pos(2), pos(3));
        
        switch status
            case 1  % spiral until you find something
                currentSpiralSpeed ...
                = Spiral(currentSpiralSpeed, maxSpiralSpeed, maxFwdVel,serPort);
                
                if(BumpRight || BumpLeft || BumpFront)
                    SetFwdVelAngVelCreate(serPort,0,0);
                    if(seenWallBefore(grid, pos, robotDiameter))
                       status = 4; 
                    else
                        lastHitPoint = [pos(1),pos(2)];
                        status = 2; 
                        currentSpiralSpeed = maxSpiralSpeed;
                    end
                else
                    [grid, insideGrid, marked] = markEmpty(grid, pos, robotDiameter);
                    if(marked)
                        timeSinceNewSquare = tic;
                    end
                end
                
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if(Wall)
                    [grid, insideGrid, marked] = markFilled(grid, pos, robotDiameter, true);
                    if(~insideGrid)
                        status = 4;
                    end
                    if(marked)
                        %reset timer if found new stuff
                        timeSinceNewSquare = tic;
                    end
                end
                if (GetDistance(pos, lastHitPoint) > wallFollowThreshold)
                    status = 3;
                end
                
            case 3 % Wall Follow | Left the threshold of the hit point
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort);
                
                if(Wall)
                    [grid, insideGrid, marked] = markFilled(grid, pos, robotDiameter, true);
                    if(~insideGrid)
                        status = 4;
                    end
                    if(marked)
                        %reset timer if found new stuff
                        timeSinceNewSquare = tic;
                    end
                end
                if(GetDistance(pos, lastHitPoint) < wallFollowThreshold)
                   status = 4;
                end
                
            case 4 % pick random unseen spot and turn towards it
                [spot, found] = findEmptySpot(grid, robotDiameter);
                if(~found)
                    done = true;
                    disp('no more empty spots on grid!');
                else
                    pos(3) = turnToFacePoint(serPort, pos, spot);
                    status = 5;
                    goalPoint = spot;
                end
            case 5 % drive straight until you hit something or leave the area or get there... (spiral then)
                %reset timer if marked
                SetFwdVelAngVelCreate(serPort,maxFwdVel,0);
                isAtPoint = atPoint(pos, goalPoint, toc(tStart));
                if(isAtPoint)
                    status = 1;
                end
                if(BumpRight || BumpLeft || BumpFront)
                    if(seenWallBefore(grid, pos, robotDiameter))
                        status = 6; 
                    else
                        SetFwdVelAngVelCreate(serPort,0,0);
                        status = 2;
                    end
                    lastHitPoint = [pos(1), pos(2)];
                else
                    [grid, insideGrid, marked] = markEmpty(grid, pos, robotDiameter);
                    if(marked)
                        timeSinceNewSquare = tic;
                    end
                end
            case 6 % M-Line wall follow before threshold
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort);
                % check if we've reached the goal
                isAtGoal = atPoint(pos, goalPoint, toc(tStart));
                if(isAtGoal)
                    status = 1;
                end
                if(GetDistance(pos, lastHitPoint) > wallFollowThreshold)
                    status = 7;
                end
            case 7 % M-Line wall follow after threshold
                
                % check if we're on the M Line
                if (onLine(pos,lastHitPoint, goalPoint)) 

                    disp('On M Line');

                    % check if we're closer to the goal
                    toGoal = pdist([pos(1),pos(2);goalPoint(1),goalPoint(2)],...
                                   'euclidean');
                    original = pdist([lastHitPoint (1),lastHitPoint (2);goalPoint(1),goalPoint(2)],...
                                   'euclidean');
                    isCloserOnMLine = toGoal < original;

                    fprintf('qHit to goal: %.3fm\n', qHitToGoal);
                    fprintf('Curr to goal: %.3fm\n',  toGoal);

                    % if we're back at the start, we're trapped
                    if (atPoint(lastHitPoint, goalPoint, toc(tStart)))
                        disp('circumnavigated wall, trapped, setting goal to filled');
                        markFilled(grid, pos, robotDiameter, false);
                    end
                    if(isCloserOnMLine)
                        pos(3) = turnToFacePoint(serPort, pos, goalPoint);
                        status = 5;
                    end
                end
        end
        
        if(toc(timeSinceNewSquare) > maxTimeWithoutImproving)
            disp('It seems we haven''t improved grid in too long');
            done = true;
        end
        
        
        drawnow;
    end
    
    disp('-------- End Cover Algorithm --------');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


function seenBefore = seenWallBefore(grid, pos, robotDiameter)

    seenBefore = false;
    frontOfRobot = [pos(1) + (robotDiameter/2)*cos(pos(3)), pos(2) + (robotDiameter/2)*cos(pos(3))];
    [row,col] = translateCoordGridSpace(grid, frontOfRobot, robotDiameter);
    if(grid(row,col) ==2)
        seenBefore = true;
    end
    
end

function dist = GetDistance(point1, point2)

    dist = pdist([point1(1), point1(2); point2(1), point2(2)], 'euclidean'); 

end

function newRadsPerSecond = Spiral(currentRadsPerSecond, maxRadsPerSecond, maxFwdSpeed, serPort)
    rateOfDecrease = .997;
    newRadsPerSecond = currentRadsPerSecond * rateOfDecrease;
    v = maxFwdSpeed * (1- (currentRadsPerSecond/maxRadsPerSecond));
    SetFwdVelAngVelCreate(serPort, v, newRadsPerSecond );
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

function WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort)
% Wall follow from homework 1 solution

% constants taken from HW 1 solution, moved inside function
    velocity_val = 0.2;
    angular_velocity_val = 0.1;
    
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


function newGrid = createGrid(gridSize)
    newGrid = zeros(gridSize, gridSize);
end


function [grid, insideGrid, marked] = markEmpty(grid, pos, robotDiameter)
% Takes a coordinate position, finds the grid location
% and if the location isn't already marked as filled
% it marks it as empty. Marks all 4 corners of the robot
% 
% Input:
% grid - the current grid matrix
% pos - the x,y coordinates of the position to be resolved
% robotDiameter - the width and height of the grid squares
%
% Output:
% the new grid
    s = size(grid);
    s = s(1);
    marked = 0;
    insideGrid = false;
    i = 0;
    while(i < 4)
        angle = i*(pi/2) + pos(3);
        spot = [pos(1) + (robotDiameter/2)*cos(angle), pos(2) + (robotDiameter/2)*cos(angle)];
        [row, col] = translateCoordGridSpace(grid, spot, robotDiameter);
        if(col > s || row > s || col < 0 || row < 0)
            disp('part of robot in unknown area (not in grid)!');
        else
            insideGrid = true;
            if(grid(row,col)== 0)
                grid(row,col) = 1; 
                fprintf('empty!  - > row: %f col: %f\n',row,col);
                insideGrid = true;
                marked = true;
            end
        end
        i = i +1;
    end
    
end



function [grid, insideGrid, marked] = markFilled(grid, pos, robotDiameter, following)
% Takes a coordinate position, finds the grid location
% and marks the spot as filled regardless of if it was 
% previously marked as empty. It just marks the wall follow side of the bot
% 
% Input:
% grid - the current grid matrix
% pos - the x,y coordinates of the position to be resolved
% robotDiameter - the width and height of the grid squares
%
% Output:
% the new grid
    s = size(grid);
    s = s(1);
    marked = false;
    % to the right of the robot
    if(following)
        spot = [pos(1) + (robotDiameter/2)*cos(-pi/2), pos(2) + (robotDiameter/2)*cos(-pi/2)];
    else
        spot = pos;
    end
    
    [row, col] = translateCoordGridSpace(grid, spot, robotDiameter);
    if(col > s || row > s || col < 0 || row < 0)
        disp('tried to access unknown part of grid!');
        insideGrid = false;
    else
        insideGrid = true;
        if(grid(row,col)~=2)
            grid(row,col) = 2;
            fprintf('filled! - > row: %f col: %f\n',row,col);
            marked = true;
        end
    end
end

function [row, col] = translateCoordGridSpace(grid, pos, robotDiameter)
% Takes a coordinate position and resolves it to the grid
%
% Input:
% grid - the current grid matrix
% pos - the x,y coordinates of the position to be resolved
% robotDiameter - the width and height of the grid squares
%
% Output:
% [row,col] the grid resolved location of the point
    s = size(grid); 
    s = s(1);
    offset = s/2 + 1;
    col = floor(pos(1)/robotDiameter + offset); % x refers to column
    row = floor(pos(2)/robotDiameter + offset); % y refers to row
end

function spot = translateGridSpaceToCoord(grid, robotDiameter, row, col)
    s = size(grid);
    s = s(1);
    spot = [0,0];
    offset = s/2+1;
    spot(1) = (col-offset)*robotDiameter;
    spot(2) = (row-offset)*robotDiameter;
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

    distCushion = 0.15 + (duration/60)*0.03;
    dist = pdist([pos(1), pos(2); qGoal(1), qGoal(2)], 'euclidean'); 
    isAtPoint = dist < distCushion; 
    
end


function [spot,found] = findEmptySpot(grid,robotDiameter)
    s = size(grid);
    spot = [0,0];
    row = 1;
    col = 1;
    found = false;
    while(row <= s(1))
        while(col <= s(2))
            if(grid(row,col)==0)
                found = true;
            end
            col=col+1;
        end
        row=row+1;
    end
    if(found)
    successful = false;
        while(~successful)
           row = round(rand(1)*s(1));
           col = round(rand(1)*s(2));
           if(grid(row,col)==0)
              spot = translateGridSpaceToCoord(grid, robotDiameter, row, col)
              successful = true; 
           end
        end
    end
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

