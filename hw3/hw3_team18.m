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
    robotDiameter = .4;             %height and width of each grid square
    robotRadius = robotDiameter/2;  % used for marking walls and stuff
    status = 1;                     % state machine!
    done = 0;                       % whether or not the program is done 
    maxSpiralSpeed = .4;            % when starting a spiral, the speed of turn
    currentSpiralSpeed = .4;        % keeping track of current speed
    maxFwdVel = .3;                 % maximium speed forward
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% SET UP %%%%%%%%%%%%%%%%%%%
    grid = createGrid(gridSize);
    tStart = tic;       % time limit marker
    pos = [0,0,0];      % x,y,theta - current position and angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
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
                    status = 2; 
                else
                    [grid, insideGrid, marked] = markEmpty(grid, pos, robotDiameter);
                    %reset timer if marked
                end
                
            case 2  % wall follow
                %hit point from case 1 or case 4 
                WallFollow(BumpRight, BumpLeft, BumpFront, Wall, serPort)
                %reset timer if marked during wall follow
                
                %copy hw1 solution cases
                
            case 3 % pick random unseen spot and turn towards it
                [spot, found] = findEmptySpot(grid);
                if(~found)
                    done = true;
                else
                    pos(3) = turnToFacePoint(serPort, pos, spot);
                    status = 4;
                end
            case 4 % drive straight until you hit something or leave the area or get there... (spiral then)
                %reset timer if marked
                
        end
        drawnow;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


function newRadsPerSecond = Spiral(currentRadsPerSecond, maxRadsPerSecond, maxFwdSpeed, serPort)
    rateOfDecrease = .98;
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
% it marks it as empty
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
    [row, col] = translateCoordGridSpace(grid, pos, robotDiameter);
    if(col > s || row > s || col < 0 || row < 0)
        disp('tried to access unknown part of grid!');
        insideGrid = false;
    else
        if(grid(row,col)== 0)
            grid(row,col) = 1; 
            fprintf('empty!  - > row: %f col: %f\n',row,col);
            insideGrid = true;
            marked = true;
        end
    end
end

function [grid, insideGrid] = markFilled(grid, pos, robotDiameter)
% Takes a coordinate position, finds the grid location
% and marks the spot as filled regardless of if it was 
% previously marked as empty
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
    [row, col] = translateCoordGridSpace(grid, pos, robotDiameter);
    if(col > s || row > s || col < 0 || row < 0)
        disp('tried to access unknown part of grid!');
        insideGrid = false;
    else
        grid(row,col) = 2;
        fprintf('filled! - > row: %f col: %f\n',row,col);
        insideGrid = true;
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


function [spot,found] = findEmptySpot(grid)
    s = size(grid);
    row = 1;
    col = 1;
    found = false;
    while(row <= s(1))
        while(col <= s(2))
            if(grid(row,col)==0 && ~found)
                spot = [row,col];
                found = true;
            end
            col=col+1;
        end
        row=row+1;
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

