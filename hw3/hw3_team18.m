% HW2 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

function hw3_team18(serPort)
    %% DESCRIPTION %%%%%%%%%%%%%%%%%%%
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% VARIABLES %%%%%%%%%%%%%%%%%%%%%
    % 1 will mark empty, 2 will mark filled
    % 0 means that space was never traversed.
    gridSize = 11; %must be odd, robot starts in center
    robotDiameter = .4; %height and width of each grid square
    robotRadius = robotDiameter/2; % used for marking walls and stuff
    status = 1; %



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% SET UP CODE %%%%%%%%%%%%%%%%%%%
    grid = createGrid(gridSize);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%
    switch status
        case 1 %spiral until you find something
        case 2 % wall follow for some random amount of time
        case 3 % drive till you hit something
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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


function grid = markEmpty(grid, pos, robotDiameter)
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
    [row, col] = translateCoordGridSpace(grid, pos, robotDiameter);
    if(grid(row,col)== 0)
       grid(row,col) = 1; 
       fprintf('empty!  - > row: %f col: %f\n',row,col);
    end
end

function grid = markFilled(grid, pos, robotDiameter)
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
    [row, col] = translateCoordGridSpace(grid, pos, robotDiameter);
    grid(row,col) = 2;
    fprintf('filled! - > row: %f col: %f\n',row,col);
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
    offset = s(1)/2 + 1;
    disp(pos(1)/robotDiameter + offset);
    col = floor(pos(1)/robotDiameter + offset); % x refers to column
    row = floor(pos(2)/robotDiameter + offset); % y refers to row
    if(col > s || row > s || col < 0 || row < 0)
        disp('tried to access unknown part of grid!');
        row = offset;
        col = offset; % just mark the center point so it doesn't mess up
    end
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
