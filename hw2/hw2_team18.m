% HW2 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

function hw2_team18(serPort)
% Bug2 algorithm to move from a starting point to a goal point.
% Starts out along the straight line path from start to goal (the M Line). 
% If it hits any obstacles, it will wall follow until M Line is reacuired,
% then head towards the goal point again on the M Line. Stops at goal.
%
% Input:
% serPort - Serial port for communicating with robot

    disp('Starting bug2');

    % reset sensors so start is (0,0,0)
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    % set goal is straight in front, angle irrelevant
    qGoal = [5,0];        
    
    % loop variables
    tStart = tic;       % time limit marker
    pos = [0,0,0];      % x,y,theta - current position and angle
    isAtGoal = 0;       % true when we've reached the goal
    failed = 0;         % true when bug2 failed
    
    % while we aren't at the goal, follow M until wall, circle wall, repeat
    while (~isAtGoal && ~failed)
    
        % go until we hit a wall or the goal
        [pos, isAtGoal, failed] = driveToWallOrGoal(serPort, pos,...
                                                       qGoal, tStart);
        
        % if not done, follow wall until back on M Line or at goal
        if (~isAtGoal && ~failed)
            [pos, isAtGoal, failed] = followWall(serPort, pos,...
                                                    qGoal, tStart);
        end
        
    end
    
    disp('Completed bug2');
    
end

function [qHit, isAtGoal, failed] = driveToWallOrGoal(serPort, pos,...
                                                      qGoal, tStart)
% Drives straight forward until a wall is hit or the goal is reached. 
%
% Input:
% serPort - Serial port for communicating with robot
% pos - Current position of robot (x, y, theta)
% qGoal - Goal position (x, y)
% tStart - Start time of program, used to keep track of duration
%
% Output:
% qHit - Position and angle of hit point when wall encountered
% isAtGoal - True if goal reached
% failed - True if failed to find wall or goal

    disp('Starting driveToWallOrGoal');
    
    % constants
    maxDuration = 600;   % max time to allow the program to run (s)
    maxDistSansBump = 5; % max distance to travel without obstacles (m)
    maxFwdVel = 0.4;      % max allowable forward velocity (m/s)

    % reset sensors for odomentry
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    % loop variables
    isAtGoal = atPoint(pos, qGoal, toc(tStart));
    angToTurn = 0;
    distSansBump = 0;
    qHit = pos; 
    failed = 0;

    % start robot moving
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);

    % loop until we're at the goal or we've bumped
    while (~isAtGoal && angToTurn == 0)

        % pause to let the robot run
        pause(0.05);

        % bail if we've gone too far without a wall
        if (distSansBump > maxDistSansBump)
            disp('Failed to find a wall');
            failed = 1;
            return;
        end

        % bail if we've taken too long
        if (toc(tStart) > maxDuration)
            disp('Took too long to run');
            failed = 1;
            return;
        end

        % update position
        recentDist = DistanceSensorRoomba(serPort);
        distSansBump = distSansBump + recentDist;
        pos(1) = pos(1) + recentDist * cos(pos(3));
        pos(2) = pos(2) + recentDist * sin(pos(3));

        % print position
        fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));

        % check if we've reached the goal
        isAtGoal = atPoint(pos, qGoal, toc(tStart));
        
        % check for wall
        angToTurn = checkForBump(serPort);

    end

    % stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0);

    % if obstacle was hit, perform turn
    if (angToTurn ~= 0)

        % turn the robot
        actualAng = turnRadians(serPort, angToTurn);
        pos(3) = mod(pos(3) + actualAng, 2*pi);
        
        % reset distance because during the bump motion we never travel
        DistanceSensorRoomba(serPort);

        % set hit position
        qHit = pos;
        
    end

    disp('Completed driveToWallOrGoal');
    
end

function [qLeave, isAtGoal, failed] = followWall(serPort, qHit,...
                                                 qGoal, tStart)
% Follows wall until goal is reached, or M Line is reached at a point
% closer to the goal than the start point. 
%
% Input:
% serPort - Serial port for communicating with robot
% qHit - Current position and angle of robot, hit point of wall
% qGoal - Goal position (x, y)
% tStart - Start time of program, used to keep track of duration
%
% Output:
% qLeave - Position and angle of leave point when M Line encountered
% isAtGoal - True if goal reached
% failed - True if failed to find wall or goal

    disp('Starting followWall');

    % constants
    maxDuration = 600;   % max time to allow the program to run (s)
    bumpFwdVel = 0.1;    % velocity after bump (m/s)
    wallFwdVel = 0.3;    % velocity when using wall sensor (m/s)
    wallAngle = -0.1;    % 7 degrees, amount to turn when wall sensing
    minDist = 0.1;       % min dist to travel
    
    % loop variables
    isOnMLine = 0;      % true when back on M Line
    isAtGoal = 0;       % true if we reach the goal
    dist = 0;           % dist traveled (m)
    pos = qHit;
    qLeave = pos;
    failed = 0;
    initDistToGoal = pdist([pos(1),pos(2);qGoal(1),qGoal(2)],'euclidean');

    % reset sensors
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    % start robot moving
    SetFwdVelAngVelCreate(serPort, bumpFwdVel, 0);

    % loop until we've circumnavigated
    while (~isAtGoal && ~isOnMLine)

        % pause to let the robot run
        pause(0.05);
        
        % bail if we've taken too long
        if (toc(tStart) > maxDuration)
            disp('Took too long to run');
            failed = 1;
            return;
        end

        % update distance
        recentDist = DistanceSensorRoomba(serPort);
        dist = dist + recentDist;

        % update angle
        recentAng = AngleSensorRoomba(serPort);
        pos(3) = mod(pos(3) + recentAng, 2*pi);

        % update position
        pos(1) = pos(1) + recentDist * cos(pos(3));
        pos(2) = pos(2) + recentDist * sin(pos(3));

        % print position
        fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));

        % check if we've reached the goal
        isAtGoal = atPoint(pos, qGoal, toc(tStart));
        
        if (~isAtGoal && dist > minDist)

            % check for M Line
            isOnMLine = backOnLine(pos, qHit, qGoal);
            
            % only count it if we're closer to goal
            if (isOnMLine)
                toGoal = pdist([pos(1),pos(2);qGoal(1),qGoal(2)],'euclidean');
                isOnMLine = toGoal < initDistToGoal;
                
                % if we're back at the start, we're trapped
                if (atPoint(pos, qHit, toc(tStart)))
                    disp('circumnavigated wall');
                    failed = 1;
                    return;
                end
            end
            
            % follow wall using IR sensor
            if (WallSensorReadRoomba(serPort) == 0)
                % stop
                SetFwdVelAngVelCreate(serPort, 0, 0);
                % turn a small angle
                actualAng = turnRadians(serPort, wallAngle);
                % handle odometry
                pos(3) = mod(pos(3) + actualAng, 2*pi);
            end

            % drive forward
            SetFwdVelAngVelCreate(serPort, wallFwdVel, 0);
            
        end
        
    end

    % stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0);

    % turn to face goal
    pos(3) = turnToFacePoint(serPort, pos, qGoal);
    
    % set leave point
    qLeave = pos;
    
    disp('Completed followWall');

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
    compensateAng = pi+atan((pos(2)-qGoal(2))/(pos(1)-qGoal(1)))-pos(3);
    
    if (pos(1) > qGoal(1))
        compensateAng = compensateAng - pos(3);
    else
        compensateAng = compensateAng + pos(3);
    end
    
    fprintf('Turning: %.3f\n', compensateAng*(180/pi));
    
    % do the turn
    actualTurn = turnRadians(serPort, compensateAng);
    
    % update the angle
    newAng = mod(pos(3) + actualTurn, 2*pi);

    disp('Completed turnToFacePoint');
    
end

function isOnLine = backOnLine(pos, qStart, qGoal)
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

    distCushion = 0.25 + (duration/60)*0.1;
    dist = pdist([pos(1), pos(2); qGoal(1), qGoal(2)], 'euclidean'); 
    isAtPoint = dist < distCushion; 
    
end

function angToTurn = checkForBump(serPort)
% Check bump sensors and determine how much to turn if needed.
%
% Input:
% serPort - Serial port for communicating with robot
%
% Output:
% angToTurn - angle to turn as a result of bump (rad)

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight, BumpLeft, ~, ~, ~, BumpFront] ...
         = BumpsWheelDropsSensorsRoomba(serPort);

    % Turn counter-clockwise if bumped
    if BumpRight
        angToTurn = pi/4;
    elseif BumpLeft
        angToTurn = pi/2 + pi/4;
    elseif BumpFront
        angToTurn = pi/2;
    else
        angToTurn = 0;
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
        disp(angTurned);
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
