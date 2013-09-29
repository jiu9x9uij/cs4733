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
    
        % turn to face the goal point
        pos(3) = turnToFacePoint(serPort, pos, qGoal);
        
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
    maxFwdVel = 0.4;     % max allowable forward velocity (m/s)

    % reset sensors for odomentry
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    % loop variables
    isAtGoal = atPoint(pos, qGoal, toc(tStart));
    bumpedWall = 0;
    qHit = pos;

    % start robot moving
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);

    % loop until we're at the goal or we've bumped
    while (~isAtGoal && ~bumpedWall)

        % pause to let the robot run
        pause(0.05);

        % bail if we've taken too long
        if (toc(tStart) > maxDuration)
            disp('Took too long to run');
            failed = 1;
            return;
        end

        % update position
        recentDist = DistanceSensorRoomba(serPort);
        qHit(1) = qHit(1) + recentDist * cos(qHit(3));
        qHit(2) = qHit(2) + recentDist * sin(qHit(3));
        printPosition(qHit);

        % check if we've reached the goal
        isAtGoal = atPoint(qHit, qGoal, toc(tStart));
        
        % check for wall
        bumpedWall = checkForBump(serPort) ~= 0;

    end

    % stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0);

    % no need to update position again here, tested it and doesn't change
    
    % succeeded!
    failed = 0; 
    
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
    postBumpDist = 0.1;  % min dist to travel after bump (m)
    distCorrect = 0.02;  % correction when inching forward (m)
    
    % loop variables
    isCloserOnMLine = 0; % true when back on M Line
    isAtGoal = 0;        % true if we reach the goal
    distSansBump = 0;    % dist traveled (m)
    qLeave = qHit;       % leave point when we finish (x, y, theta)
    
    % figure out how far we are from goal now
    initDistToGoal = pdist([qLeave(1),qLeave(2);qGoal(1),qGoal(2)],...
                           'euclidean');

    % loop until we're at the goal or back on M Line
    while (~isAtGoal && ~isCloserOnMLine)

        % pause to let the robot run
        pause(0.05);
        
        % bail if we've taken too long
        if (toc(tStart) > maxDuration)
            disp('Took too long to run');
            failed = 1;
            return;
        end

        % update distance and position
        recentDist = DistanceSensorRoomba(serPort);
        distSansBump = distSansBump + recentDist;
        qLeave(1) = qLeave(1) + recentDist * cos(qLeave(3));
        qLeave(2) = qLeave(2) + recentDist * sin(qLeave(3));
        printPosition(qLeave);
        
        % update angle
        recentAng = AngleSensorRoomba(serPort);
        qLeave(3) = mod(qLeave(3) + recentAng, 2*pi);

        % check for bump
        angToTurn = checkForBump(serPort);

        % if obstacle was hit, turn and go slow
        if angToTurn ~= 0

            % turn the robot
            actualAng = turnRadians(serPort, angToTurn);
            qLeave(3) = mod(qLeave(3) + actualAng, 2*pi);

            % reset distance because during the bump motion we never travel
            % DistanceSensorRoomba(serPort);
            distSansBump = 0;

            % drive forward slowly
            SetFwdVelAngVelCreate(serPort, bumpFwdVel, 0);

        % otherwise if we've gone far enough
        elseif distSansBump > postBumpDist

            % follow wall using IR sensor
            if (WallSensorReadRoomba(serPort) == 0)
                % turn a small angle
                actualAng = turnRadians(serPort, wallAngle);
                qLeave(3) = mod(qLeave(3) + actualAng, 2*pi);
                
                % bug in simulator doesn't update position, manually add
                distSansBump = distSansBump + distCorrect;
                qLeave(1) = qLeave(1) + distCorrect * cos(qLeave(3));
                qLeave(2) = qLeave(2) + distCorrect * sin(qLeave(3));
            end

            % drive forward
            SetFwdVelAngVelCreate(serPort, wallFwdVel, 0);
            
            % check if we're on the M Line
            if (onLine(qLeave, qHit, qGoal))

                disp('On M Line');

                % check if we're closer to the goal
                toGoal = pdist([qLeave(1),qLeave(2);qGoal(1),qGoal(2)],...
                               'euclidean');
                isCloserOnMLine = toGoal < initDistToGoal;

                % if we're back at the start, we're trapped
                if (atPoint(qLeave, qHit, toc(tStart)))
                    disp('circumnavigated wall, trapped');
                    failed = 1;
                    return;
                end

            end
                    
        end
        
        % check if we've reached the goal
        isAtGoal = atPoint(qLeave, qGoal, toc(tStart));
        
    end

    % stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0);

    % succeeded!
    failed = 0;

    disp('Completed followWall');

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
        angToTurn = pi/8;
    elseif BumpLeft
        angToTurn = pi/2 + pi/8;
    elseif BumpFront
        angToTurn = pi/4;
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

function printPosition(pos)
% Displays x,y,theta position in readible format.
%
% Input:
% pos - Position to display

    fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));

end
