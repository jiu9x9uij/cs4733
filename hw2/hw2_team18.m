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

    isSimulator = isa(serPort,'CreateRobot');

    qStart = [0,0,0]; % x,y,theta - start at origin with no angle
    qGoal = [5,0];    % goal is straight in front, angle irrelevant
    
    % reset sensors so start is (0,0,0)
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    tStart = tic;       % time limit marker
    curPos = qStart;    % x,y,theta of current position
    isAtGoal = 0;       % true when we've reached the goal
    failed = 0;         % true when bug2 failed
    
    % while we aren't at the goal, follow M until wall, circle wall, repeat
    while (~isAtGoal && ~failed)
    
        % go until we hit a wall or the goal
        [curPos, isAtGoal, failed] = driveToWallOrGoal(serPort, curPos,...
                                                       qGoal, tStart);
        
        % if not done, follow wall until back on M Line or at goal
        if (~isAtGoal && ~failed)
            [curPos, isAtGoal, failed] = followWall(serPort, curPos,...
                                                    qGoal, tStart);
        end
        
    end
    
    disp('Completed bug2');
    
end

function [qHit, isAtGoal, failed] = driveToWallOrGoal(serPort, pos,...
                                                      qGoal, tStart)
    disp('Starting driveToWallOrGoal');
    
    % constants
    maxDuration = 600;
    maxDistSansBump = 5;
    
    % reset sensors for odomentry
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    % start robot moving
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);

    % check for goal
    isAtGoal = atGoal(pos, qGoal, toc(tStart));
    angToTurn = 0;
    
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
        fprintf('(%.3f, %.3f, %.3f)\n',pos(1),pos(2),pos(3)*(180/pi));

        % check if we've reached the goal
        isAtGoal = atGoal(curPos, goalPos, toc(tStart));
        
        % check for wall
        angToTurn = checkForBump(serPort);

    end

    % stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0);

    % if obstacle was hit, perform turn
    if angToTurn ~= 0

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

function [qLeave, isAtGoal, failed] = followWall(serPort, pos,...
                                                 qGoal, tStart)
% Circumnavigates wall counter-clockwise from qStart to qGoal using 
% bump and wall sensors. 
%
% Input:
% serPort - Serial port for communicating with robot
% qHit - hit point where wall first encountered and robot currently is
% qGoal - goal point, makes M Line with qHit
%
% Output:
% qLeave - leave point and angle where robot is back on M Line

    disp('Starting followWall');

    % gather inputs
    xCur = pos(1);
    yCur = pos(2);
    ang = pos(3);
    xGoal = qGoal(1);
    yGoal = qGoal(2);
    
    % print out inputs
    fprintf('qHit: (%.3f, %.3f), ang: %.3f\n', xCur, yCur, ang * (180/pi));

    % constants
    maxDuration= 600;   % max time to allow the program to run (s)
    maxDistSansBump= 5; % max distance to travel without obstacles (m)
    minDist = 1;        % distance you must travel before can check if back
                        % in starting position

    % initialize loop variables
    tStart= tic;        % time limit marker
    totalDist = 0;      % total travel distance
    backAtStart = 0;    % true when wall has been circumnavigated
    backOnMLine = 0;    % true when back on M Line
    closerToGoal = 0;   % true when closer to goal than start position
    distSansBump= 0;    % distance traveled without hitting obstacles (m)
    hitFirstWall= 0;    % true after we hit a wall for the first time
    xStart = xCur;      % x-coord where we start circumnavigation
    yStart = yCur;      % y-coord where we start circumnavigation
    maxFwdVel = 0.4;      % max allowable forward velocity (m/s)
    bumpFwdVel = 0.1;     % velocity after bump (m/s)
    wallFwdVel = 0.3;     % velocity when using wall sensor (m/s)
    wallAngle = -0.1;     % 7 degrees, amount to turn when wall sensing
    postBumpDist = 0.1;   % after you bump, travel for a sec
    distSansWall= 0;      % dist traveled since last wall sensor

    % reset sensors so start is (0,0)
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    % start robot moving
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);

    % loop until we've circumnavigated
    while ~backAtStart

        % bail if we've gone too far without a wall
        if (distSansBump > maxDistSansBump && distSansWall > maxDistSansBump)
            disp('Failed to find a wall');
            return;
        end

        % bail if we've taken too long
        if (toc(tStart) > maxDuration)
            disp('Took too long to run');
            return;
        end

        recentDist= DistanceSensorRoomba(serPort);
        distSansBump= distSansBump+recentDist;
        distSansWall= distSansWall+recentDist;
        if (hitFirstWall)
            totalDist = totalDist+recentDist;
        end

        % update angle
        recentAng = AngleSensorRoomba(serPort);
        ang = mod(ang + recentAng, 2*pi);

        % check position and see how far we are from start
        xCur = xCur + recentDist * cos(ang);
        yCur = yCur + recentDist * sin(ang);
        distFromStart= pdist([xStart, yStart; xCur, yCur], 'euclidean');
        fprintf('pos: (%.3f, %.3f), ang: %.3f\n', xCur, yCur, ang * (180/pi));

        % if we've traveled far enough, check if we're back at start
        if totalDist > minDist
            backAtStart = distFromStart < distCushion(toc(tStart)); 
        end

        % check for and react to bump sensor readings
        angToTurn= checkForBump(serPort);

        % if obstacle was hit, turn then arc to follow wall
        if angToTurn ~= 0

            % stop robot, i changed it to 0,0 because it threw errors
            % before when it said inf
            SetFwdVelAngVelCreate(serPort, 0, 0);

            % turn the robot
            actualAng = turnRadians(serPort, angToTurn);
            ang = mod(ang + actualAng, 2*pi);

            % mark start position if this is first bump
            if (~hitFirstWall)
                hitFirstWall= true;
                xStart = xCur;
                yStart = yCur;
            end

            % reset distance because during the bump motion we never travel
            DistanceSensorRoomba(serPort);
            distSansBump = 0;

            % follow the wall edge by arcing
            % this is our moment to set default pattern
            SetFwdVelAngVelCreate(serPort, bumpFwdVel, 0);

        elseif hitFirstWall && distSansBump > postBumpDist
           % we did not bump a wall, so listen to the wall sensor 
            wall_sensor = WallSensorReadRoomba(serPort);
            if wall_sensor == 1
                distSansWall = 0;
                SetFwdVelAngVelCreate(serPort, wallFwdVel, 0);
            else
                % stop
                SetFwdVelAngVelCreate(serPort, 0, 0);
                % turn a small angle
                actualAng = turnRadians(serPort, -wallAngle);
                % handle odometry
                ang = mod(ang + actualAng, 2*pi);
                % start driving again
                SetFwdVelAngVelCreate(serPort, wallFwdVel, 0);
            end

        end

        % pause to let the robot run
        pause(0.05);

    end

    % stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0);

    disp('Completed followWall');

end

function newAng = turnToFacePoint(xCur, yCur, ang, xGoal, yGoal)

    % calculate what we need to turn
    compensateAng = pi + atan((yCur-yGoal)/(xCur-xGoal)) - ang;

    % do the turn
    turnRadians(serPort, compensateAng);
    
    % update the angle
    newAng = mod(ang + compensateAng, 2*pi);

end

function isOnLine = backOnLine(xCur, yCur, xStart, yStart, xGoal, yGoal)

    allowableSlopeDiff = 0.05;

    startToCurSlope = (yCur - yStart)/(xCur - xStart);
    curToGoalSlope = (yGoal - yCur)/(xGoal - xCur);
    
    isOnLine = abs(startToCurSlope - curToGoalSlope) < allowableSlopeDiff;

end

function isAtGoal = atGoal(curPos, goalPos, duration)
% Calculates how far from start you can be for us to say you're
% "back at start". Depends on how long program has been running.
%
% Input:
% duration - How long program has been running (s)
%
% Output:
% isAtGoal - How far from start you can be to be considered "back"

    xCur = curPos(1);
    yCur = curPos(2);
    xGoal = goalPos(1);
    yGoal = goalPos(2);
    distCushion = 0.25 + (duration/60)*0.1;
    dist= pdist([xCur, yCur; xGoal, yGoal], 'euclidean'); 
    isAtGoal = dist < distCushion; 
    
end

function angToTurn = checkForBump(serPort)
% Check bump sensors and steer the robot away from obstacles if necessary.
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated

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
% Turn Create with maximum angular velocity and no linear velocity.
%
% Input:
% serPort - Serial port for communicating with robot
% angToTurn - Angle to turn (rad)
%
% Output:
% angTurned - Actual angle turned (rad)

    turnSpeed = 0.35; % turn angle speed (rad/s)

    if (angToTurn < 0)
        turnSpeed = -turnSpeed;
        angToTurn = -angToTurn;
    end
    
    % reset angle sensor
    angTurned = AngleSensorRoomba(serPort);
    
    % start turning
    SetFwdVelAngVelCreate(serPort, 0, turnSpeed);
    
    % loop until turn complete
    while (abs(angTurned) < abs(angToTurn))
        pause(0.01);
        angTurned = angTurned + AngleSensorRoomba(serPort);
    end
    
    % stop turning
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    % pause in case robot still turning a little
    pause(0.2);
    
    % reset angle sensor
    angTurned = angTurned + AngleSensorRoomba(serPort);
    
end
