function followWallEdge(serPort)
% Drives forward until a wall is found, then circumnavigates the wall 
% counter-clockwise by using the bump sensors.
%
% Input:
% serPort - Serial port for communicating with robot

    disp('Starting followWallEdge');

    % constants
    maxDuration= 120;   % max time to allow the program to run (s)
    maxDistSansBump= 5; % max distance to travel without obstacles (m)
    maxFwdVel= 0.4;     % max allowable forward velocity (m/s)
    defaultArc = -.8;   % default angular arc, represents how closely 
                        % it follows the walls (bigger means tighter)
    minDist = 1;        % distance you must travel before can check if back
                        % in starting position
    distCushion = .2;   % how close you have to be to the starting
    
    % initialize loop variables
    tStart= tic;        % time limit marker
    totalDist = 0;      % total travel distance
    backAtStart = 0;    % true when wall has been circumnavigated
    distSansBump= 0;    % distance traveled without hitting obstacles (m)
    hitFirstWall= 0;    % true after we hit a wall for the first time
    xStart = 0;         % x-coord where we start circumnavigation
    yStart = 0;         % y-coord where we start circumnavigation
    xCur = 0;           % current x-coord
    yCur = 0;           % current y-coord
    ang = 0;            % current orientation of robot (rad)
    
    % reset sensors
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    % start robot moving
    SetFwdVelAngVelCreate(serPort, maxFwdVel, inf);
    
    % loop until we've circumnavigated
    while ~backAtStart
        
        % bail if we've gone too far without a wall
        if (distSansBump > maxDistSansBump)
            disp('Failed to find a wall');
            return;
        end
        
        % bail if we've taken too long
        if (toc(tStart) > maxDuration)
            disp('Took too long to run');
            return;
        end
      
        % update distance from odometry
        recentDist= DistanceSensorRoomba(serPort);
        distSansBump= distSansBump+recentDist;
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
        fprintf('(%d, %d)\tdist %d\n', xCur, yCur, distFromStart);

        % if we've traveled far enough, check if we're back at start
        if totalDist > minDist
            backAtStart = distFromStart < distCushion; 
        end
        
        % check for and react to bump sensor readings
        angToTurn= checkForBump(serPort);
        
        % if obstacle was hit, turn then arc to follow wall
        if angToTurn ~= 0
            
            % stop robot
            SetFwdVelAngVelCreate(serPort, 0, inf);
            
            % turn the robot
            turnRadians(serPort, angToTurn);
            ang = mod(ang + angToTurn, 2*pi);
            
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
            SetFwdVelAngVelCreate(serPort, w2v(defaultArc), defaultArc);
        end
        
        % pause to let the robot run
        pause(0.1);
    end
    
    % stop robot motion
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    disp('Completed followWallEdge');
end

function angToTurn= checkForBump(serPort)
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

function turnRadians(serPort, angToTurn)
% Turn Create with maximum angular velocity and no linear velocity.
%
% Input:
% serPort - Serial port for communicating with robot
% angToTurn - Angle to turn (rad)

    % start turning at max speed
    SetFwdVelAngVelCreate(serPort, 0, v2w(0));

    % reset angle sensor
    AngleSensorRoomba(serPort);
    
    % loop until turn complete
    angTurned= 0;
    while angTurned < angToTurn
        angTurned= angTurned + abs(AngleSensorRoomba(serPort));
        pause(0.1)
    end
    
    % reset angle sesor
    AngleSensorRoomba(serPort);
end

function w= v2w(v)
% Calculate the max allowable angular velocity from the linear velocity.
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % robot constants
    maxWheelVel= 0.5;   % max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % radius of the robot (m)
    
    % max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end

function v= w2v(w)
% Calculate the max allowable linear velocity from the angular velocity.
%
% Input:
% w - Angular velocity of Create (rad/s)
%
% Output:
% v - Linear velocity of Create (m/s)
    
    % robot constants
    maxWheelVel= 0.5;   % max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % radius of the robot (m)
    
    % max velocity combinations obey rule v+wr <= v_max
    v = maxWheelVel - (robotRadius * abs(w)) - .05;
end