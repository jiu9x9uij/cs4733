% HW1 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

function hw1_team18(serPort)
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
    defaultArc = .1;   % default angular arc, represents how closely 
                        % it follows the walls (bigger means tighter)
    minDist = 1;        % distance you must travel before can check if back
                        % in starting position
    distCushion = .2;   % how close you have to be to the starting
    slowTurnSpeed = .35; % rads per second
    
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
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);
    
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
      
        wall_sensor = WallSensorReadRoomba(serPort);
        if wall_sensor == 1
            disp('********** WALL SENSOR IS 1 **********\n');
        end
        
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
        fprintf('pos: (%.3f, %.3f), ang: %.3f \n', xCur, yCur, ang * (180/pi));
        

        % if we've traveled far enough, check if we're back at start
        if totalDist > minDist
            backAtStart = distFromStart < distCushion; 
        end
        
        % check for and react to bump sensor readings
        angToTurn= checkForBump(serPort);
        
        % if obstacle was hit, turn then arc to follow wall
        if angToTurn ~= 0
            
            % stop robot, i changed it to 0,0 because it threw errors
            % before when it said inf
            SetFwdVelAngVelCreate(serPort, 0, 0);
            
            % turn the robot
            actualAng = turnRadians(serPort, angToTurn, slowTurnSpeed);
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
            SetFwdVelAngVelCreate(serPort, w2v(defaultArc), defaultArc);
        end
        
        % pause to let the robot run
        pause(0.01);
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
        %angToTurn = pi;
    elseif BumpLeft
        angToTurn = pi/2 + pi/8;
        %angToTurn = pi;
    elseif BumpFront
        angToTurn = pi/4;
        %angToTurn = pi;
    else
        angToTurn = 0;
    end
    
end

function angTurned = turnRadians(serPort, angToTurn, slowTurnSpeed)
% Turn Create with maximum angular velocity and no linear velocity.
%
% Input:
% serPort - Serial port for communicating with robot
% angToTurn - Angle to turn (rad)

    % reset angle sensor
    AngleSensorRoomba(serPort);
    
    % loop until turn complete
    angTurned= 0;
    while angTurned < angToTurn
        pause(0.01)
        % start turning at speed given in constants section
        SetFwdVelAngVelCreate(serPort, 0, slowTurnSpeed);
        angTurned= angTurned + abs(AngleSensorRoomba(serPort));
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    pause(0.2);
    % reset angle sesor
    angTurned = angTurned + abs(AngleSensorRoomba(serPort));
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