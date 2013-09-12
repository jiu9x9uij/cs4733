function followBox(serPort)
% Drives forward until a wall is found, then circumnavigates the wall 
% counter-clockwise by using the bump sensors.
%
% Input:
% serPort - Serial port for communicating with robot

    disp('Starting followWallEdge');

    % constants
    maxDuration= 1200;  % max time to allow the program to run (s)
    maxDistSansBump= 5; % max distance to travel without obstacles (m)
    maxFwdVel= 0.5;     % max allowable forward velocity (m/s)
    defaultArc = -.8;   % default angular arc, represents how closely 
                        % it follows the walls (bigger means tighter)
    minDist = 1;        % distance you must travel before can check if back
                        % in starting position
    distCushion = .2;   % how close you have to be to the starting
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    totalDist = 0;      % total travel distance
    backAtStart = 0;    % true when obstacle has been circumnavigated
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    xStart = 0;
    yStart = 0;
    first= true;
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort, maxFwdVel, 0)
    
    % Enter main loop
    while toc(tStart) < maxDuration && ...
          distSansBump <= maxDistSansBump && ...
          ~backAtStart
        
        % Update distance and angle recorders from odometry
        recentDist = DistanceSensorRoomba(serPort);
        distSansBump= distSansBump+recentDist;
        if(~first)
            totalDist = totalDist+recentDist;
        end
        
        % Check position and see how far we are from start
        [xCur, yCur, ~]= OverheadLocalizationCreate(serPort);
        distFromStart= pdist([xStart, yStart; xCur, yCur], 'euclidean');
        fprintf('x %d, y %d, dist %d\n', xCur, yCur, distFromStart);

        %we check if we're back at the start, only if we've traveled a bit
        if totalDist > minDist
            % Consider back at start if kind of close
            backAtStart = distFromStart < distCushion; 
        end        
        
        % Check for and react to bump sensor readings
        bumped= bumpCheckReact(serPort);
        
        % If obstacle was hit set default arc
        if bumped
            if(first)
                first = false;
                [xStart, yStart, ~]= OverheadLocalizationCreate(serPort);
            end
            distSansBump = 0;
            % reset distance because during the bump motion we never
            % traveled any actual distance
            DistanceSensorRoomba(serPort);
            SetFwdVelAngVelCreate(serPort, w2v(defaultArc), defaultArc);
        end
        
        % Briefly pause to avoid continuous loop iteration
        pause(0.1)
    end
    
    % Stop robot motion
    SetFwdVelAngVelCreate(serPort,0,0);
    
    disp('Completed followWallEdge');

end

function bumped= bumpCheckReact(serPort)
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
    bumped= BumpRight || BumpLeft || BumpFront;
    
    % Halt forward motion and turn only if bumped
    if bumped
       
        % Turn counter-clockwise
        if BumpRight
            angToTurn= pi/8;
        elseif BumpLeft
            angToTurn= pi/2 + pi/8;
        elseif BumpFront
            angToTurn= pi/4;
        end
        
        turnRadians(serPort, angToTurn);
        
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
    AngleSensorRoomba(serPort)
    
    % loop until turn complete
    angTurned= 0;
    while angTurned < angToTurn
        angTurned= angTurned + abs(AngleSensorRoomba(serPort));
        pause(0.1)
    end
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