function followBoxEdge(serPort)
% move and hit a wall, and then follow the wall the entire distance of the wall, 
% stopping when it returns to the start point. 

    % Drive until we hit a wall
    [BumpRight, BumpLeft, BumpFront]= driveUntilWallHit(serPort);
    
    % If we couldn't find one, bail
    if (~BumpRight && ~BumpLeft && ~BumpFront)
        return
    end
    
    if BumpRight || BumpFront
        SetFwdVelAngVelCreate(serPort,0,1)  % Turn counter-clockwise
    elseif BumpLeft
        SetFwdVelAngVelCreate(serPort,0,-1) % Turn clockwise
    end
    
    angTurned= 0;
    while angTurned < pi/4 && ~BumpFront
        angTurned= angTurned+abs(AngleSensorRoomba(serPort));
        pause(0.1)
    end
    
    if (1 == 2)

    % Set constants for this program
    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxDistSansBump= 5; % Max distance to travel without obstacles (m)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
    maxOdomAng= pi/4;   % Max angle to move around a circle before 
                        % increasing the turning radius (rad)
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    angTurned= 0;       % Angle turned since turning radius increase (rad)
    v= 0;               % Forward velocity (m/s)
    w= v2w(v);          % Angular velocity (rad/s)
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)
    
    % Enter main loop
    while toc(tStart) < maxDuration && distSansBump <= maxDistSansBump
        % Check for and react to bump sensor readings
        bumped= bumpCheckReact(serPort);
        
        % If obstacle was hit reset distance and angle recorders
        if bumped
            DistanceSensorRoomba(serPort);  % Reset odometry too
            AngleSensorRoomba(serPort);
            distSansBump= 0;
            angTurned= 0;
            
            % Start moving again at previous velocities
            SetFwdVelAngVelCreate(serPort,v,w)
        end
        
        % Update distance and angle recorders from odometry
        distSansBump= distSansBump+DistanceSensorRoomba(serPort);
        angTurned= angTurned+AngleSensorRoomba(serPort);
        
        % Increase turning radius if it is time
        if angTurned >= maxOdomAng
            % Either increase forward velocity by the increment or by half
            % the difference to the max velocity, whichever is lower
            v= min(v+maxVelIncr,v+(maxFwdVel-v)/2);
            % Choose angular velocity based on max allowable wheel speed
            w= v2w(v);
            SetFwdVelAngVelCreate(serPort,v,w)
            % This could be accomplished more simply by using
            % SetFwdVelRadiusRoomba, this way is just done more fun
        end
        
        % Briefly pause to avoid continuous loop iteration
        pause(0.1)
    end
    
    % Stop robot motion
    SetFwdVelAngVelCreate(serPort,0,0)
    
    end
end


function [BumpRight,BumpLeft,BumpFront]= driveUntilWallHit(serPort)

    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxDist= 5; % Max distance to travel without obstacles (m)
    maxFwdVel= 1.0;     % Max allowable forward velocity (m/s)
    maxVelIncr= 0.1;   % Max incrementation of forward velocity (m/s)
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    dist= 0;            % Distance traveled (m)
    v= 0.1;             % Forward velocity (m/s)
    w= 0;               % Angular velocity (rad/s)
    bumped= 0;          % True when a bump occurs
    
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)

    % Loop until wall is hit or we've gone too far/long
    while toc(tStart) < maxDuration && ...
          dist <= maxDist && ...
          ~bumped
        
        % Check for bump sensor readings
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront] ...
            = BumpsWheelDropsSensorsRoomba(serPort);    
        bumped= BumpRight || BumpLeft || BumpFront;
        
        % Update distance
        dist= dist+DistanceSensorRoomba(serPort);
        
        % Cleanly speed up to maxFwdVel
        v= min(v+maxVelIncr,v+(maxFwdVel-v)/2);
        
        % Briefly pause to avoid continuous loop iteration
        pause(0.1)
    end
    
    % Reset odometry
    DistanceSensorRoomba(serPort); 
    AngleSensorRoomba(serPort);
    
    % Stop moving
    SetFwdVelAngVelCreate(serPort,0,0)
end


function bumped= bumpCheckReact(serPort)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster, ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    
    bumped= BumpRight || BumpLeft || BumpFront;
    
    % Halt forward motion and turn only if bumped
    if bumped
        AngleSensorRoomba(serPort);     % Reset angular odometry
        v= 0;       % Forward velocity
        w= v2w(v);  % Angular velocity
        
        % Turn away from obstacle
        if BumpRight
            SetFwdVelAngVelCreate(serPort,v,w)  % Turn counter-clockwise
            ang= pi/4;  % Angle to turn
        elseif BumpLeft
            SetFwdVelAngVelCreate(serPort,v,-w) % Turn clockwise
            ang= pi/4;
        elseif BumpFront
            SetFwdVelAngVelCreate(serPort,v,w)  % Turn counter-clockwise
            ang= pi/2;                          % Turn further
        end
        
        % Wait for turn to complete
        angTurned= 0;
        while angTurned < ang
            angTurned= angTurned+abs(AngleSensorRoomba(serPort));
            pause(0.1)
        end
        % This could be accomplished more simply by using turnAngle, 
        % this way is just more fun
    end
end


function w= v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end