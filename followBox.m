function followBox(serPort)


    % consts
    maxDuration= 1200;  % Max time to allow the program to run (s)
    maxDistSansBump= 5; % Max distance to travel without obstacles (m)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    defaultArc = -.8;   % default angular arc, this can be changed
                        % to represent how closely it follows the walls
                        % aka bigger means tighter follow
    global currentAngle;
    currentAngle = 0;   % start out with no angle
    recentDist = 0;     % distance travelled since last polled
    coord = [0 0];      % current position
    minDist = 1;       % distance you must travel before can check if back
                        % in starting position
    distCushion = .2;   % how close you have to be to the starting
    totalDist = 0;      % total travel distance
    done = 0;
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    v= maxFwdVel;       % start out going straight and fast cuz why not
    w= 0;               % Angular velocity (rad/s)
    xStart = 0;
    yStart = 0;
    first= true;
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,v,w)
    % Enter main loop
    while toc(tStart) < maxDuration && distSansBump <= maxDistSansBump ...
            && ~done
        
        % Update distance and angle recorders from odometry
        recentDist = DistanceSensorRoomba(serPort);
        distSansBump= distSansBump+recentDist;
        if(~first)
            totalDist = totalDist+recentDist;
        end
        currentAngle= currentAngle+AngleSensorRoomba(serPort);
        
        %maybe we don't need this shiz
        %coord = getNewPosition(coord, recentDist, currentAngle);
        
        % Check position and see how far we are from start
        [xCur, yCur, ~]= OverheadLocalizationCreate(serPort);
        distFromStart= pdist([xStart, yStart; xCur, yCur], 'euclidean');
        fprintf('x %d, y %d, dist %d\n', xCur, yCur, distFromStart);

        %we check if we're back at the start, only if we've traveled a bit
        if totalDist > minDist
            % Consider back at start if kind of close
            done = distFromStart < distCushion; 
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
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w);
    
    % make a beep for fun
    BeepRoomba(serPort);
    
end

function coord = getNewPosition(coord, recentDist, currentAngle)
    % get the deltas
    deltaX = cos(currentAngle) * recentDist;
    deltaY = sin(currentAngle) * recentDist;
    % update the new coordinate
    coord = [coord(1) + deltaX, coord(2) + deltaY];
    % disp(coord);
end

function bumped= bumpCheckReact(serPort)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated
global currentAngle;
    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight, BumpLeft, ~, ~, ~, BumpFront] ...
         = BumpsWheelDropsSensorsRoomba(serPort);
    bumped= BumpRight || BumpLeft || BumpFront;
    
    % Halt forward motion and turn only if bumped
    if bumped
        v= 0;       % Forward velocity
        w= v2w(v);  % Angular velocity
        
        % Turn away from obstacle
        if BumpRight
            SetFwdVelAngVelCreate(serPort,v,w)  % Turn counter-clockwise
            ang= pi/8;  % Angle to turn
        elseif BumpLeft
            SetFwdVelAngVelCreate(serPort,v,w) % Turn more counter
            ang= pi/2 + pi/8;
        elseif BumpFront
            SetFwdVelAngVelCreate(serPort,v,w)  % Turn counter-clockwise
            ang= pi/4;                          % Turn further
        end
        
        % Wait for turn to complete
        angTurned= 0;
        while angTurned < ang
            angTurned= angTurned+abs(AngleSensorRoomba(serPort));
            pause(0.1)
        end
        
        AngleSensorRoomba(serPort); %reset the angle sensor
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

function v= w2v(w)
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
    v = maxWheelVel - (robotRadius * abs(w)) - .05;
end