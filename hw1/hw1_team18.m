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
    maxDuration= 300;   % max time to allow the program to run (s)
    maxDistSansBump= 5; % max distance to travel without obstacles (m)
    maxFwdVel= 0.4;     % max allowable forward velocity (m/s)
    wallFwdVel = 0.1;   % follow that wall tight man
    bumpFwdVel = 0.3;
    wallAngle = -0.1;   % approx 7 degrees, trying to hug it tight
                        % it follows the walls (bigger means tighter)
    minDist = 1;        % distance you must travel before can check if back
                        % in starting position
    postBumpDist = .1;  % after you bump, travel for a sec
    slowTurnSpeed = .35; % rads per second

    % initialize loop variables
    tStart= tic;        % time limit marker
    totalDist = 0;      % total travel distance
    backAtStart = 0;    % true when wall has been circumnavigated
    distSansBump= 0;    % distance traveled without hitting obstacles (m)
    distSansWall= 0;
    hitFirstWall= 0;    % true after we hit a wall for the first time
    xStart = 0;         % x-coord where we start circumnavigation
    yStart = 0;         % y-coord where we start circumnavigation
    xCur = 0;           % current x-coord
    yCur = 0;           % current y-coord
    ang = 0;            % current orientation of robot (rad)

    if isa(serPort,'CreateRobot')
        followWallSimulator();
    else
        followWallRealRobot();
    end
        
    function followWallRealRobot
        disp('REAL ROBOT OH SNAP');
        slowTurnSpeed= 0.05;
        maxFwdVel = 0.1;
        distMultiplier= 0.8;
        angMultiplier= 0.81;
        distIncr = 0.05;
        angToTurn = 45;
        
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

            % Check bump sensors (ignore wheel drop sensors)
            [BumpRight, BumpLeft, ~, ~, ~, BumpFront] ...
                 = BumpsWheelDropsSensorsRoomba(serPort);

            % Turn counter-clockwise if bumped
            numTurns = 0;
            if BumpRight == 1
                numTurns = 1;
            elseif BumpLeft == 1
                numTurns = 3;
            elseif BumpFront == 1
                numTurns = 2;
            end
            
            % if obstacle was hit, turn then arc to follow wall
            if numTurns > 0

                % back up a little
                travelDist(serPort, maxFwdVel, -distMultiplier*distIncr/2);
                fprintf('travel %.3f\n', -distIncr/2);
                
                % update odometry double because it thinks it's farther
                % than it is
                xCur = xCur - distIncr * cosd(ang);
                yCur = yCur - distIncr * sind(ang);
                
                % mark start position if this is first bump
                if (~hitFirstWall)
                    hitFirstWall= true;
                    xStart = xCur;
                    yStart = yCur;
                end
                
                % execute the turn
                while (numTurns > 0)
                    turnAngle(serPort, slowTurnSpeed, angMultiplier*angToTurn);
                    fprintf('turn %.3f\n', angToTurn);
                    numTurns = numTurns - 1;
                end
                
                % update odometry
                ang = mod(ang + angToTurn*numTurns, 360);

                % reset distance
                distSansBump = 0;

            else
                % go a step forward
                travelDist(serPort, maxFwdVel, distMultiplier*distIncr);
                fprintf('travel %.3f\n', distIncr);
                distSansBump= distSansBump+distIncr;
                
                % update odometry
                xCur = xCur + distIncr * cosd(ang);
                yCur = yCur + distIncr * sind(ang);
                
                % turn a little if we're in wall following
                if (hitFirstWall)
                    totalDist = totalDist+distIncr;
                    
                    turnAngle(serPort, slowTurnSpeed, -angMultiplier*angToTurn);
                    fprintf('turn %.3f\n', -angToTurn);
                    ang = mod(ang - angToTurn, 360);
                end
                
            end
            
            % check position and see how far we are from start
            distFromStart= pdist([xStart, yStart; xCur, yCur], 'euclidean');
            fprintf('pos: (%.3f, %.3f), ang: %.3f\n', xCur, yCur, ang);

            % if we've traveled far enough, check if we're back at start
            if totalDist > minDist
                backAtStart = distFromStart < distCushion(toc(tStart)); 
            end

        end
        
        %GO TO (0,0)!!!
        %turn to face 0,0, go correct distance, stop

        %helper theta angle
        littleGuy = atand(yCur/xCur);
        %calculate what we need to turn
        compensateAng = 180 + littleGuy - ang;
        %do the turning
        turnAngle(serPort, slowTurnSpeed, angMultiplier*compensateAng);
        fprintf('turn %.3f\n', compensateAng);
        ang = mod(ang + compensateAng, 360);
        % how far are we from legit origin
        distFromStart= pdist([0, 0; xCur, yCur], 'euclidean');
        travelDist(serPort, maxFwdVel, distMultiplier*distFromStart);
        fprintf('travel %.3f\n', distFromStart);
        turnAngle(serPort, slowTurnSpeed, -angMultiplier*ang);
        fprintf('turn %.3f\n', -ang);

        disp('Completed followWallEdge');
        
    end

    function followWallSimulator

        % reset sensors so start is (0,0)
        DistanceSensorRoomba(serPort);
        AngleSensorRoomba(serPort);

        % start robot moving
        SetFwdVelAngVelCreate(serPort, maxFwdVel, 0);

        % loop until we've circumnavigated
        while ~backAtStart
            % pause to let the robot run
            pause(0.05);

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
                % this is our moment to set default pattern
                SetFwdVelAngVelCreate(serPort, wallFwdVel, 0);

            elseif hitFirstWall && distSansBump > postBumpDist
               % we did not bump a wall, so listen to the wall sensor 
                wall_sensor = WallSensorReadRoomba(serPort);
                if wall_sensor == 1
                    disp('********** We is good **********');
                    distSansWall = 0;
                    SetFwdVelAngVelCreate(serPort, bumpFwdVel, 0);
                else
                    % stop
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    % turn a small angle
                    actualAng = turnRadians(serPort, wallAngle, -slowTurnSpeed);
                    % handle odometry
                    ang = mod(ang + actualAng, 2*pi);
                    % start driving again
                    SetFwdVelAngVelCreate(serPort, bumpFwdVel, 0);
                end

            end


        end

        % stop robot motion
        SetFwdVelAngVelCreate(serPort, 0, 0);

        %GO TO (0,0)!!!
        %turn to face 0,0, go correct distance, stop

        %helper theta angle
        littleGuy = atan(yCur/xCur);
        %calculate what we need to turn
        compensateAng = pi + littleGuy - ang;
        %do the turning
        turnRadians(serPort, compensateAng, slowTurnSpeed);
        ang = mod(ang + compensateAng, 2*pi);
        % how far are we from legit origin
        distFromStart= pdist([0, 0; xCur, yCur], 'euclidean');
        travelDist(serPort, maxFwdVel, distFromStart);
        
        turnRadians(serPort, ang, -slowTurnSpeed);

        disp('Completed followWallEdge');
    end
    
end

function dist = distCushion(duration)
    dist= 0.2 + (duration/60)*0.1;
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
    angTurned = AngleSensorRoomba(serPort);
    SetFwdVelAngVelCreate(serPort, 0, slowTurnSpeed);
    % loop until turn complete
    while abs(angTurned) < abs(angToTurn)
        pause(0.01)
        % start turning at speed given in constants section
        angTurned= angTurned + AngleSensorRoomba(serPort);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    pause(0.2);
    % reset angle sesor
    angTurned = angTurned + AngleSensorRoomba(serPort);
end
