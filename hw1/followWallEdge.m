function followWallEdge(serPort)

    % Constant definitions
    maxDuration= 1200; % max execution time (s)
    turnV= 0.2;        % velocity while turning (m/s)
    v= 0.3;            % linear robot velocity (m/s)
    turnDegrees= 60;   % amount to turn when wall hit (degrees)
    turnRadius= 0.3;   % radius when driving around (radians)

    % Start robot moving
    SetFwdVelAngVelCreate(serPort, v, 0)
    
    % Continue until a wall is hit
    [BumpRight, BumpLeft, BumpFront]= continueToWall(serPort);
    
    % Stop robot moving
    SetFwdVelAngVelCreate(serPort, 0, 0)
    
    % If no wall was found, bail
    if (~BumpRight && ~BumpLeft && ~BumpFront)
        disp('did not find a wall, nothing to do!');
        return;
    end
    
    % Record starting position so we know when to stop
    [xStart, yStart, ~]= OverheadLocalizationCreate(serPort);
        
    fprintf('xStart %d, yStart %d\n', xStart, yStart); 
    
    % Init loop variables
    backAtStart= 0;
    tStart= tic;

    % Enter main loop
    while toc(tStart) < maxDuration && ~backAtStart
        
        fprintf('BumpRight %d, BumpLeft %d, BumpFront %d\n', ...
            BumpRight, BumpLeft, BumpFront); 
        
        % turn away from contanct
        if (BumpRight)
            % counter-clockwise
            turnAngle(serPort, turnV, turnDegrees);
            SetFwdVelRadiusRoomba(serPort, v, -turnRadius);
        elseif (BumpFront)
            % counter-clockwise
            turnAngle(serPort, turnV, turnDegrees*2);
            SetFwdVelRadiusRoomba(serPort, v, -turnRadius);            
        else
            % clockwise
            turnAngle(serPort, maxTurnV, -turnDegrees);
            SetFwdVelRadiusRoomba(serPort, v, turnRadius);
        end
        
        % Continue until a wall is hit
        [BumpRight, BumpLeft, BumpFront]= continueToWall(serPort);
        
        % Stop robot moving
        SetFwdVelAngVelCreate(serPort, 0, 0)

        % if we didn't hit anything, bail
        if (~BumpRight && ~BumpLeft && ~BumpFront)
            disp('oh noes! I got incredibly lost!');
            return;
        end
        
        % Check position and see how far we are from start
        [xCur, yCur, ~]= OverheadLocalizationCreate(serPort);
        distFromStart= pdist([xStart, yStart; xCur, yCur], 'euclidean');
        
        fprintf('x %d, y %d, dist %d\n', xStart, yStart, distFromStart);
        
        % Consider back at start if kind of close
        backAtStart= distFromStart < 0.1; 

    end
    
end

function [BumpRight, BumpLeft, BumpFront]= continueToWall(serPort)

    maxDuration= 120; % bail after 2 mins
    tStart= tic;   

    while toc(tStart) < maxDuration
        
        % check to see if we bumped anything
        [BumpRight, BumpLeft, ~, ~, ~, BumpFront] ...
            = BumpsWheelDropsSensorsRoomba(serPort);
        
        % if we hit a wall, all done
        if (BumpRight || BumpLeft || BumpFront)
            return
        end

        % pause to let the robot move
        pause(0.1)
        
    end
        
end
