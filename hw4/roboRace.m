
%% ROBORACE %%%%%%%%%%%%%%%%%%%%%

function roboRace(serPort)
% points has 2 columns and some number of rows
% first row is current point
% following rows are points to go to, in order
% last row is goal point

    % Clear the cache
    clc;                                                          

    % Poll for bump Sensors to avoid getting NaN values and
    % clear the distance and angle sensors
    BumpsWheelDropsSensorsRoomba(serPort);
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort); 
    
    % current position and orientation
    points = [  -3.1070,   -0.5800;
   -2.0334,   -0.5089;
   -1.5634,   -0.5089;
    0.9965,   -0.8488;
    5.0390,    0.3417;
   10.6570,    0.00 ];
    obstacles = cell(1);
    pos = [points(1,1), points(1,2), 0];
    qGoal = points(size(points,1),:);
    %turn to face the first point
    [pos(3),~] = turnToFacePoint(serPort, pos, points(2,:));
    
    %%loop variables
    atgoal = false;
    status = 1; % drivin straight
    currentStraightDist = 0;
    currentDeltaAngle = 0;
    idx = 3;
    recentLeaveDist = 0;
    distToLine = 0; % if we bump we need this guy
    compensateTurn = 1; %same deal, for bump stuff
    
    %%first calculation
    [data, recentLeaveDist] = computeDistAndAngle(serPort, points(idx-2,:), ...
        points(idx-1,:), points(idx,:), recentLeaveDist, pos(3));
    
    %%begin main loop
    while (~atgoal)

        Dist = DistanceSensorRoomba(serPort);
        Angle = AngleSensorRoomba(serPort);  
        Bump = checkForBump(serPort);
        
        % update odometry and plot position
        pos(3) = mod(pos(3) + Angle, 2*pi);
        pos(1) = pos(1) + Dist * cos(pos(3));
        pos(2) = pos(2) + Dist * sin(pos(3));
        
        % print position
        fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));
        
        if (Bump && ~(status==4))
            disp('oh no bumped!!');
            stopRobot(serPort);
            a = points(idx-2,:);
            b = points(idx-1,:);
            c = points(idx,:);
            pos(1,1:2) = getClosestPoint(obstacles,pos);
            [whereIProbablyShouldBe, distToLine] = pointOnLine(a,b,pos(1,1:2));
            %tricky because we have to get back into the system
            [pos(3), compensateTurn] = turnToFacePoint(serPort, pos, whereIProbablyShouldBe);
            currentStraightDist = 0;
            status = 4;
            AngleSensorRoomba(serPort); %just to clear this shizzle cuz who knows
            DistanceSensorRoomba(serPort);
            disp('CHANGE TO STATE 4');
            Bump = checkForBump(serPort);
        end
        
        breakEarlyAngCompensate =0;% 5*pi/180;
        breakEarlyDistCompensate = 0.1;
        
        switch status
                
            case 1 % driving straight
                moveForward(serPort);
                
                currentStraightDist = currentStraightDist + Dist;
                if(currentStraightDist >= data(1) - breakEarlyDistCompensate)
                   currentStraightDist = 0;
                   status = 2;
                   disp('CHANGE TO STATE 2');
                end
                
            case 2 % turning
                
                %how far we've turned in this particular arc
                currentDeltaAngle = currentDeltaAngle + Angle;
                
                if (abs(currentDeltaAngle) >= abs(data(2)*(pi/180))-breakEarlyAngCompensate)
                    moveForward(serPort);
                    currentDeltaAngle = 0;
                    idx = idx + 1;
                    %this means we have only to head straight to the goal
                    if (idx > size(points,1))
                        status = 3;
                        data(1) = getDistance(pos,qGoal);
                        disp('CHANGE TO STATE 3');
                    else
                        % calculate data with current pos as first point
                        [data, recentLeaveDist] = computeDistAndAngle(serPort, pos, points(idx-1,:), points(idx,:), recentLeaveDist, pos(3));
                        status = 1;
                        disp('CHANGE TO STATE 1');
                    end
                else
                    positive = data(2) > 0;
                    turnRobot(serPort, positive);
                end
            case 3 %driving straight for some distance to the goal
                moveForward(serPort);
                currentStraightDist = currentStraightDist + Dist;
                %might as well limit our distance
                if(currentStraightDist >= data(1))
                   atgoal = true; 
                end
            case 4 %try and get to the line we should be on
                moveForward(serPort);
                currentStraightDist = currentStraightDist + Dist;
                if(Bump)
                    stopRobot(serPort);
                    turnRadians(serPort, compensateTurn/4);
                    AngleSensorRoomba(serPort); %clear it so the pos(3) never gets updated
                end
                if(currentStraightDist >= distToLine)
                   stopRobot(serPort);
                   status = 5;
                   disp('CHANGE TO STATE 5');
                end
            case 5 %turn to face "point b", drive to point b, turn to c
                pos(3) = turnToFacePoint(serPort, pos, b);
                quickDist = driveDistance(serPort, norm(b-pos(1,1:2)));
                pos(1) = pos(1) + quickDist * cos(pos(3));
                pos(2) = pos(2) + quickDist * sin(pos(3));
                pos(3) = turnToFacePoint(serPort, pos, c);
                idx = idx + 1;
                %this means we have only to head straight to the goal
                if (idx > size(points,1))
                    status = 3;
                    data(1) = getDistance(pos,qGoal);
                    disp('CHANGE TO STATE 3');
                else
                    % calculate data with current pos as first point
                    [data, recentLeaveDist] = computeDistAndAngle(serPort,...
                        pos, points(idx-1,:), points(idx,:), 0, pos(3));
                    status = 1;
                    disp('CHANGE TO STATE 1');
                end
        end
    end
    
    disp('DONE!');
    stopRobot(serPort);

end

function Bump = checkForBump(serPort)
% simply reads the bump sensors
%
% Input:
% serPort - for robot access
%
% Output: 
% whether or not any of the bump sensors are true
    
    [BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

    % handle possible NaN
    while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
        [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
            = BumpsWheelDropsSensorsRoomba(serPort);
    end

    Bump = BumpRight || BumpLeft || BumpFront;
end

function turnRobot(serPort, positive) 
% turns the robot
%
% Input:
% serPort - for robot access
% positive - whether to turn positive or negative
%
% Output: 
% none
    
    turnSpeed = .3; % in rads/s
    turnFwdSpeed = .1; % in rads/s
    if(positive)
        SetFwdVelAngVelCreate(serPort,turnFwdSpeed,turnSpeed);
    else
        SetFwdVelAngVelCreate(serPort,turnFwdSpeed,-turnSpeed);
    end

end

function distGone = driveDistance(serPort, totalDist)
% drives a certain distance and stops
%
% Input:
% serPort - for robot access
% totalDist - distance to travel before stopping
%
% Output: 
% actual distance traveled    

    done = false;
    distGone = 0;
    while(~done)
        dist = DistanceSensorRoomba(serPort);
        distGone = distGone + dist;
        moveForward(serPort); 
        if(distGone >= totalDist)
            done = true;
        end
    end
    stopRobot(serPort);
end

function stopRobot(serPort)
% stops the robot
%
% Input:
% serPort - for robot access
%
% Output: 
% none
    
    SetFwdVelAngVelCreate(serPort,0.00001,0.00001);
end

function moveForward(serPort)
% drive straight with compensation
%
% Input:
% serPort - for robot access
%
% Output:
% none
    
    angSpeedCompensate = .02713;
    if (rand(1) > .8)
        angSpeedCompensate = .02714;
    end
    fwdSpeed = .3;
    if isa(serPort,'CreateRobot')
        angSpeedCompensate = 0;
        fwdSpeed = .4;
    end
    SetFwdVelAngVelCreate(serPort,fwdSpeed,angSpeedCompensate);
end

function [data, newRecentLeave] = computeDistAndAngle(serPort, A, B, C, recentLeaveDist, angle)
% Compute dist to travel and amount to turn for corner cutting system
%
% Input:
% serPort for checking if real robot
% A,B,C - points, B is the center point
% recentLeaveDist - how far onto the line A-B we already are
% angle - current Angle
%
% Output:
% data -dist to travel and amount to turn
% newRecentLeave - how far onto new line we will end up

% we can break this up into a series of distances and angles
% the distances represent driving straight, the angles are
% using our "standard" turn radius which can be set
% depending on our turn radius, the distances changes

    %stuff for the real robot
    measuredTurnDiameter = .98; %old.. 1.1938
    robotDiameter = .335;
    turnRadius = (measuredTurnDiameter-robotDiameter)/2;
    
    if isa(serPort,'CreateRobot')
        %set the constants for simulator only
        turnSpeed = .3; % must be same as above in turnrobot
        turnFwdSpeed = .1; % must be same as above in turnrobot
        turnRadius = (turnFwdSpeed * ((2*pi)/turnSpeed))/pi/2;
    end
    
    %because we might not be facing exactly the right way, we have to
    %modify our point B to B' that we will actually hit given our
    %trajectory
        disp(A); disp(B); disp(C);
    p = A(1,1:2);
    q = C;
    r = [cos(angle), sin(angle)];
    s = B - C;
    
    %find t and u such that p + t r = q + u s
    %t = (q ? p) × s / (r × s)
    t = cross2d((q-p), (s / cross2d(r,s)));
    
    newB = p + t*r;
    
    if (newB(1) < p(1))
        disp('FUCK');
        newB = B;
    end
    
    data = zeros(1,2);

    myangle = computeAngleBetweenPoints(A,newB,C);
    data(2) = myangle;

    mydist = getDistance(A,newB);
    leaveDist = computeLeaveLineDist(myangle,turnRadius);
    mydist = mydist - leaveDist - recentLeaveDist;
    newRecentLeave = leaveDist;

    data(1) = mydist;
    
    fprintf('ANGLE %.3f, DIST %.3f, NEWB: (%.3f, %.3f)\n', myangle, mydist, newB(1), newB(2));
    
end

function result = cross2d(v1,v2)
    result = v1(1) * v2(2) - v1(2)*v2(1);
end

function angle = computeAngleBetweenPoints(A, B, C)
% Find angle between three points
%
% Input:
% A,B,C - points, B is the center point
%
% Output:
% angle - angle in radians

    ABrise = B(2)-A(2);
    ABrun = B(1)-A(1);
    ABangle = atan2d(ABrise,ABrun);
    
    BCrise = C(2)-B(2);
    BCrun = C(1)-B(1);
    BCangle = atan2d(BCrise,BCrun);
    
    angle = BCangle-ABangle;
    if(angle > 180)
       angle = 180 - angle; 
    elseif(angle < -180)
        angle = 360 + angle;
    end

end

function dist = computeLeaveLineDist(angle, radius)
    angle = abs(angle/2);
    dist = tand(angle)*radius;
end

function mylikelypoint = getClosestPoint(obstacles,pos)
% Find likely collision vertex
%
% Input:
% obstacles - coordinates of the obstacles
% pos - where we thought we were
%
% Output:
% mylikelypoint - closest vertex in obstacles to pos
    
    bestDistance = 1000000;
    mylikelypoint = [0,0];
    for i= 1:size(obstacles,2)
        obstacle = obstacles{i};
        for j=1:size(obstacle,1)
            vertex = obstacle(j,:);
            currentDist = getDistance(pos,vertex);
            if(currentDist < bestDistance)
                bestDistance = currentDist;
                mylikelypoint = vertex;
            end
        end
    end
end

function [intersection,distanceToLine] = pointOnLine(lineStart, lineEnd, point)
% Closest point on a line from another point
%
% Input:
% lineStart - first point (x,y) of line
% lineEnd - second point (x,y) of line
% point - the point not on the line
%
% Output:
% intersection - closest point on the line
% distanceToLine - distance from point to intersection

    line = (lineEnd-lineStart);
    line = line/norm(line);
    
    r = lineStart - point;
    v = [line(2), -line(1)]; %perp to line
    
    %make sure v is pointing from point
    currentDist = getDistance(point, lineStart);
    newDist = getDistance(point + v,lineStart);
    if(newDist > currentDist)
        v = v*-1;
    end
    
    distanceToLine = dot(v,r);
    intersection = point + v*distanceToLine;
    distanceToLine = abs(distanceToLine);

end

function [newAng, compensateAng] = turnToFacePoint(serPort, pos, qGoal)
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

function angTurned = turnRadians(serPort, angToTurn)
% Perform an acurate in place turn.
%
% Input:
% serPort - Serial port for communicating with robot
% angToTurn - Angle to turn, positive is counter-clockwise (rad)
%
% Output:
% angTurned - Actual angle turned (rad)

    % compensate for real robot
    angToTurn = 0.9*angToTurn;

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

function dist = getDistance(point1, point2)
% Euclidean distance between two points.
%
% Input:
% point1 - first point (x,y)
% point2 - second point (x,y)
%
% Output:
% dist - distnace between points (m)

    dist = pdist([point1(1),point1(2);point2(1),point2(2)], 'euclidean'); 

end
