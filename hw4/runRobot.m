
function runRobot(serPort) % add points back soon!
% points has 2 columns and some number of rows
% first row is current point
% following rows are points to go to, in order
% last row is goal point

%% DESCRIPTION %%%%%%%%%%%%%%%%%%%
    % Get through the course!!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%
    %=============================================================%
    % Clear cache & avoid NaN values                              %
    %=============================================================%
    clc;                                                          % Clear the cache

    % Poll for bump Sensors to avoid getting NaN values when the 
    % robot first hits a wall
DistanceSensorRoomba(serPort);
AngleSensorRoomba(serPort); 
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);
    %=============================================================%

    %%just for now!
    points = [0 0; 1 1; 1 3; 3 3; 3 5; 2 7]; 
    
    

    %% CONSTANTS YO

    angSpeedCompensate = 0.01; % old.. .081
    turnSpeed = .3; % in rads/s %old..  .45
    fwdSpeed = .2; % in m/s
    turnFwdSpeed = .1; % in rads/s %old.. .2
    measuredTurnDiameter = .996; %old.. 1.1938
    robotDiameter = .335;
    turnRadius = (measuredTurnDiameter-robotDiameter)/2;
    tStart = tic; % for keeping track of time

    %%NOW WE BEGIN ACTUALLY MOVING THE ROBO

    % current position and orientation
    pos = [points(1,1), points(1,2), 0];
    qGoal = points(size(points,1),:);
    %turn to face the first point
    pos(3) = turnToFacePoint(serPort, pos, points(2,:));
    
    %%loop variables
    atgoal = false;
    status = 1; % drivin straight
    currentStraightDist = 0;
    currentDeltaAngle = 0;
    idx = 3;
    recentLeaveDist = 0;
    
    %%first calculation
    [data, recentLeaveDist] = computeDistAndAngle(points(idx-2,:), ...
        points(idx-1,:), points(idx,:), recentLeaveDist, turnRadius);
    
    %%begin main loop
    while (~atgoal)

        Dist = DistanceSensorRoomba(serPort);
        Angle = AngleSensorRoomba(serPort);    
    
        % read all the sensors at once
        %{
        [BumpRight, BumpLeft, BumpFront, ~, ~, ~, ...
        ~, ~, ~, ~, ~,~, ~, ~, ~, Dist, Angle, ...
        ~, ~, ~, ~, ~, ~]  = AllSensorsReadRoomba(serPort);
    
        % handle possible NaN
        while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        end
        %}
        
        % update odometry and plot position
        pos(3) = mod(pos(3) + Angle, 2*pi);
        pos(1) = pos(1) + Dist * cos(pos(3));
        pos(2) = pos(2) + Dist * sin(pos(3));
        
        fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));
        
        switch status
                
            case 1 % driving straight
                %disp('driving straight!');
                SetFwdVelAngVelCreate(serPort,fwdSpeed,angSpeedCompensate);
                
                currentStraightDist = currentStraightDist + Dist;
                %TODO better than >= ?
                if(currentStraightDist >= data(1) * .75)
                   currentStraightDist = 0;
                   status = 2;
                   disp('CHANGE TO STATE 2');
                end
                
            case 2 % turning
                %disp('turning!');
                if(data(2) > 0)
                    %disp('positive angle');
                    SetFwdVelAngVelCreate(serPort,turnFwdSpeed,turnSpeed);
                else
                    %disp('negative angle');
                    SetFwdVelAngVelCreate(serPort,turnFwdSpeed,-turnSpeed);
                end
                
                currentDeltaAngle = currentDeltaAngle + Angle;
                if(abs(currentDeltaAngle) >= .75*abs(data(2) * (pi/180)))
                    currentDeltaAngle = 0;
                    idx = idx + 1;
                    if(idx > size(points,1))
                        status = 3;
                        data(1) = getDistance(pos,qGoal);
                        disp('CHANGE TO STATE 3');
                    else
                        % calculate data with current pos as first point
                        [data, recentLeaveDist] = computeDistAndAngle(pos, points(idx-1,:), points(idx,:), recentLeaveDist, turnRadius);
                        status = 1;
                        disp('CHANGE TO STATE 1');
                        disp('DATA: ');
                        disp(data);
                    end
                end
            case 3 %driving straight but to the goal
                SetFwdVelAngVelCreate(serPort,fwdSpeed,0);
                currentStraightDist = currentStraightDist + Dist;
                %might as well limit our distance
                if(currentStraightDist >= data(1))
                   atgoal = true; 
                end
        end
        % printPosition(pos);
    end
    
    disp('DONE YES WE BE DONE YO');
        
end

function [data, newRecentLeave] = computeDistAndAngle(A, B, C, recentLeaveDist, turnRadius)
    % we can break this up into a series of distances and angles
    % the distances represent driving straight, the angles are
    % using our "standard" turn radius which can be set
    % depending on our turn radius, the distances changes

    data = zeros(1,2);

    myangle = computeAngleBetweenPoints(A,B,C);
    data(2) = myangle;

    mydist = getDistance(A,B);
    leaveDist = computeLeaveLineDist(myangle,turnRadius);
    mydist = mydist - leaveDist - recentLeaveDist;
    newRecentLeave = leaveDist;

    data(1) = mydist;
end

function angle = computeAngleBetweenPoints(A, B, C)
    
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
    %actualTurn = turnRadians(serPort, compensateAng);
    %newAng = mod(pos(3) + actualTurn, 2*pi);
    turnAngle(serPort, 0.1, .7*compensateAng);
    AngleSensorRoomba(serPort);
    newAng = mod(pos(3) + compensateAng, 2*pi);
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

function printPosition(pos)
% Displays x,y,theta position in readible format.
% Also plots the position in blue and orientation in green.
%
% Input:
% pos - Position to display
    figure(1); % draw odometry on new plot
    hold on;   % draw all points on the new plot

    %fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));
    
    % plot position
    plot(pos(1), pos(2), 'b.');
    
    % plot orientation
    dispOrientation = 0.25;
    plot([pos(1),pos(1)+dispOrientation*cos(pos(3))], ...
         [pos(2),pos(2)+dispOrientation*sin(pos(3))], 'g');
    
    drawnow;
     
end
