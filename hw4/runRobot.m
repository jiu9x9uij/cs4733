
function runRobot(serPort) % add points back soon!
% points has 2 columns and some number of rows
% first row is current point
% following rows are points to go to, in order
% last row is goal point
% EXAMPLE:
%{
    points = [
       -3.1070  ,  0.5800;
       -2.0000  ,  1.0000;
        1.0000  ,  3.0000;
        3.0000  ,  1.0000;
        4.0000  ,  3.0000;
       10.6570  , -0.0300
    ]
%}

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
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);
    %=============================================================%


    %%just for now!
    points = [0 0; 1 1; 1 3; 3 3; 4 5; 6 2]; 


    %% CONSTANTS YO

    turnSpeed = .5; % in rads/s
    fwdSpeed = .4; % in m/s
    turnRadius = (fwdSpeed * ((2*pi)/turnSpeed))/pi/2;

    % we can break this up into a series of distances and angles
    % the distances represent driving straight, the angles are
    % using our "standard" turn radius which can be set
    % depending on our turn radius, the distances changes

    %% so we walk through the group, 3 points at a time
    dataMatrix = zeros(size(points,1)-2, 2);
    count = 1;
    recentLeaveDist = 0;
    for i=3:size(points, 1)

        A = points(i-2,:);
        B = points(i-1,:);
        C = points(i,:);


        myangle = computeAngleBetweenPoints(A,B,C);
        dataMatrix(count, 2) = myangle;

        mydist = pdist([A(1),A(2);B(1),B(2)], 'euclidean'); 
        leaveDist = computeLeaveLineDist(myangle,turnRadius);
        mydist = mydist - leaveDist - recentLeaveDist;
        recentLeaveDist = leaveDist;

        dataMatrix(count,1) = mydist;
        count = count + 1;
    end

    disp(dataMatrix);
    %disp('woot!');

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
    idx = 1;
    
    %%begin main loop
    while(~atgoal)
        
        data = dataMatrix(idx,:);
        
        % read all the sensors at once
        [BumpRight, BumpLeft, BumpFront, Wall, ~, ~, ...
        ~, ~, ~, ~, ~,~, ~, ~, ~, Dist, Angle, ...
        ~, ~, ~, ~, ~, ~]  = AllSensorsReadRoomba(serPort);
    
        % handle possible NaN
        while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
            Wall = WallSensorReadRoomba(serPort);
        end
        
        % update odometry and plot position
        pos(3) = mod(pos(3) + Angle, 2*pi);
        pos(1) = pos(1) + Dist * cos(pos(3));
        pos(2) = pos(2) + Dist * sin(pos(3));
        
        
        switch status
            case 1 % driving straight
                disp('driving straight!');
                SetFwdVelAngVelCreate(serPort,fwdSpeed,0);
                
                currentStraightDist = currentStraightDist + Dist;
                %TODO better than >= ?
                if(currentStraightDist >= data(1))
                   currentStraightDist = 0;
                   status = 2;
                end
                
            case 2 % turning
                disp('turning!');
                if(data(2) > 0)
                    SetFwdVelAngVelCreate(serPort,fwdSpeed,turnSpeed);
                else
                    SetFwdVelAngVelCreate(serPort,fwdSpeed,-turnSpeed);
                end
                
                currentDeltaAngle = currentDeltaAngle + Angle;
                if(abs(currentDeltaAngle) >= abs(data(2) * (pi/180)))
                    %when i'm done turning, calc next data
                    % for now we can ++ idx
                    currentDeltaAngle = 0;
                    idx = idx + 1;
                    status = 1;
                end
        end
       
        
        
        %atgoal = atPoint(pos, qGoal, duration)
        drawnow;
    end
    
    
    
end


function angle = computeAngleBetweenPoints(A, B, C)
    
    ABrise = B(2)-A(2);
    ABrun = B(1)-A(1);
    ABangle = atand(ABrise/ABrun);
    
    BCrise = C(2)-B(2);
    BCrun = C(1)-B(1);
    BCangle = atand(BCrise/BCrun);
    
    
    angle = BCangle-ABangle;

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
    actualTurn = turnRadians(serPort, compensateAng);
    newAng = mod(pos(3) + actualTurn, 2*pi);

    disp('Completed turnToFacePoint');
    
end

function isAtPoint = atPoint(pos, qGoal, duration)
% Determine if we're at goal position, which depends on program duration.
%
% Input:
% pos - Current position of robot (x, y, theta)
% qGoal - Goal position (x, y)
% duration - How long program has been running (s)
%
% Output:
% isAtPoint - True if robot is considered at goal

    distCushion = 0.2 + (duration/60)*0.04;
    dist = getDistance(pos, qGoal); 
    isAtPoint = dist < distCushion; 
    
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