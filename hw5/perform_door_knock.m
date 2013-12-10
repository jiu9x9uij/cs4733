
function perform_door_knock(serPort)

    fwdSpeed = 0.2;
    time_back = 0.5;
    
    drive_to_bump(serPort, fwdSpeed);
    
    drive_time(serPort, -fwdSpeed, time_back);
    
    drive_to_bump(serPort, fwdSpeed);
    
    drive_time(serPort, -fwdSpeed, time_back);
    
    BeepRoomba(serPort);
    pause(1);
    drive_distance(serPort, fwdSpeed, 1.5);
    
end

function drive_to_bump(serPort, fwdSpeed)

    drive_straight(serPort, fwdSpeed);
    
    bumped = checkForBump(serPort);
    while ~bumped
        bumped = checkForBump(serPort);
    end
    
    stop_robot(serPort);
    
end

function drive_to_door(serPort, angle)

    dist_to_wall = 0.9;
    fwdSpeed = 0.2;
    
    dist_to_drive = dist_to_wall/tand(abs(angle));
    disp('DIST TO DRIVE');
    disp(dist_to_drive);
    
    drive_distance(serPort, fwdSpeed, dist_to_drive);
    
end

function face_door(serPort, angle)

    if angle > 0
        turn_angle(serPort, pi/2);
    else
        turn_angle(serPort, -pi/2);
    end
    
end


%% ROBOT UTILITIES %%%%%%%%%%%%%%%%%%%%%

function drive_distance(serPort, fwdSpeed, dist)

    DistanceSensorRoomba(serPort);

    totalDist = 0;
    
    drive_straight(serPort, fwdSpeed);
        
    while (totalDist < dist)        
        totalDist = totalDist + DistanceSensorRoomba(serPort);
    end
    
    stop_robot(serPort);

end

function drive_time(serPort, fwdSpeed, time)

    t = tic;
    drive_straight(serPort, fwdSpeed);
    while (toc(t) < time)
    end
    
    stop_robot(serPort);
    
end

function drive_straight(serPort, fwdSpeed)

    angCompensate = 0.0904;

    SetFwdVelAngVelCreate(serPort, fwdSpeed, angCompensate*fwdSpeed);

end

function stop_robot(serPort)

    SetFwdVelAngVelCreate(serPort, 0, 0);
    
end

function [angTurned] = turn_angle(serPort, angToTurn)
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

function [bumped] = checkForBump(serPort)
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

    bumped = BumpRight || BumpLeft || BumpFront;
end




%% PLOTTING %%%%%%%%%%%%%%%%%%%%%

function plot_position(pos)
% Plots (x,y,theta), with position in blue and orientation in green.
%
% Input:
% pos - Robot position to display

    % draw on figure 1
    figure(1);
    hold on;
    
    % plot position
    plot(pos(1), pos(2), 'b.');
    
    % plot orientation
    dispOrientation = 0.25;
    plot([pos(1),pos(1)+dispOrientation*cos(pos(3))], ...
         [pos(2),pos(2)+dispOrientation*sin(pos(3))], 'g');
    
    % flush plot
    drawnow;
     
end
