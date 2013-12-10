

function drive_to_and_face_door(serPort, angle)

    dist_to_wall = 0.9;
    
    dist_to_drive = dist_to_wall/tan(abs(angle));
    
    drive_forward(serPort, dist_to_drive);
    
end

function face_door(serPort, angle)

    if angle > 0
        turn_angle(serPort, pi/2);
    else
        turn_angle(serPort, -pi/2);
    end
    
end

function drive_forward(serPort, dist)

    DistanceSensorRoomba(serPort);

    totalDist = 0;
    fwdSpeed = .3;
    
    drive_straight(serPort, fwdSpeed);
        
    while (totalDist < dist)        
        totalDist = totalDist + DistanceSensorRoomba(serPort);
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

function [found, door] = find_door(img)
end
function [angle] = get_angle_from_door(door)
end

function door_finder(serPort)

    fwdSpeed = 0.1;
    found = 0;
    door = 0;
    
    drive_straight(serPort, fwdSpeed);
    
    while ~found
        img = imread('photo1.jpg');
        [found, door] = find_door(img);
    end

    stop_robot(serPort);
    
    angle = get_angle_from_door(door);
    
    drive_to_door(serPort, angle);
    
    face_door(serPort, angle);
    
    img = imread('photo1.jpg');
    [found, door] = find_door(img);

    if ~found
        disp('OH NO, LOST THE DOOR! Starting over and trying again.')
        face_door(serPort, -angle);
        door_finder(serPort);
    end
    
    angle = get_angle_from_door(door);

    turn_angle(serPort, angle);
    
    knock_on_door(serPort);
    
end

function knock_on_door(serPort)

    fwdSpeed = 0.2;
    
    drive_to_bump(serPort, fwdSpeed);
    
    t = tic;
    drive_straight(serPort, -fwdSpeed);
    while (toc(t) < 1)
    end
    stop_robot(serPort);
    
    drive_to_bump(serPort, fwdSpeed);
    
    t = tic;
    drive_straight(serPort, -fwdSpeed);
    while (toc(t) < 1)
    end
    stop_robot(serPort);
    
    drive_forward(serPort, 1);
    
end

function drive_to_bump(serPort, fwdSpeed)

    drive_straight(serPort, fwdSpeed);
    
    Bumped = checkForBump(serPort);
    while ~Bumped
        Bumped = checkForBump(serPort);
    end
    
    stop_robot(serPort);
    
end

function Bumped = checkForBump(serPort)
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

    Bumped = BumpRight || BumpLeft || BumpFront;
end
