
function test(serPort) % add points back soon!
    clc;                                                          % Clear the cache
    AllSensorsReadRoomba(serPort);

    angSpeedCompensate = 0.081;
    turnSpeed = .45; % in rads/s
    fwdSpeed = .4; % in m/s
    turnFwdSpeed = .2; % in rads/s
    turnRadius = 1.1938;
    robotDiameter = .335;
    turnRadius = (turnRadius - robotDiameter)/2;
    done = false;
    pos = [0,0,0];
    start = [0,0,0];
    
    while(~done)

        % read all the sensors at once
        [BumpRight, BumpLeft, BumpFront, ~, ~, ~, ...
        ~, ~, ~, ~, ~,~, ~, ~, ~, Dist, Angle, ...
        ~, ~, ~, ~, ~, ~]  = AllSensorsReadRoomba(serPort);
    
        % handle possible NaN
        while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        end
        
        pos(3) = mod(pos(3) + Angle, 2*pi);
        pos(1) = pos(1) + Dist * cos(pos(3));
        pos(2) = pos(2) + Dist * sin(pos(3));
        
        SetFwdVelAngVelCreate(serPort,fwdSpeed,angSpeedCompensate);
                
        if (getDistance(pos, start) > 5)
            done = true;
        end
        
    end
    
    disp('DONE YES WE BE DONE YO');
    SetFwdVelAngVelCreate(serPort,0.000001,0.0000001);
    
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
