function robotspeedtest(serPort)

    clc;
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);
    DistanceSensorRoomba(serPort);

    angSpeedCompensate = 0.01;
    fwdSpeed = .3;
    
    SetFwdVelAngVelCreate(serPort, fwdSpeed, angSpeedCompensate);
    
    totalDist = 0;
    
    while (totalDist < 5)
        [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        
        while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        end
        
        Dist = DistanceSensorRoomba(serPort);

        totalDist = totalDist + Dist;
        
        fprintf('distance: %.3f \n' , totalDist);
        
        if (BumpRight || BumpLeft || BumpFront)
            break;
        end
    end
    
    SetFwdVelAngVelCreate(serPort, 0, 0);

end