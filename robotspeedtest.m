function robotspeedtest(serPort)

    clc;
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);
    DistanceSensorRoomba(serPort);

    SetFwdVelAngVelCreate(serPort, .3, 0);
    
    while (true)
        [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        
        while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        end
        
        Dist = DistanceSensorRoomba(serPort);

        fprintf('distance: %.3f \n' , Dist);
        
        if (BumpRight || BumpLeft || BumpFront)
            break;
        end
        
    end
    
    SetFwdVelAngVelCreate(serPort, 0, 0);

end