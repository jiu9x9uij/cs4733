function robotspeedtest(serPort)

    clc;
    [~, ~, ~, ~, ~, ~] = BumpsWheelDropsSensorsRoomba(serPort);
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    angSpeedCompensate = .02713;
    fwdSpeed = .3;
    
    SetFwdVelAngVelCreate(serPort, fwdSpeed, angSpeedCompensate);
    
    totalDist = 0;
    totalAng = 0;
    
    while (totalDist < 7)
        [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        
        while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
            [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
                = BumpsWheelDropsSensorsRoomba(serPort);
        end
        
        Dist = DistanceSensorRoomba(serPort);
        Ang = AngleSensorRoomba(serPort);

        totalDist = totalDist + Dist;
        totalAng = totalAng + Ang;
        
        fprintf('distance: %.3f, ang: %.3f \n' , totalDist, totalAng);
        
        if (BumpRight || BumpLeft || BumpFront)
            break;
        end
    end
    
    SetFwdVelAngVelCreate(serPort, 0, 0);

end