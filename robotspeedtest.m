function robotspeedtest(serPort)
    
    DistanceSensorRoomba(serPort);
    SetFwdVelAngVelCreate(serPort, .05, 0);
    count = 0;
    while(count < 10)
       
        %distance_temp = DistanceSensorRoomba(serPort); 
        %fprintf('distance: %.3f \n' , distance_temp);
        
        %[BumpRight, BumpLeft, ~, ~, ~, BumpFront] ...
         %= BumpsWheelDropsSensorsRoomba(serPort);
        %disp('bumped?');
        
        %Wall = WallSensorReadRoomba(serPort);
        %disp('wallll');
        
        [BumpRight, BumpLeft, BumpFront, Wall, virtWall, CliffLft, ...
    CliffRgt, CliffFrntLft, CliffFrntRgt, LeftCurrOver, RightCurrOver, ...
    DirtL, DirtR, ButtonPlay, ButtonAdv, Dist, Angle, ...
    Volts, Current, Temp, Charge, Capacity, pCharge]   = AllSensorsReadRoomba(serPort);
        fprintf('distance: %.3f  wall: %f \n' , Dist, Wall);
        
        
        count = count +1;
    end
        
    SetFwdVelAngVelCreate(serPort, 0, 0);



end