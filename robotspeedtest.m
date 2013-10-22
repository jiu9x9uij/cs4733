function robotspeedtest(serPort)

    display('Turning 180 degrees at .15r/s');
    pause(1);
    turnAngle(serPort, .15, 180);
    pause(1);
    display('Turning 180 degrees at .1r/s');
    pause(1);
    turnAngle(serPort, .1, 180);
    pause(1);
    display('Turning 180 degrees at .05r/s');
    pause(1);
    turnAngle(serPort, .05, 180);
    pause(1);
    
    DistanceSensorRoomba(serPort);
    SetFwdVelAngVelCreate(serPort, .05, 0);
    count = 0;
    while(count < 10)
        [BumpRight, BumpLeft, BumpFront, Wall, virtWall, CliffLft, ...
         CliffRgt, CliffFrntLft, CliffFrntRgt, LeftCurrOver, RightCurrOver, ...
         DirtL, DirtR, ButtonPlay, ButtonAdv, Dist, Angle, ...
         Volts, Current, Temp, Charge, Capacity, pCharge] ...
            = AllSensorsReadRoomba(serPort);
        
        fprintf('distance: %.3f  wall: %f \n' , Dist, Wall);

        count = count +1;
    end
        
    SetFwdVelAngVelCreate(serPort, 0, 0);

end