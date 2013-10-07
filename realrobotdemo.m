function realrobotdemo(serPort)


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
    
end