function angletest()

disp('(0,0), (-1,-1), (-2,-2)');
computeAngleBetweenPoints([0,0], [-1,-1], [-2,-2]);


disp('(0,0), (0,-1), (0,1)');
computeAngleBetweenPoints([0,0], [0,-1], [0,1]);


disp('(0,0), (-1,1), (-1,2)');
computeAngleBetweenPoints([0,0], [-1,1], [-1,2]);



end


function angle = computeAngleBetweenPoints(A,B, C)
    
    ABrise = B(2)-A(2);
    ABrun = B(1)-A(1);
    ABangle = atan2d(ABrise,ABrun);
    disp('AB ANGLE');
    disp(ABangle);
    
    BCrise = C(2)-B(2);
    BCrun = C(1)-B(1);
    BCangle = atan2d(BCrise,BCrun);
    disp('BC ANGLE');
    disp(BCangle);
    
    angle = BCangle-ABangle;
    if(angle > 180)
       angle = 180 - angle; 
    elseif(angle < -180)
        angle = 360 + angle;
    end
    disp('FINAL ANGLE');
    disp(angle);
    

end