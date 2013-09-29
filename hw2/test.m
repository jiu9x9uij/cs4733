function test(pos)
    qGoal = [1,1];
    
    % handle special case where already at goal
    if (pos(1) == qGoal(1) && pos(2) == qGoal(2))
        disp('no turn');
        return;
    end

    invTan = atan((pos(2)-qGoal(2))/(pos(1)-qGoal(1)));
    
    if (pos(1) >= qGoal(1))
        compensateAng = pi+invTan;
        compensateAng = compensateAng - pos(3);
    else
        compensateAng = invTan;
        compensateAng = compensateAng - pos(3);
    end
    
    disp(compensateAng*180/pi);
    
end
