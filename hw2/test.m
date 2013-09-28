function test(serPort)
    qGoal = [5,0];
    pos = [0,0,-1/2*pi];
    turnToFacePoint(serPort, pos, qGoal);
end

function compensateAng = turnToFacePoint(pos, qGoal)

    % calculate what we need to turn
    compensateAng = pi+atan((pos(2)-qGoal(2))/(pos(1)-qGoal(1)))-pos(3);
    compensateAng = mod(compensateAng, 2*pi);

end
