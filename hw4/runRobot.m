
function runRobot(serPort, points)
% points has 2 columns and some number of rows
% first row is current point
% following rows are points to go to, in order
% last row is goal point
% EXAMPLE:
%{
    points = [
       -3.1070  ,  0.5800;
       -2.0000  ,  1.0000;
        1.0000  ,  3.0000;
        3.0000  ,  1.0000;
        4.0000  ,  3.0000;
       10.6570  , -0.0300
    ]
%}

    % current position and orientation
    pos = [points(1,1), points(1,2), 0];

    % loop through points, go to them one at a time
    for i=2:size(points, 1)
        
        next_point = points(i,:);
        
    end
    
end
