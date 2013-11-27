% HW5 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

%% MAIN METHOD %%%%%%%%%%%%%%%%%%%%%

function hw5_team18(serPort)

    % clear any persitent variables in functions
    clear functions;
    
	% clear the cache
    clc;
    
    % poll sensors to avoid getting NaN values and clear odometry
    BumpsWheelDropsSensorsRoomba(serPort);
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort); 
    
    fake_color_tracker(serPort);

end


%% DEAN PUT YOUR STUFF HERE %%%%%%%%%%%%%%%%%%%%%

function fake_color_tracker(serPort)
% DEMO COLOR TRACKER
% Go fwd and right for 6s
% Spin left for 6s
% Go back for 6s
% Stop for 2s
% Done

    tStart = tic;

    while true

        % simulate image read delay
        pause(0.3);
        
        if (toc(tStart) < 6)            
            size_change = 0.5 + rand/2; % rand between 0.5 and 1 (forward)
            horizontal_change = rand;   % rand between 0 and 1 (right)
        elseif (toc(tStart) < 12)
            size_change = 1;            % no forward motion
            horizontal_change = -rand;  % rand between 0 and -1 (left)
        elseif (toc(tStart) < 18)
            size_change = 1 + rand;     % rand between 1 and 2 (backward)
            horizontal_change = 0;      % no angular motion
        elseif (toc(tStart) < 20)
            size_change = 1;            % no forward motion
            horizontal_change = 0;      % no angular motion
        else
            break;
        end
        
        move_robot(serPort, size_change, horizontal_change);
        
    end
    
end



%% MOVE ROBOT %%%%%%%%%%%%%%%%%%%%%

function move_robot(serPort, size_change, horizontal_change)
% Update robot based on new location of color marker.
%
% Input:
% size_change - Ratio of new_size_length to orig_side_length. So if orig
%   is 20x20 pixels, and new is 30x30, should be 1.5. Should be > 0.
% horizontal_change - Ratio of centroid_horizontal_movement to max centroid
%   horizontal movement, which is based on image size. Should be -1<x<1.

    % keep track of position for debugging
    persistent pos;
    if isempty(pos)
        pos = [0,0,0];
    end

    % read sensors
    dist = DistanceSensorRoomba(serPort);
    ang = AngleSensorRoomba(serPort);  
    [br, bl, bf] = check_for_bump(serPort);

    % update odometry
    pos(3) = mod(pos(3) + ang, 2*pi);
    pos(1) = pos(1) + dist * cos(pos(3));
    pos(2) = pos(2) + dist * sin(pos(3));
    
    % print odometry
    % fprintf('(%.3f, %.3f, %.3f)\n', pos(1), pos(2), pos(3)*(180/pi));
    plot_position(pos);

    % handle bump
    if (br || bl || bf)
        disp('OH NO I BUMPED!');
        exit(1);
    end
    
    % update robot velocities
    [fwd_vel, ang_vel] = get_robot_vel(size_change, horizontal_change);
    SetFwdVelAngVelCreate(serPort, fwd_vel, ang_vel);

    % print velocities
    fprintf('FWD VEL: %.3f, ANG VEL: %.3f\n', fwd_vel, ang_vel);

end

function [fwd_vel, ang_vel] = get_robot_vel(size_change, horizontal_change)
% Compute robot speed based on new location of color marker.
%
% Input:
% size_change - Ratio of new_size_length to orig_side_length. So if orig
%   is 20x20 pixels, and new is 30x30, should be 1.5. Should be > 0.
% horizontal_change - Ratio of centroid_horizontal_movement to max centroid
%   horizontal movement, which is based on image size. Should be -1<x<1.

    % CONSTANTS
    MAX_FWD_VEL = 0.4; % max allowable forward velocity (m/s)
    MAX_ANG_VEL = 0.4; % max allowable angular velocity (rad/s)
    
    % DESIRED OUTPUTS BASED ON INPUTS
    % 
    % size_change --> fwd_vel
    % 1           --> 0
    % 0.5         --> MAX_FWD_VEL
    % 2           --> -MAX_FWD_VEL
    %
    % horizontal_change --> ang_vel
    % 0                 --> 0
    % -1                --> MAX_ANG_VEL
    % 1                 --> -MAX_ANG_VEL

    if size_change > 2
        fwd_vel = -MAX_FWD_VEL;
    elseif size_change < 0.5
        fwd_vel = MAX_FWD_VEL;
    elseif size_change > 1
        fwd_vel = -MAX_FWD_VEL * (size_change - 1);
    elseif size_change < 1
        fwd_vel = 2 * MAX_FWD_VEL * (1 - size_change);
    else
        fwd_vel = 0;
    end
    
    ang_vel = horizontal_change * -MAX_ANG_VEL;
    
end

function [BumpRight, BumpLeft, BumpFront] = check_for_bump(serPort)
% Reads the bump sensors, handling possible NaN.
%
% Input:
% serPort - for robot access
%
% Output: 
% Bump sensor readings.
    
    [BumpRight,BumpLeft,~,~,~,BumpFront] ...
        = BumpsWheelDropsSensorsRoomba(serPort);

    while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
        [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
            = BumpsWheelDropsSensorsRoomba(serPort);
    end
    
end


%% PLOTTING %%%%%%%%%%%%%%%%%%%%%

function plot_position(pos)
% Plots (x,y,theta), with position in blue and orientation in green.
%
% Input:
% pos - Robot position to display

    % draw on figure 1
    figure(1);
    hold on;
    
    % plot position
    plot(pos(1), pos(2), 'b.');
    
    % plot orientation
    dispOrientation = 0.25;
    plot([pos(1),pos(1)+dispOrientation*cos(pos(3))], ...
         [pos(2),pos(2)+dispOrientation*sin(pos(3))], 'g');
    
    % flush plot
    drawnow;
     
end
