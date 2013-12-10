% HW5 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

%% METHOD FROM ASSIGNMENT %%%%%%%%%%

%{
a. You will initiate the color tracker manually. Take an image and have the user click on 
the image color you want to track. This will give you a threshold range for color 
segmentation. 

b. Threshold the image, and find the largest blob in the image which will be your target 
(use a large enough target). - apply_mask/find_largest_blob

c. Calculate the centroid and area of the blob (in pixels), and compare it to the previous 
centroid and area of this blob. If they don?t change, the target hasn?t moved. If they do, 
you need to move your robot to either increase or decrease the blob area (move 
forward and back) and rotate to keep the blob centered. 

d. You will have to play a bit with the gains on your robot?s movement, i.e. how fast and 
far to move to re-adjust the image. Keep in mind you are doing this continuously as 
each image frame is read in real-time. Given the web link for the images, you probably 
will only get 2 or 3 frames per second which will help determine how fast to move the 
robot. You can also reduce the camera resolution to allow faster processing and a 
possibly higher frame rate. 

e. Move the marker to make the robot follow you. Show that it will stop when you stop, 
and turn when you turn, etc.
%}


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
    
    
    % now do either part 1 or part 2 based on input
    
    
    color_tracker(serPort);

end



%% DEAN PUT YOUR STUFF HERE %%%%%%%%%%%%%%%%%%%%%

function color_tracker(serPort)
    
    % one time stuff for user to choose color
    % HAVE USER SELECT COLOR
    startImg = imread('http://10.0.0.23/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0');
    p = ChoosePoint(startImg);
    startSmooth = smooth_image(startImg);
   
    color = startSmooth(p(2),p(1),:);
    color = [color(1), color(2), color(3)];
    %at this point we have the desired color, removing anomolies
    masked = apply_mask(startSmooth, color);
    originalBlob = get_largest_blob(masked);
    [originalPixels, ~] = analyzeBlob(originalBlob);
    while true
        
        %ask for image
        img = getImage('http://10.0.0.23/snapshot.cgi?user=admin&pwd=&resolution=32&rate=0');
        %get correct blob
        masked = apply_mask(img, color);
        blob = get_largest_blob(masked);
        
        
        %analyze blob for horizontal and size
        %[size_change, horizontal_change] = analyzeBlobs(originalBlob, blob);
        [newPixels, centroid] = analyzeBlob(blob);
        imshow(blob);
        size_change = newPixels / originalPixels;
        center = size(img,2)/2;
        horizontal_change = (centroid(2) - center)/center;
        
        %call move robot stuff
        move_robot(serPort, size_change, horizontal_change);
        fprintf('SIZE CHANGE: %.3f    HORIZONTAL:  %.3f\n', size_change, horizontal_change);     
    end


end

function [pixels, centroid] = analyzeBlob(blob)

    pixels = 0;
    centroid = [0,0];
    % area + centroid
    s = size(blob);
    height = s(1);
    width = s(2);
    for row = 1:height
       for col = 1:width
           if(blob(row,col)==1)
              centroid = centroid + [row,col];
              pixels = pixels + 1;
           end
       end
    end

    centroid = centroid / pixels;
end

function [ pos ] = ChoosePoint( img )
%ChosePoint( image )
%   Takes an image, prompt user to choose point, return color of point
    f = figure();
    image(img);
    
    % gets an input from the figure
    % if the user exits the figure instead, try again
    pos = round(ginput(1));
    %disp(p);
    
    % reallocate the color, flipping the x/y coordinates
    
    %color = img(p(2),p(1),:);
    %color = [color(1), color(2), color(3)];
    %disp(color);
    
    close(f);

end


function blob = get_largest_blob(img)
    biggestBlobCount = 0;
    s = size(img);
    height = s(1);
    width = s(2);
    for row = 1:height
       for col = 1:width
           if(img(row,col)==1)
              [maybeBlob, count] = growBlob(img, row,col);
              img = img - maybeBlob;
              if(count > biggestBlobCount)
                  blob = maybeBlob;
                  biggestBlobCount = count;
              end
           end
       end
    end
    imshow(blob);
end


function [blob, count] = growBlob(img, row, col)
    max_size = 300;
    q = zeros(max_size,2);
    queue_front = 0;
    queue_back = 0;
    s = size(img);
    height = s(1);
    width = s(2);
    blob = zeros(s);
    hash = zeros(s);
    count = 0;
    
    queue_back = incr_queue(queue_back, max_size);
    
    q(queue_back,:) = [row,col];
    added = 0;
    while(queue_back~=queue_front)
        %pop off the queue
        queue_front = incr_queue(queue_front, max_size);
        pos = q(queue_front,:);
        %set the spot on the blob
        blob(pos(1),pos(2)) = 1;
        count = count + 1;
        %walk through above, below, left and right
        %and add them to the queue if they aren't already marked
        if(pos(1) + 1 <= height && (hash(pos(1)+1,pos(2))==0) && img(pos(1)+1,pos(2))==1)
            queue_back = incr_queue(queue_back, max_size);
            q(queue_back,:) = [pos(1)+1,pos(2)];
            hash(pos(1)+1,pos(2))=1;
            added = added +1;
        end
        if(pos(1) - 1 >=1 && (hash(pos(1)-1,pos(2))==0) && img(pos(1)-1,pos(2))==1)
            queue_back = incr_queue(queue_back, max_size);
            q(queue_back,:) = [pos(1)-1,pos(2)];
            hash(pos(1)-1,pos(2))=1;
            added = added +1;
        end
        if(pos(2) + 1 <= width && (hash(pos(1),pos(2)+1)==0) && img(pos(1),pos(2)+1)==1)
            queue_back = incr_queue(queue_back, max_size);
            q(queue_back,:) = [pos(1),pos(2)+1];
            hash(pos(1),pos(2)+1)=1;
            added = added +1;
        end
        if(pos(2) - 1 >= 1 && (hash(pos(1),pos(2)-1)==0) && img(pos(1),pos(2)-1)==1)
            queue_back = incr_queue(queue_back, max_size);
            q(queue_back,:) = [pos(1),pos(2)-1];
            hash(pos(1),pos(2)-1)=1;
            added = added +1;
        end
    end


end

function new_idx = incr_queue(idx,max)
    new_idx = mod(idx,max) +1;
end

function img = getImage(location)
% Read image and apply gaussian filter to remove noise
%
% Input:                                                  
% location - the location of the image to read
% 
% Ouput:
% img - the image post filter

    input = imread(location);
    img = smooth_image(input);
end


function masked = apply_mask(img, color)
% Filter out everything except stuff within range of our color
% TODO - why/how did them peeps use proportional masks??
%
% Input:
% inputImg - the original image
% 
% color - The target color that we're looking for
%
% Ouput:
% masked - the image mask
    
    
    red = double(color(1));
    green = double(color(2));
    blue = double(color(3));


    thresh = 50; % how much above or below desired color in either r g or b
    

    masked = img(:,:,1) < red + thresh & ...
             img(:,:,1) > red - thresh & ...
             img(:,:,2) < green + thresh & ...
             img(:,:,2) > green - thresh & ...
             img(:,:,3) < blue + thresh & ...
             img(:,:,3) > blue - thresh;
    
    s = size(img);
    height = s(1);
    width = s(2);
    ratioMasked = zeros(s(1),s(2));
    ratioThresh = .15;
    total = red + green + blue;
    ratioR = red / total;
    ratioG = green/total;
    ratioB = blue/total;
    %fprintf('%.3f %.3f  %.3f  %.3f', total, ratioR, ratioG, ratioB);
    for row = 1:height
        for col = 1:width
            localColor = double(img(row,col,:));
            localTotal = localColor(1) + localColor(2) + localColor(3);
            redPassed = abs(ratioR - (localColor(1) / localTotal)) < ratioThresh;
            greenPassed = abs(ratioG - (localColor(2) / localTotal)) < ratioThresh;
            bluePassed = abs(ratioB - (localColor(3) / localTotal)) < ratioThresh;
            if(redPassed && greenPassed && bluePassed)
               ratioMasked(row,col) = 1; 
            end
        end
    end
    
    masked = ratioMasked | masked;
         
    imshow(masked);

end

function smim = smooth_image(im)

    % 5 works for now
    sigma = 5;
    
    %assert(ndims(im) == 2, 'Image must be greyscale');
    
    % If needed convert im to double
    if ~strcmp(class(im(:,:,1)),'double')
        im(:,:,1) = double(im(:,:,1));
        im(:,:,2) = double(im(:,:,2));
        im(:,:,3) = double(im(:,:,3));
    end
    
    
    sze = ceil(6*sigma);  
    if ~mod(sze,2)    % Ensure filter size is odd
        sze = sze+1;
    end
    sze = max(sze,1); % and make sure it is at least 1
    
    h = fspecial('gaussian', [sze sze], sigma);

    smim = im; % start by copying to ensure size match

    
    smim(:,:,1) = filter2(h, im(:,:,1));
    smim(:,:,2) = filter2(h, im(:,:,2));
    smim(:,:,3) = filter2(h, im(:,:,3));
end





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


%% EDGE/FIND A DOOR CODE %%%%%%

function [edge_mask] = find_vertical_edges(img)


end

function [door_mask] = find_doors(img, edge_mask, color)


end

function [found, door] = find_largest_door(door_mask)



end

function [angle] = compute_angle_to_door(door)


end

function [angle] = angle_from_horizontal(horizontal)


end

function drive_to_and_face_door(serPort, angle)

% hard-coded center of hallway is 0.9m from wall

end

function correct_angle(serPort)
% call find_largest_door, compute angle and then turn that angle

end

function perform_door_knock(serPort)


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
