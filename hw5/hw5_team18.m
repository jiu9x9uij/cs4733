% HW5 - Team 18
% Alden Quimby - adq2101
% Matthew Dean - mtd2121

%% MAIN METHOD %%%%%%%%%%%%%%%%%%%%%

function hw5_team18(serPort, part)

    % clear the cache
    clc;
    
    % poll sensors to avoid getting NaN values and clear odometry
    BumpsWheelDropsSensorsRoomba(serPort);
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort); 
    
    if part == 1
        color_tracker(serPort);
    elseif part == 2
        door_finder(serPort);
    elseif part == 3
        hall_follow(serPort);
    end

    disp('DONE!');
    
end

function [url] = camera_url()
% Get the URL of the IP camera.
%
% Output: 
% url - the url for reading images

    url = 'http://192.168.1.100/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0';

end


%% COLOR TRACKER %%%%%%%%%%%%%%%%%%%%%

function color_tracker(serPort)
% Track and follow a target object as it moves around, maintaining a constant distance.
% 
% Input:
% serPort - Serial port for communicating with robot

    url = camera_url();

    color = choose_color(url);

    [originalPixels, ~, ~] = analyze_image(url, color);

    while true
        
        [pixels, centroid, img] = analyze_image(url, color);

        size_change = pixels / originalPixels;
        center = size(img,2)/2;
        horizontal_change = (centroid(2) - center)/center;

        fprintf('SIZE: %.3f    HORIZONTAL: %.3f\n', size_change, horizontal_change);
        
        move_robot(serPort, size_change, horizontal_change);

    end

end

function [pixels, centroid, img] = analyze_image(url, color)
% Take an image and find the pixels and centroid of the
% largest blob matching the color in the image.
%
% Input:
% url - ip camera url
% color - color to track
%
% Output:
% pixels - pixels of the largest blob
% centroid - centroid of the largest blob
% img - the image taken

    % constants
    THRESH = 50;
    RATIO_THRESH = .08;

    img = imread(url);
    smooth = smooth_image(img);
    masked = apply_mask(smooth, color, THRESH, RATIO_THRESH);
    blob = get_largest_blob(masked);
    [pixels, centroid] = analyze_blob(blob);

end

function [color] = choose_color(url)
% Take an image and choose a color by clicking on it
%
% Input:
% url - ip camera url
%
% Output:
% color - the color chosen by the user

    img = imread(url);
    p = choose_point(img);
    smooth = smooth_image(img);
    color = smooth(p(2),p(1),:);
    color = [color(1), color(2), color(3)];

    disp('COLOR: ');
    disp(color);

end

function [pixels, centroid] = analyze_blob(blob)
% Find the pixels and centroid of a blob
%
% Input:
% blob - mask representing largest blob
%
% Output:
% pixels - pixels of the blob
% centroid - centroid of the blob

    pixels = 0;
    centroid = [0,0];
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

function [pos] = choose_point(img)
% Prompts user to choose a point on the image
%
% Input:
% img - the image to choose from
%
% Output:
% pos - position chosen by user

    f = figure();
    image(img);
    pos = round(ginput(1));
    close(f);

end

function [blob] = get_largest_blob(img)
% Find the largest blob in an image
%
% Input:
% img - the image to search
%
% Output:
% blob - the largest blob in the image

    biggestBlobCount = 0;
    s = size(img);
    height = s(1);
    width = s(2);
    blob = zeros(height, width);
    for row = 1:height
       for col = 1:width
           if( img(row,col)==1)
              [maybeBlob, count] = growBlob(img, row,col);
              img = img - maybeBlob;
              if (count > biggestBlobCount)
                  blob = maybeBlob;
                  biggestBlobCount = count;
              end
           end
       end
    end

end

function [blob, count] = growBlob(img, row, col)
% Identify a blob by growing up, down, left, right
%
% Input:
% img - the image to search
% row - starting point row
% col - starting point col
%
% Output:
% blob - the blob from the starting point
% count - number of pixels in the blob

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

    incr_queue = @(idx,max) mod(idx,max) + 1;
    
    queue_back = incr_queue(queue_back, max_size);
    
    q(queue_back,:) = [row,col];
    added = 0;
    while (queue_back~=queue_front)
        % pop off the queue
        queue_front = incr_queue(queue_front, max_size);
        pos = q(queue_front,:);
        % set the spot on the blob
        blob(pos(1),pos(2)) = 1;
        count = count + 1;
        % walk through above, below, left and right
        % and add them to the queue if they arent already marked
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

function [masked] = apply_mask(img, color, thresh, ratioThresh)
% Filter out everything except stuff within range of our color
%
% Input:
% img - the original image
% color - The target color that were looking for
% thresh - threshold for simple mask
% ratioThresh - threshold for ratio mask
%
% Ouput:
% masked - the image mask

    red = double(color(1));
    green = double(color(2));
    blue = double(color(3));

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

end

function smim = smooth_image(im)
    % Performs a gaussian filter, migrated from starter code given to us
    % to handle rgb values instead of just grayscale
    %
    % Input:
    % img - the original image
    %
    % Ouput:
    % smim - the smoothed out image

    % 5 works for now as our sigma for the filter
    sigma = 5;
    
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


%% MOVE ROBOT %%%%%%%%%%%%%%%%%%%%%

function move_robot(serPort, size_change, horizontal_change)
% Update robot based on new location of color marker.
%
% Input:
% size_change - Ratio of new_size_length to orig_side_length. So if orig
%   is 20x20 pixels, and new is 30x30, should be 1.5. Should be > 0.
% horizontal_change - Ratio of centroid_horizontal_movement to max centroid
%   horizontal movement, which is based on image size. Should be -1<x<1.

    % handle bump
    if check_for_bump(serPort)
        throw('OH NO I BUMPED!');
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
    MAX_FWD_VEL = 0.2; % max allowable forward velocity (m/s)
    MAX_ANG_VEL = 0.2; % max allowable angular velocity (rad/s)
    
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

    % if change btwn [-buff,buff], don't move
    size_buff = 0.05;
    
    if size_change > 1 + size_buff
        size_change = size_change - size_buff;
    elseif size_change < 1 - size_buff
        size_change = size_change + size_buff;
    end
    
    if abs(size_change - 1) < size_buff
        fwd_vel = 0;
    else

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
        
    end

       
    % if change btwn [-buff,buff], dont move
    horiz_buff = 0.1;
    
    if horizontal_change > horiz_buff
        horizontal_change = horizontal_change - horiz_buff;
    elseif horizontal_change < -horiz_buff
        horizontal_change = horizontal_change + horiz_buff;
    end
    
    if abs(horizontal_change) < horiz_buff
        ang_vel = 0;
    else
        ang_vel = horizontal_change * -MAX_ANG_VEL;
    end
    
end


%% DOOR FINDER %%%%%%

function door_finder(serPort)
    % Main loop of our solution to part 2 of the homework. Drives down the
    % hallway looking for a door, then drives to the door and knocks on it
    % beeps and drives through
    %
    % Input:
    % serPort - for robot access

    disp('door_finder');

    fwdSpeed = 0.1;
    found = 0;
    door = 0;
    
    url = camera_url();
    drive_straight(serPort, fwdSpeed);
    
    while ~found
        img = imread(url);
        [found, door] = find_door(img);
    end

    disp('stop_robot');
    stop_robot(serPort);
    
    disp('get_angle_from_door');
    angle = get_angle_from_door(door);
    
    disp('drive_to_door');
    drive_to_door(serPort, angle);
    
    disp('face_door');
    face_door(serPort, angle);
    
    disp('find_door');
    img = imread(url);
    [found, door] = find_door(img);
    
    if ~found
        disp('OH NO, LOST THE DOOR! Starting over and trying again.')
        face_door(serPort, -angle);
        door_finder(serPort);
        return;
    end
    
    disp('turn_angle');
    angle = get_angle_from_door(door);

    disp('turn_angle');
    % TIMES 2 BECAUSE IF IT'S LEFT HALF OF SCREEN, ANGLE ISN'T GREAT ENOUGH
    turn_angle(serPort, angle * 2 * pi/180);
    
    disp('perform_door_knock');
    perform_door_knock(serPort);
    
end

function [found, door] = find_door(img)
    % Attemps to find a single door by looking for all doors and picking
    % the biggest most viable candidate
    %
    % Input:
    % img - the original image
    %
    % Ouput:
    % found - whether we found any viable candidates
    % door - the mask containing the single door
    
    DOOR_COLOR = [100,106,130]; %NOT ACCURATE, GOOD FOR AFTER FILTERS

    door_mask = find_doors(img, DOOR_COLOR);
    
    [found, door] = find_largest_door(door_mask);

end

function [door_mask] = find_doors(img, color)
    % Performs a "door mask" which essentially highlights all areas of the
    % image that are the blue of the door. Because of all the complications
    % of similar colors in the hallway (like the teal overhang) we use some
    % color expanding algorithms to try and identify the blue doors as
    % distinct
    %
    % Input:
    % img - the original image
    % color - the color of the door we're looking for
    %
    % Ouput:
    % door_mask - the mask with all potential doors
    
    smooth = smooth_image(img);
    expanded = expand_colors(smooth);
    door_mask = apply_mask(expanded, color, 20, .05);

end

function [found, door] = find_largest_door(door_mask)
    % Finds the biggest blob in the door mask and makes sure its big enough
    % to count (5% of the screen at least) then returns it
    %
    % Input:
    % door_mask - the mask containing all the doors
    %
    % Ouput:
    % found - if any doors were found
    % door - the biggest door blob that's valid
    door = get_largest_blob(door_mask);
    thresh = .05; % PERCENT OF ENTIRE PIC
    
    s = size(door_mask);
    screen = s(1) * s(2);
    blob_size = sum(sum(door)); %THIS WORKS, TESTED IT
    
    found = blob_size > screen * thresh;
end

function [angle] = get_angle_from_door(door)
    % Given a door in an image, find the angle to that door
    %
    % Input:
    % door - the mask with the door blob
    %
    % Ouput:
    % angle - the angle from the robo to the door
    
    [~, centroid] = analyze_blob(door);
    center = size(door,2)/2;
    horizontal = (centroid(2) - center)/center;
    angle = angle_from_horizontal(horizontal);
end

function expanded = expand_colors(img)
    % Does some work to "expand" the colors making things like "teal" stick
    % out from blue. This is because similar colors provided too many false
    % positives for door detection. Performs a decorrelated stretch, linear
    % combination, and checks against over saturation.
    %
    % Input:
    % img - the original, smoothed image
    %
    % Ouput:
    % expanded - the new and improved image for processing
    
    newImg = decorrstretch(img);
    expanded = imlincomb(.7, img, .3, newImg);

    s = size(img);
    height = s(1);
    width = s(2);
    bad = im2bw(rgb2gray(expanded),.7);
    for row = 1:height
        for col = 1:width
            if(bad(row,col))
                expanded(row,col,:) = 0;
            end
        end
    end

end

function [angle] = angle_from_horizontal(horizontal)
    % Given a number between -1 and 1 representing the horizontal location
    % of something, what's the angle in real terms for the robot? uses
    % constants determined by our camera view angle
    %
    % Input:
    % horizontal, -1 <-> 1 representing how left or right on the image it
    % is
    %
    % Ouput:
    % angle - the angle from the robot to whatever spot we're talking about
    
    
    %constants to tweak:
    distance_to_plane = 1; % pick 1m
    width_of_plane = 0.85; % measured with camera
    
    half_angle = atand((width_of_plane/2)/distance_to_plane);
    angle = half_angle * -horizontal;
end

function perform_door_knock(serPort)
    % Performs a knock on the door by driving until bump, backing up, and
    % driving until bump, and then backing up again
    %
    % Input:
    % serPort - for robot access
    
    fwdSpeed = 0.2;
    time_back = 0.5;
    
    drive_to_bump(serPort, fwdSpeed);
    
    drive_time(serPort, -fwdSpeed, time_back);
    
    drive_to_bump(serPort, fwdSpeed);
    
    drive_time(serPort, -fwdSpeed, time_back);
    
    BeepRoomba(serPort);
    pause(1);
    drive_distance(serPort, fwdSpeed, .69); % lol
    
end

function drive_to_bump(serPort, fwdSpeed)
    % Drive until you hit the wall, for door knocking stuff
    %
    % Input:
    % serPort - for robot access
    % fwdSpeed - how fast to move

    drive_straight(serPort, fwdSpeed);
    
    bumped = check_for_bump(serPort);
    while ~bumped
        bumped = check_for_bump(serPort);
    end
    
    stop_robot(serPort);
    
end

function drive_to_door(serPort, angle)
    % Given the angle to the door, drive down the hallway until you're next
    % to the door, uses constants like the hallway width to determine this
    %
    % Input:
    % serPort - for robot access
    % angle - angle to the desired door

    dist_to_wall = 0.9;
    fwdSpeed = 0.2;
    
    dist_to_drive = dist_to_wall/tand(abs(angle));
    disp('DIST TO DRIVE');
    disp(dist_to_drive);
    
    drive_distance(serPort, fwdSpeed, dist_to_drive);
    
end

function face_door(serPort, angle)
    % Turns to face the door, either 90 or -90
    %
    % Input:
    % serPort - for robot access
    % angle - the original angle to the door, so we know which way to turn

    if angle > 0
        turn_angle(serPort, pi/2);
    else
        turn_angle(serPort, -pi/2);
    end
    
end


%% HALLWAY FOLLOW %%%%%%%%%%%%%%%%%%%%%%

function hall_follow(serPort)
    % Simple loop that follows the bright lights of the hallway
    %
    % Input:
    % serPort - for robot access

    url = camera_url();
    while(true)
        img = imread(url);
        horizontal = find_hallway_center(img);
        move_robot(serPort, .5, horizontal); % .5 means max fwd speed
    end
end

function horizontal = find_hallway_center(img)
    % Walks the image, 10 pixel columns at a time and finds which column is
    % the brightest. It offsets by 5 pixels so checks the image very
    % accurately. Used to identify the bright lights of the center of the
    % hallway.
    %
    % Input:
    % img - what the robot is seeing right now
    %
    % Output:
    % horizontal - the -1 <-> 1 value that represents the brightest column
    % aka probably the hallway lights

    total1s = 0;
    s = size(img);
    height = s(1);
    width = s(2);
    bestBrightness = -1;
    currentBestCol = -1;
    currentTotal = 0;
    currentPixels = 0;
    currentStartCol = 1;
    max = currentStartCol + 10;
    bright = im2bw(rgb2gray(img),.95);
    while(max <=width)
        for col = currentStartCol:max
            for row = 1:height
                currentTotal = currentTotal + bright(row,col);
                currentPixels = currentPixels + 1;
                total1s = total1s + bright(row,col);
            end
        end
        average = currentTotal / currentPixels;
        if(average > bestBrightness)
           bestBrightness = average;
           currentBestCol = max-5;
        end
        currentTotal = 0;
        currentPixels = 0;
        currentStartCol = currentStartCol + 5;
        max = currentStartCol + 10;
    end
    center = size(img,2)/2;
    horizontal = (currentBestCol - center)/center;
end


%% ROBOT UTILITIES %%%%%%%%%%%%%%%%%%%%%

function drive_distance(serPort, fwdSpeed, dist)
    % Drive straight for some distance

    DistanceSensorRoomba(serPort);

    totalDist = 0;
    
    drive_straight(serPort, fwdSpeed);
        
    while (totalDist < dist)        
        totalDist = totalDist + DistanceSensorRoomba(serPort);
    end
    
    stop_robot(serPort);

end

function drive_time(serPort, fwdSpeed, time)
    % drive straight for some period of time
    
    t = tic;
    drive_straight(serPort, fwdSpeed);
    while (toc(t) < time)
    end
    
    stop_robot(serPort);
    
end

function drive_straight(serPort, fwdSpeed)
    % drive straight (can have ang compensation for fast speeds but found
    % that this hw didn't require it
    
    
    angCompensate = 0.0;

    SetFwdVelAngVelCreate(serPort, fwdSpeed, angCompensate*fwdSpeed);

end

function stop_robot(serPort)
    % simply stops the robo

    SetFwdVelAngVelCreate(serPort, 0, 0);
    
end

function [angTurned] = turn_angle(serPort, angToTurn)
% Perform an acurate in place turn.
%
% Input:
% serPort - Serial port for communicating with robot
% angToTurn - Angle to turn, positive is counter-clockwise (rad)
%
% Output:
% angTurned - Actual angle turned (rad)

    % compensate for real robot
    angToTurn = 0.9*angToTurn;

    % constants
    turnSpeed = 0.35; % turn angle speed (rad/s)

    % if turning clockwise, use negative speed and positive angle
    if (angToTurn < 0)
        turnSpeed = -turnSpeed;
        angToTurn = -angToTurn;
    end
    
    % if turning more than half way, turn the other way
    if (angToTurn > pi)
        angToTurn = 2*pi - angToTurn;
        turnSpeed = -turnSpeed;
    end
    
    % reset angle sensor
    angTurned = AngleSensorRoomba(serPort);
    
    % start turning
    SetFwdVelAngVelCreate(serPort, 0, turnSpeed);
    
    % loop until turn complete
    while (abs(angTurned) < angToTurn)
        pause(0.01);
        angTurned = angTurned + AngleSensorRoomba(serPort);
    end
    
    % stop turning
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    % pause in case robot still turning a little
    pause(0.2);
    
    % reset angle sensor and update angle
    angTurned = angTurned + AngleSensorRoomba(serPort);
    
end

function [bumped] = check_for_bump(serPort)
% simply reads the bump sensors
%
% Input:
% serPort - for robot access
%
% Output: 
% whether or not any of the bump sensors are true
    
    [BumpRight,BumpLeft,~,~,~,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

    % handle possible NaN
    while (isnan(BumpRight) || isnan(BumpLeft) || isnan(BumpFront))
        [BumpRight, BumpLeft , ~, ~, ~, BumpFront] ...
            = BumpsWheelDropsSensorsRoomba(serPort);
    end

    bumped = BumpRight || BumpLeft || BumpFront;
end

