

function imagetests


starterImage = getImage('photo1.jpg');
%TODO does it matter if user sees raw image? Probably...
%so we need to use position, then use the color after gaussian
color = ChoosePoint(starterImage);
masked = apply_mask(starterImage, color);
get_largest_blob(masked);
%{
pause(2)
img2 = getImage('photo2.jpg');
apply_mask(img2, color);

pause(2)
img3 = getImage('photo3.jpg');
apply_mask(img3, color);

pause(2)
img4 = getImage('photo4.jpg');
apply_mask(img4, color);
%}

end

function blob = get_largest_blob(img)
disp('get largest');
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
disp('grow!');
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
    disp('begin loop!');
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
    img = smooth_image(input,5); % this is sigma for gaussian
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
    
    
    red = color(1);
    green = color(2);
    blue = color(3);

    thresh = 50; % how much above or below desired color in either r g or b


    masked = img(:,:,1) < red + thresh & ...
             img(:,:,1) > red - thresh & ...
             img(:,:,2) < green + thresh & ...
             img(:,:,2) > green - thresh & ...
             img(:,:,3) < blue + thresh & ...
             img(:,:,3) > blue - thresh;
         
    %imshow(masked);

end

%from sample code but modified for colors vs grayscale
function smim = smooth_image(im, sigma)
 
    
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



%STRAIGHT UP TAKEN JUST FOR TESTING!!!!%%%

function [ color ] = ChoosePoint( img )
%ChosePoint( image )
%   Takes an image, prompt user to choose point, return color of point
    f = figure();
    image(img);
    
    % gets an input from the figure
    % if the user exits the figure instead, try again
    p = round(ginput(1));
    %disp(p);
    
    % reallocate the color, flipping the x/y coordinates
    
    color = img(p(2),p(1),:);
    color = [color(1), color(2), color(3)];
    %disp(color);
    

    close(f);

end
