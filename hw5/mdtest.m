

function mdtest()
    camera_ip = '192.168.1.100';
    url = strcat('http://', camera_ip, '/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0');
    
    
    img = imread(url);
    smoothed = smooth_image(img);
    masked = find_hallway_center(smoothed);
    
    figure(1);
    imshow(img);
    figure(2);
    imshow(masked);
    drawnow;
end

function masked = find_hallway_center(img)
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
    masked = zeros(s(1),s(2));
    masked(:,currentBestCol) = 1;
    disp('TOTAL 1S');
    disp(total1s);
end



function expanded = expand_colors(img)
%{
hsv_image = rgb2hsv(img);
hsv_image(:,:,2) = 1;
expanded = hsv2rgb(hsv_image);

expanded = img;
expanded(:,:,1) = histeq(img(:,:,1));
expanded(:,:,2) = histeq(img(:,:,2));
expanded(:,:,3) = histeq(expanded(:,:,3));
%}
newImg = decorrstretch(img);

expanded = imadjust(img,stretchlim(img),[]);
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


    thresh = 20; % how much above or below desired color in either r g or b
    

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
    ratioThresh = 0.05;
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

    % 5 works for now
    sigma = 4;
    
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


function blob = get_largest_blob(img)
    biggestBlobCount = 0;
    s = size(img);
    height = s(1);
    width = s(2);
    blob = zeros(height, width);
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
