
function doit()

    img = imread('photo1.jpg');
    
    saturated = im2bw(rgb2gray(img), .9);

    mfImg = img;
    mfImg(:,:,1)=medfilt2(img(:,:,1));
    mfImg(:,:,2)=medfilt2(img(:,:,2));
    mfImg(:,:,3)=medfilt2(img(:,:,3));
    
    result = apply_mask(saturated, img) + apply_mask(1-saturated, mfImg)
    
    imshow(result);
    drawnow;

end

function done = apply_mask(mask, img)
    s = size(img);
    rows = s(1);
    cols = s(2);
    done = img;
    for row = 1:rows
        for col = 1:cols
            if(~mask(row,col))
                done(row,col,:) = 0;
            end
        end
    end


end