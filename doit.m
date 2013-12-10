
function blend_image()

    img = imread('http://192.168.1.100/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0');
    
    saturated = im2bw(rgb2gray(img), .8);

    mfImg = img;
    mfImg(:,:,1)=medfilt2(img(:,:,1), [100,100]);
    mfImg(:,:,2)=medfilt2(img(:,:,2), [100,100]);
    mfImg(:,:,3)=medfilt2(img(:,:,3), [100,100]);
    result = mask_image(1-saturated, img) + mask_image(saturated, mfImg);
    
    imshow(result);
    drawnow;

end

function done = mask_image(mask, img)
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