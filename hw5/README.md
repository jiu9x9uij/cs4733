Alden Quimby - adq2101
Matthew Dean - mtd2121
CS4733 HW5

---------------------------
---- RUNNING OUR CODE -----
---------------------------

PART 1 (COLOR TRACKING):
"hw5_team18(serPort, 1)"

PART 2 (DOOR FINDING):
"hw5_team18(serPort, 2)"

EXTRA CREDIT (HALL FOLLOW):
"hw5_team18(serPort, 3)"

-----------------------
---- PART I NOTES -----
-----------------------

For part one we used a color mask to determine which areas of our image matched the desired color. The mask itself used a combination of directly comparing the desired rgb value with the image using a threshold, as well as comparing the ratios of the desired rgb to the image. This ratio mask was particularly helpful for shadows and changing lighting. For example it would take the red ratio ->  r/ (r + g + b) and then compare that vs each pixel of our current image. A shadow over our marker could still have the same ratios because the whole image is darker so the denominator gets smaller, and the numerator gets smaller.

To move the robot, we simply added angular velocity based on the horizontal location of the centroid of the marker, and added fwd and back velocity based on how large the marker was in the image compared to the original. This allows a smooth tracking path, even around corners. 

-----------------------
---- PART II NOTES ----
-----------------------

For part two we found that some of the colors in the hallway would confuse our color mask from part 1. In order to improve the performance of our mask, we implemented some image processing before the mask that helped to separate colors from each other. We used decorrelated stretching, and combined it with removing some of the over saturated parts of the image (like the lights). This allowed our mask to excel in finding the doors based on color. Once we located the blobs of the image that were valid door areas, we picked the biggest one and used simple math to determine how far away we were given the width of the hall, and the angle to the desired door area. We then head straight forward for a computed distance, turn 90 to face the door (left or right), and check for the door again in case we over or undershot. After turning to face the door, we knock twice, beep, and head through the doorway after a short pause.

Please note that our algorithm assumes our robot is centered in the hallway, and that it is facing straight down the hallway. We talked to Alex (TA Alex) about this, and he said it was fine. If we were not facing straight down the hallway, but were at an arbitrary orientation, we could adapt our extra credit work to face the correct direction. Read more about it below, but the basic idea would be to do a 360 and determine the orientation with the highest brightness column (which would always be the ceiling lights because of our min-saturation mask), then turn to that orientation. We did not implement this because Alex said we could assume we're in the middle of the hallway, facing down it.

----------------------------
---- EXTRA CREDIT NOTES ----
----------------------------

We succeeded in driving straight down the middle of any hallway for any distance. However we did not succeed in rounding the corners. Given the varied nature of the corners of the hallway (for example, some corners end with brightly lit windows, while others are bare walls) we did not succeed in handling turning the corner (although we did try). We were succesful however in following the hallway itself using techniques in Yinxiao's paper to find the center lights of the hallway. We've written a simple loop that follows the hallway. If you start the robot against a wall with some part of the hallway in view, it will head towards the end of the hall succesfully. 
