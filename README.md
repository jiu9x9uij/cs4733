Matt Dean mtd2121
Alden Quimby adq2101
CS 4733 HW #1


Intro:

	For this assigment, we took extra care to try and optimize for the difference in the environments between the simulator and the real robot. We found that the simulator allowed us to utilize faster speeds and make use of the wall sensor while maintaning accuracy while the real robot required us to fall back to simpler methods.

The simulator:

	We didn't make use of any of the built in functions of traveldist or turnangle because we found it was easier and more accurate to simply turn the robot manually while checking the angle as we went. Additionally, the wall sensor was very helpful in allowing us to ride the edge of a wall without having to continually bump into it. Overall we found our program to consistently succeed in simulator maps with very little error.

The real robot:
	It was difficult for us to connect to the robot at a rate of more than 1 set of commands per second. This meant that our robot was continually running into an object, and not realizing it had done this, while it slid around, losing angle accuracy. To try and combat this, we fell back to discrete motions. Essentially we tell the robot to move 1/10th of a meter, then check the bump sensors, then turn a certain angle, etc etc. This is very different from our simulator program which runs a rapid while loop, continually checking the sensors. We still found the real robot to have big accuracy issues, mainly because the built in functions of "turnangle" and "traveldist" were inaccurate. Depending on the speed at which we told the robot to drive for traveldist, it would move very different distances. To deal with this discrepancy, we added multipliers so that practically, the robot would move roughly correct distances and angles given our speeds (even though we would tell the robot to move something different). As a whole, we had to keep the robot moving very slowly to get any accuracy so traversing a small table was very slow. 
