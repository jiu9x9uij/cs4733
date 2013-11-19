Alden Quimby - adq2101
Matthew Dean - mtd2121
CS4733 HW4

---------------------------
---- RUNNING OUR CODE -----
---------------------------

PLEASE RUN "hw4_team18(0, 'obstacles.txt', 'start_goal.txt')"
- the file names are the obstacle and start/goal files that you want to use
- the 0 means there is no serial port (no real robot)
- if you pass a serial port as the first arg, the robot will execute the path

-----------------------
---- PART I NOTES -----
-----------------------

Using Matlab, we implemented everything as described and tested on many different
obstacle sets. We grow the obstacles using the reflection algorithm and
convex hull algorithm. Please note that we choose the RIGHTMOST point on
the robot here, so in our output, the obstacles will be grown in the positive
x direction mostly. Also, we represented the robot as an octogon, so the grown
obstacles have many edges. You can change the first line in our program to make
our computation faster if you'd like: "robot_num_verts = 4;". 
Then we create a visibility graph, and search it using Dijkstra
for the shortest path. There are many edge cases in the V-graph that we handle.
For example, we handle two grown obstacles overlapping and a grown obstacle 
overlapping with the wall.

Our program outputs the following:
- wall               = dark blue object
- start point        = red point
- goal point         = blue point
- obstacle set       = light blue objects
- grown obstacle set = black objects
- Visibility Graph   = green edges
- safe path          = red path

-----------------------
---- PART II NOTES ----
-----------------------

In an attempt to speed up our traversal of the course we chose to implement
a system that improves upon the classic "drive to point, turn to face next point, repeat".
Instead, we take the points three at a time. Let's look at an example with points
A, B and C. The robot will currently be at point A, and have to travel to B then C.
Given a certain turn radius of the robot, we can calculate a point on the line
between A and B where it would be optimal to begin turning, and also calculate the point
on the line between B and C where we would regain contact. It "cuts the corners" of the course
so there is some risk involved, but the advantage is that the robot never stops
moving and there is a large increase in overall speed. In order to improve accuracy
we recalculate the correct places to leave the line and how long to turn based on the current
orientation of the robot. We even calculate a B' given that we are heading towards B but may
be slightly off course. 


----------------------------
---- EXTRA CREDIT NOTES ----
----------------------------

We accounted for poor odoemtry using three techniques. 

1. TURNING
When turning, we tell the robot to stop turning 5 degrees before it thinks
it should. Because we do not want the robot to stop mid race, there is a 5
degree lag where it continues to turn after it should really be going straight.
We measured this in Davis with our robot and found this was the best offest to use.

2. DRIVING STRAIGHT
When driving straight, we give the robot an angular speed of 0.02713 rad/sec.
We also measured this in Davis, and it is needed for our robot to actually go straight.
Additionally, we tell our robot to stop 0.1m before it thinks it should.
This is similar to the angle situation, the robot continues for a small time
because of bluetooth delay.

3. OBSTACLES
If we bump into an obstacle, we recalculate odometry and get back to shortest path.
In more detail, if at any point along our traversal we hit an obstacle, we do our best to estimate where
we are, and then try and regain our path. First, we find the closest vertice of any of the obstacles.
Most likely, we have hit near the corner of a box and because we don't know how far along the box's edge
we have gone, it's a fine estimate to say "we're at the vertex" for our purposes. We then compute
the point on our path that is closest to our new assumed point and head straight for it. If during this
attempt to regain the line we bump again, we assume we simply needed to turn more and add some extra turning 
(aka we hit the wall and our angle odometry was off). Once we regain the line, which in our above
system is the line between A and B, we head straight to B, turn to face C, and restart our old system.
The reason we cannot attempt to cut this corner is because it may have been the corner cutting 
itself that made us hit the obstacle so it's safer to B itself.

