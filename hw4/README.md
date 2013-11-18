
1. compensate for distance by measuring in Davis
2. compensate turning by measuring in Davis
3. when bumping an obstacle, recalculate odometry and get back to shortest path

-------------RunRobot----------------

General System: 

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

If we bump an obstacle:

If at any point along our traversal we hit an obstacle, we do our best to estimate where
we are, and then try and regain our path. First, we find the closest vertice of any of the obstacles.
Most likely, we have hit near the corner of a box and because we don't know how far along the box's edge
we have gone, it's a fine estimate to say "we're at the vertex" for our purposes. We then compute
the point on our path that is closest to our new assumed point and head straight for it. If during this
attempt to regain the line we bump again, we assume we simply needed to turn more and add some extra turning (aka we hit the wall and our angle odometry was off). Once we regain the line, which in our above
system is the line between A and B, we head straight to B, turn to face C, and restart our old system.
The reason we cannot attempt to cut this corner is because it may have been the corner cutting itself that made us hit the obstacle so it's safer to B itself.

