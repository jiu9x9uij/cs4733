Alden Quimby - adq2101
Matthew Dean - mtd2121
CS4733 HW2

-----------------------
---- GENERAL NOTES ----
-----------------------

- Basic algorithm: 
    1. turn to face goal point
    2. drive on M-Line until goal reached or obstacle hit
    3. if obstacle, follow wall until M-Line reached at closer point,
       or qHit reached (failed), or goal reached (succeeded)
- We used the WallFollow function from the hw1 solution
- We have a distance cushion that grows over time to determine
  if the robot is at a certain point (like qHit or goal) or to
  determine if the robot is back on the M-Line
- We print odometry to a new plot with position in blue and
  orientation as a green line

-----------------------
------ SIM NOTES ------
-----------------------

- None, everything should work as expected.

-----------------------
----- ROBOT NOTES -----
-----------------------

??
