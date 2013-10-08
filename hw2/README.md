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
- We print odometry to a plot with position in blue and orientation 
  as a green line, every time we take distance/angle readings
- Note that when the robot re-reaches the M-Line and turns to drive
  along it, we use a turn function that does not graph the updated
  angle while turning, it only graphs the new orientation after
  completing the turn

-----------------------
------ SIM NOTES ------
-----------------------

- None, everything should work as expected.

-----------------------
----- ROBOT NOTES -----
-----------------------

- We once again had difficult connection speeds with our robot
- We both have a PC and a mac laptop, but our PCs do not have
  functioning bluetooth and the mac connections are slow
- With incredibly an incredibly slow connection, our odometry was
  pretty far off when testing
- Luckily we were able to briefly test the robot on a windows machine
  in the lab and it looked like things were much better. So our numbers
  are designed for the lab and that fast windows connection.
