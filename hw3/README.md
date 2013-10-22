Alden Quimby - adq2101
Matthew Dean - mtd2121
CS4733 HW3

- !!!! Notes on running algorithm
    - In order to make the algorithm run most efficiently, tweak the gridsize
      variable to most closely match the space to be explored. Currently the
      gridsize is made for the size of the default SimulatorGUI screen (10m)
      An issue arises if the gridsize is much larger than the actual space
      enclosing the robot since it will try and explore the unreachable area
      and continually lap the room trying to get there. This is because we
      choose our points from the grid itself. 
      The gridsize referes to the number
      of robot diameters that make up the size of the square. The robot is of
      diameter .335 meters. 

- Goal
  - Cover environment with spiraling, random driving, wall following

- Basic algorithm
  - Start by spiraling until an obstacle is hit
  - Circumnavigate obstacle
  - Pick random, unexplored point and try to drive to it
  - If new obstacle encountered along the way, circumnavigate it
    and forget about goal point
  - If old obstacle encountered along the way, use bug2 M-line algorithm
    to go around obstacle and continue to goal point
  - If goal point reached, begin spiraling
  - Finish if whole occupancy grid filled, or grid hasn't changed in 3mins

- State machine for basic algorithm
    State 1 - spiral outward until hitting something
    State 2 - wall follow, inside hitpoint threshold
    State 3 - wall follow, left hitpoint threshold
    State 4 - stop, pick random empty square, turn to it
    State 5 - drive straight to point picked in 4
    State 6 - M-Line wall follow, inside hitpoint threshold
    State 7 - M-Line wall follow, left hitpoint threshold

- Implementation
    - We implemented the state machine described above
    - We used the WallFollow function from the hw1 solution
    - We made an occupancy grid roughly 10m wide, 10m high, and assume
      the robot starts at the center of this grid
    - If any part of the robot is ever outside this grid, we stop,
      pick a random unexplored point, and drive to it
    - When driving along, we mark grid spaces corresponding to four x,y
      points (front, left, right, back of robot) as empty
    - In general this updates 1 grid space, but it's possible that 4 
      spaces update at once if the robot is on top of 4 grid spaces
    - When wall following we update the grid space located to the direct
      right of the robot (where it is near the wall) as being filled
    - If a space was previously marked empty, it can later be marked 
      filled, but not the other way around
    - When a wall is encountered, we count it as previously encountered
      if the current robot grid space or the grid space directly in
      front of the robot is marked as filled
    - If we are trying to drive to a random point and encounter a known
      obstacle, and then circumnavigate that obstacle, it means the random
      point is unreachable (based on bug2 algorithm)
        - We mark the point filled so that we don't randomly pick it again

- Plotting
    - We print current position in blue and orientation as a green 
      line every time we update odometry
    - We print the current grid to a separate plot with gray for unknown 
      spaces, orange for empty spaces, and white for filled spaces

