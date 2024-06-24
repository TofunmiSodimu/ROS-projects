1. 'Follow_me' - This is a ROS2 package implementing a 'follow me' robot. The package contains a 'follower' node and a 'followme_launch' launch file.
   - 'Follower.py' node: This node subscribes to the pose of turtle1, calculates the new angular and linear velocities to make the pose of turtle2 match that of turtle1, and
     publishes the new command velocities to turtle2.
   - 'followme_launch.py' launch file: This file launches a screen with two turtles and the 'follower' node.

  To use this package:
  - ** Source underlay in all terminals used by either adding to shell startup script or running in terminal **
  - Build in a new terminal - 'colcon build --packages-select follow_me'
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 launch follow_me followme_launch.py'.
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run turtlesim turtle_teleop_key'.

#### Follow Me Demo
![Screencast from 2024-06-07 12-35-02 (1)](https://github.com/TofunmiSodimu/ROS-projects/assets/35805326/388dd5b1-f367-4079-ade6-7291a491c3d6)

2. 'find_ball.py' - python code that uses opencv to detect a circular object in a camera frame and draws the circle around the object. This code is used in subsequent ROS2 packages.

3. '
