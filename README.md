1. 'Follow_me' - This is a ROS2 package implementing a 'follow me' robot. The package contains a 'follower' node and a 'followme_launch' launch file.
   - 'Follower.py' node: This node subscribes to the pose of turtle1, calculates the new angular and linear velocities to make the pose of turtle2 match that of turtle1, and publishes the new command velocities to turtle2.
   - 'followme_launch.py' launch file: This file launches a screen with two turtles and the 'follower' node.

  To use this package:
  - ** Source underlay in all terminals used by either adding to shell startup script or running in terminal **
  - In one terminal, cd into ROS2 workspace where package is stored and build - 'colcon build --packages-select follow_me'
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 launch follow_me followme_launch.py'.
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run turtlesim turtle_teleop_key'.

<p align="center"> 
   <h4 align="center">Follow Me Demo</h4>
</p>
<p align="center"> 
   <img src="https://github.com/TofunmiSodimu/ROS-projects/blob/main/extra/Follow_me.gif" alt="animated" />
</p>

2. 'find_ball.py' - python code that uses opencv to detect a circular object in a camera frame and draws the circle around the object. This code is used in subsequent ROS2 packages.

  To use this file:
   - Run from within IDE e.g., VS Code, OR
   - Make file an executable and run from terminal using ./find_ball.py


3. 'object_follower_python' - This is a ROS2 package that may be used to rotate a turtlebot3 robot based on the movement of a circular object in its field of view (FOV).
   - 'find_object' node: This node subscribes to the camera topic on the turtlebot3 robot, determines the center coordinates of the circular object in its FOV, publishes the center coordinates as a geometry_msgs/Point message.
   - 'rotate_robot' node: This node subscribes to the topic over which the 'find_object' node publishes center coordinates and rotates the turtlebot3 robot such that it's always facing the circular object.

To use this package:
  - ** Source underlay in all terminals used by either adding to shell startup script or running in terminal **
  - In one terminal, cd into ROS2 workspace where package is stored and build - 'colcon build --packages-select object_follower_python'
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run object_follower_python find_object.py'.
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run object_follower_python rotate_robot.py'.
    
<p align="center"> 
   <h4 align="center">Object Follower Demo</h4>
</p>
<p align="center"> 
   <img src="https://github.com/TofunmiSodimu/ROS-projects/blob/main/extra/Object_follower_resized.gif" alt="animated" />
</p>


4. dd;;d
5. akka
