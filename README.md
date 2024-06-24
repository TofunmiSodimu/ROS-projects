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


3. 'object_follower_python' - This is a ROS2 package that may be used to rotate a turtlebot3 robot based on the movement of a circular object in its camera's field of view (FOV).
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


4. 'chase_object_python' - This is a ROS2 package that controls the turtlebot3 robot to 'chase' a circular object in its field of view, and maintain a desired distance using the camera and LIDAR sensors for object detection and distance measurements.
      - 'detect_object' node: This node subscribes to the camera topic on the turtlebot3 robot, determines the center coordinates of the circular object in its FOV, publishes the center coordinates as a geometry_msgs/Point message.
      - 'get_object_range' node: This node subscribes to the topic over which 'detect_object' is publishing center coordinates. It uses the object's coordinates in the FOV of the robot to determine the angle of the object in the FOV. This node then subscribes to the lidar topic on the turtlebot3 robot, and gets the depth corresponding to the pre-determined angle. Lastly, this node publishes the depth and the corresponding angle as a std_msgs/Float32MultiArray message.
      - 'chase_object' node: This node subscribes to the topic published by the 'get_object_range' node and uses the angle and depth values to implement a controller to make the robot chase the object while maintaining a desired distance.

To use this package:
  - ** Source underlay in all terminals used by either adding to shell startup script or running in terminal **
  - In one terminal, cd into ROS2 workspace where package is stored and build - 'colcon build --packages-select chase_object_python'
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run chase_object_python detect_object.py'.
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run chase_object_python get_object_range.py'.
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run chase_object_python chase_object.py'.

<p align="center"> 
   <h4 align="center">Chase Object Demo</h4>
</p>
<p align="center"> 
   <img src="https://github.com/TofunmiSodimu/ROS-projects/blob/main/extra/chase_object_resize.gif" alt="animated" />
</p>


5. 'navigate_to_goal' - This ROS2 package implements the 'wall-following' algorithm to avoid obstacles while moving from one waypoint to the other.
      - 'goToGoal' node: This nodes subscribes to the odometry and lidar topics of the turtlebot3 and uses it to determine distance from goal/waypoints and proximity to obstacles. It then makes use of a controller to move the robot to each waypoint/goal.

To use this package:
  - ** Source underlay in all terminals used by either adding to shell startup script or running in terminal **
  - In one terminal, cd into ROS2 workspace where package is stored and build - 'colcon build --packages-select navigate_to_goal'
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run navigate_to_goal goToGoal.py'.

<p align="center"> 
   <h4 align="center">Wall Following Demo</h4>
</p>
<p align="center"> 
   <img src="https://github.com/TofunmiSodimu/ROS-projects/blob/main/extra/wall_following_resize.gif" alt="animated" />
</p>


6. 'nav_stack' - This ROS2 package controls the movement of the turtlebot3 from one waypoint to the other, using a pre-determined map of the environment and the ROS2 navigation stack.
   - 'navigation' node: This node publishes waypoints to the goal_pose topic of the robot. It also subscribes to the feedback topic to print out updates on the task.

To use this package:
  - ** Source underlay in all terminals used by either adding to shell startup script or running in terminal **
  - In one terminal, cd into ROS2 workspace where package is stored and build - 'colcon build --packages-select nav_stack'
  - Source the overlay in another terminal - 'source install/setup.bash'. Run 'ros2 run nav_stack navigation.py'.


7. zslsl
<p align="center"> 
<h4 align="center">Maze Navigation</h4>
</p>
<p align="center"> 
   <img src="https://github.com/TofunmiSodimu/ROS-projects/blob/main/extra/lil_hero_6_optimize.gif" alt="animated" />
</p>

