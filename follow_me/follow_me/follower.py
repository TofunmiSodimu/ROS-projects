# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3

class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        
        # Create subscriber to turtle1's pose
        self.subscription1 = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.turtle1_pose,
            10)
        self.subscription1

        #  Create subscriber to turtle2's pose
        self.subscription2 = self.create_subscription(
            Pose,
            'turtle2/pose',
            self.turtle2_pose,
            10)
        self.subscription2

        #  Create publisher to turtle2's command velocity
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)

        #  Create status variable
        self.status = None

        #  Create variables for storing turtle pose
        self.turtle1_pose_val = None
        self.turtle2_pose_val = None

        # Create variables for storing expected pose for turtle2
        self.new_angle = None
        self.dis = None

    # Callback function for subscriber1
    def turtle1_pose(self, msg):
        # Store turtle1's pose
        self.turtle1_pose_val = (msg.x,msg.y,msg.theta)

    # Callback function for subscriber2
    def turtle2_pose(self, msg):
        # Store turtle2's pose
        self.turtle2_pose_val = (msg.x,msg.y,msg.theta)
        
        # Call move_turtle2 function
        if self.turtle1_pose_val and self.turtle2_pose_val:
            self.move_turtle2()
    
    def move_turtle2(self):
        # Display turtle1's current pose
        #self.get_logger().info('Turtle 1\'s pose is : x = {},  y = {},  theta = {}'.format(self.turtle1_pose_val[0],self.turtle1_pose_val[1],self.turtle1_pose_val[2]))
            
        # Display turtle2's current pose
        #self.get_logger().info('Turtle 2\'s pose is : x = {},  y = {},  theta = {}'.format(self.turtle2_pose_val[0],self.turtle2_pose_val[1],self.turtle2_pose_val[2]))
        
        # Calculate new angle for turtle2
        self.new_angle = math.atan2((self.turtle1_pose_val[1]-self.turtle2_pose_val[1]),(self.turtle1_pose_val[0]-self.turtle2_pose_val[0]))

        # Calculate displacement between turtles
        self.dis = math.sqrt((self.turtle1_pose_val[1]-self.turtle2_pose_val[1])**2+(self.turtle1_pose_val[0]-self.turtle2_pose_val[0])**2)
        
        # Set velocities
        msg_ang = Vector3()
        msg_ang.x = 0.0
        msg_ang.y = 0.0
        msg_ang.z = 0.0
        msg_lin = Vector3()
        msg_lin.x = 0.0
        msg_lin.y = 0.0
        msg_lin.z = 0.0
          
        # Gradually, turn turtle2 in place - clockwise
        if (self.new_angle-self.turtle2_pose_val[2]) >= 0.1:

            # Publish new angular command vel for second robot
            msg_ang.z = 1.0
            msg = Twist()
            msg.angular = msg_ang
            msg.linear = msg_lin

            self.publisher.publish(msg)
            self.status = 'Turning Clockwise'

        # Gradually, turn turtle2 in place - counterclockwise
        elif (self.turtle2_pose_val[2] - self.new_angle) >= 0.1:

            # Publish new angular command vel for turtle2
            msg_ang.z = -1.0
            msg = Twist()
            msg.angular = msg_ang
            msg.linear = msg_lin

            self.publisher.publish(msg)    
            self.status = 'Turning Counterclockwise'

        # Move turtle2 forward
        elif self.dis >= 1.0:
            # Publish new linear vel for turtle2
            msg_lin.x = 1.0
            msg = Twist()
            msg.angular = msg_ang
            msg.linear = msg_lin

            self.publisher.publish(msg)
            self.status = 'Moving Forward'

        # Stop turtle2
        else:
            # Set all velocities to 0
            msg = Twist()
            msg.angular = msg_ang
            msg.linear = msg_lin

            self.publisher.publish(msg)
            self.status = 'Arrived'
        
        # Display status
        self.get_logger().info('Status: {}'.format(self.status))

def main(args=None):
    rclpy.init(args=args)

    follower = Follower()

    rclpy.spin(follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
