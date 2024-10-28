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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan  # Example sensor message type for detecting obstacles
from geometry_msgs.msg import Twist, Pose  # For robot motion control
from nav_msgs.msg import Odometry  # For robot location
from std_msgs.msg import String
import numpy as np
import math

class Bug1():
    
    
    Z_CALCMGV       = 0
    Z_ROTATE        = 1
    Z_FOLLOW_MGV    = 2
    Z_WALLFOLLOWING = 3
    Z_END           = 4
    
    ThreshWF        = 0.6 + 1 # 1 wegen radius des robo
    
    def __init__(self,start,goal):
        
        
        # Move To Goal Vektor
        self.posRob  = np.array(start)
        self.presentAng = 0.0 
        self.posGoal = np.array(goal)

        self.mgv = 0.0 # MoveToGoalVektor
        self.mgvAngle = 0.0 # MoveToGoalVektor-Angle
        self.mgvMag = 0.0 # MoveToGoalVektor-Magnitude

        # Wall following variablen
        self.minDistance = float('inf')
        self.posMin = np.array([0.0, 0.0])
        self.posStartWF = np.array([0.0, 0.0])
        self.distanceToWall = 0.0
        self.circumnavigate = False
        
        self.state = self.Z_CALCMGV

    def CalcDistanceToGoal(self):
        return np.linalg.norm(self.posGoal - self.posRob)

    def CalcMGV(self):
        self.mgv = self.posGoal - self.posRob
        self.mgvAngle = np.arctan2(self.mgv[1], self.mgv[0]) # arctan(y/x) 
        self.mgvMag = np.linalg.norm(self.mgv)

    def SearchBestPoint(self):
        if self.CalcDistanceToGoal() < self.minDistance:
            self.posMin = self.posRob

    def SetLaser(self, ranges):
        self.distanceToWall = ranges

    def SetRoboPos(self, posRobo):
        self.posRob         = np.array([posRobo.x, posRobo.y])
    
    def SetRoboAngle(self, orientation):
        self.presentAng     = self.CalcAngularZ(orientation.x, orientation.y, orientation.z, orientation.w)
    
    def CheckForWall(self):
        return min(self.distanceToWall) < self.ThreshWF 
    
    def GetLinearX(self):
        return self.ctrlLinearX
    
    def GetAngularZ(self):
        return self.ctrlAngularZ
    
    def CalcAngularZ(self,x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        angularz = math.atan2(t3, t4)

        return angularz # in radians
    
    def CheckRightSide(self):
        return round(((self.distanceToWall[0] + self.distanceToWall[3]) / 2) ,1)
    
    def Run(self):

        
        ctrlLinearXTemp = 0
        ctrlAngularZTemp = self.presentAng
        
        if self.state == self.Z_CALCMGV:

            ctrlLinearXTemp = 0.0
            ctrlAngularZTemp = self.presentAng
            self.CalcMGV()
            self.state = self.Z_ROTATE
            print('CalcMGV')
            
            
        elif self.state == self.Z_ROTATE:
            
            ctrlLinearXTemp = 0.0
            
            if abs(self.mgvAngle - self.presentAng) < 0.1:
                self.state = self.Z_FOLLOW_MGV
            else:
                if self.mgvAngle > 0.0 and self.mgvAngle < np.pi:
                    ctrlAngularZTemp = 0.5
                else:
                    ctrlAngularZTemp = -0.5
                    
            print('Rotate')
            print(f'mvgAngle: {self.mgvAngle}')
            print(f'presentAngle: {self.presentAng}')
        
        elif self.state == self.Z_FOLLOW_MGV:
            
            ctrlAngularZTemp = 0.0

            if self.CheckForWall():

                ctrlLinearXTemp= 0.0
                self.posStartWF = self.posRob
                self.state = self.Z_WALLFOLLOWING

            elif (self.posRob[0] == self.posGoal[0]) and (self.posRob[1] == self.posGoal[1]):

                ctrlLinearXTemp = 0.0
                self.state = self.Z_END
                
            else:
                ctrlLinearXTemp= 1.0
                 
            print('Follow')
            
        elif self.state == self.Z_WALLFOLLOWING:

            if round(self.distanceToWall[6],1) > self.ThreshWF:
                ctrlAngularZTemp = -0.3
                ctrlLinearXTemp = 0.1
            elif round(self.distanceToWall[6],1) < self.ThreshWF:
                ctrlAngularZTemp = 0.3
                ctrlLinearXTemp = 0.1
            else:
                ctrlLinearXTemp = 1.0
                ctrlAngularZTemp = 0.0

            if not self.circumnavigate:
                self.SearchBestPoint()
                if (self.posRob[1] == self.posStartWF[1]) and (self.posRob[0] == self.posStartWF[0]):
                    self.circumnavigate = True
            else:
                if (self.posRob[1] == self.posMin[1]) and (self.posRob[0] == self.posMin[0]):
                    self.state = self.Z_CALCMGV
                    self.circumnavigate = False
        
            print('Wallfollowing')
            #print(f'distance: {self.CheckRightSide()}')
        
        elif self.state == self.Z_END:
            print('End')
            
        self.ctrlLinearX = ctrlLinearXTemp
        self.ctrlAngularZ = ctrlAngularZTemp

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.bug1Handler = Bug1(np.array([0.0,0.0]) , np.array([1.0,1.0]) )
        
        
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        

        # Create publisher to control the robot's velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive/cmd_vel', 20)

        # Subscribe to a topic that provides distance to obstacles (e.g., LIDAR or laser scanner)
        self.create_subscription(LaserScan, '/diff_drive/scan', self.scan_callback, 20)

        # Subscribe to current robot state
        self.create_subscription(Odometry, '/diff_drive/odometry', self.location_callback, 20)


    def timer_callback(self):
        
        self.bug1Handler.Run()
        velocity_cmd = Twist()
        velocity_cmd.linear.x = self.bug1Handler.GetLinearX()
        velocity_cmd.angular.z = self.bug1Handler.GetAngularZ()
        #self.cmd_vel_pub.publish(velocity_cmd)
        
        
        #self.get_logger().info('linear x: "%s"' % velocity_cmd.linear.x)
        #self.get_logger().info('angular z: "%s"' % velocity_cmd.angular.z)

        
    def scan_callback(self, msg):        
        self.bug1Handler.SetLaser(msg.ranges)
        self.get_logger().info('Scan Ranges: "%s"' % msg.ranges)
        
    def location_callback(self, msg):
        
        self.bug1Handler.SetRoboPos(msg.pose.pose.position)
        self.bug1Handler.SetRoboAngle(msg.pose.pose.orientation)
        #self.get_logger().info('location x: "%s"' % msg.pose.pose.position.x)
        #self.get_logger().info('location y: "%s"' % msg.pose.pose.position.y)
        #self.get_logger().info('angular z: "%s"' % msg.pose.pose.orientation)
        

        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()