

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

        self.circumnavigate = False
        self.laserRechtsHinten = 0
        self.laserRechtsRechts = 0
        self.laserRechtsVorne = 0
        self.laserVorne = 0
        
        self.integral = 0
        
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

    def SetLaser(self, msg):
        self.laserRechtsRechts= self.get_sector_min_distance(msg, 269,271)
        self.laserVorne = self.get_sector_min_distance(msg, 30 ,330)
        self.laserRechtsVorne = round(self.get_sector_min_distance(msg, 290,340), 1)
        self.laserRechtsHinten = round(self.get_sector_min_distance(msg, 200,250), 1)

    def SetRoboPos(self, posRobo):
        self.posRob         = np.array([posRobo.x, posRobo.y])
    
    def SetRoboAngle(self, orientation):
        self.presentAng     = self.CalcAngularZ(orientation.x, orientation.y, orientation.z, orientation.w)
    
    def CheckForWall(self):
        print(self.laserVorne)
        return self.laserVorne < self.ThreshWF 
    
    def GetLinearX(self):
        return self.ctrlLinearX
    
    def GetAngularZ(self):
        return self.ctrlAngularZ
    
    def CalcAngularZ(self,x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        angularz = math.atan2(t3, t4)

        return angularz # in radians
    
    def get_sector_min_distance(self, laser_msg, angle_min, angle_max):
        
        ranges = laser_msg.ranges
        angle_min = angle_min / 360. * 2. * np.pi
        angle_max = angle_max / 360. * 2. * np.pi

        index_min = int((angle_min - laser_msg.angle_min)/ laser_msg.angle_increment)
        index_max = int((angle_max - laser_msg.angle_min) / laser_msg.angle_increment)

        range_min = np.min(ranges[index_min:index_max])
        
        return range_min

    def CalcAngularSpeed(self, right_distance, desired_distance, kp, ki, direction):
        
        error = right_distance - desired_distance
        error = abs(error)   
            
        proportional = kp * error
        self.integral += error
        integral = ki * self.integral
        angular_speed = proportional + integral * direction
        
        self.last_error = error
        
        return angular_speed
    
    def WallFollowingAlg(self, desiredDistance, kp, ki):
        

        right_distance = self.laserRechtsRechts
        if right_distance == np.inf:
            right_distance = 4.0
        
        
        print(self.laserRechtsVorne)
        print(self.laserRechtsHinten)
        
        
        
        if  self.laserRechtsVorne < self.laserRechtsHinten:
            direction = 1
            linear_speed = 0.0
            angular_speed = self.CalcAngularSpeed(right_distance, desiredDistance, kp, ki, direction)
            
        elif self.laserRechtsVorne > self.laserRechtsHinten:
            direction = -1
            linear_speed = 0.0
            angular_speed = self.CalcAngularSpeed(right_distance, desiredDistance, kp, ki, direction)
        else:
            linear_speed = 0.1
            angular_speed = 0.0

        
        return angular_speed, linear_speed
    
    def Run(self):
        
        ctrlLinearXTemp = 0
        ctrlAngularZTemp = self.presentAng
        
        if self.state == self.Z_CALCMGV:

            ctrlLinearXTemp = 0.0
            ctrlAngularZTemp = self.presentAng
            self.CalcMGV()
            self.state = self.Z_ROTATE
            
        elif self.state == self.Z_ROTATE:
            
            ctrlLinearXTemp = 0.0
            
            if abs(self.mgvAngle - self.presentAng) < 0.1:
                self.state = self.Z_FOLLOW_MGV
            else:
                if self.mgvAngle > 0.0 and self.mgvAngle < np.pi:
                    ctrlAngularZTemp = 0.5
                else:
                    ctrlAngularZTemp = -0.5

        
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
                ctrlLinearXTemp= 3.0
            
        elif self.state == self.Z_WALLFOLLOWING:
            
            ctrlAngularZTemp, ctrlLinearXTemp = self.WallFollowingAlg(0.6, 0.1, 0.001)
            
            if not self.circumnavigate:
                self.SearchBestPoint()
                if (self.posRob[1] == self.posStartWF[1]) and (self.posRob[0] == self.posStartWF[0]):
                    self.circumnavigate = True
            else:
                if (self.posRob[1] == self.posMin[1]) and (self.posRob[0] == self.posMin[0]):
                    self.state = self.Z_CALCMGV
                    self.circumnavigate = False

        
        elif self.state == self.Z_END:
            print('Finish')
            exit()
            
        self.ctrlLinearX = ctrlLinearXTemp
        self.ctrlAngularZ = ctrlAngularZTemp

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.bug1Handler = Bug1(np.array([0.0,0.0]) , np.array([1.0,1.3]) )
        

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
        self.cmd_vel_pub.publish(velocity_cmd)


        
    def scan_callback(self, msg):        
        self.bug1Handler.SetLaser(msg)

        
    def location_callback(self, msg):
        
        self.bug1Handler.SetRoboPos(msg.pose.pose.position)
        self.bug1Handler.SetRoboAngle(msg.pose.pose.orientation)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    

    rclpy.spin(minimal_publisher)
    
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()