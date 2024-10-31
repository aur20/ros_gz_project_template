import rclpy
from random import random
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

def getMovement(posGoal, posRob, thetaRob):
    '''Probs to Kosty'''
    x = 0.0
    z = 0.0
    mgv = posGoal - posRob
    mgvAngle = np.arctan2(mgv[1], mgv[0])
    if np.linalg.norm(mgv) < 0.1:
        x = 0.0
    else:
        x = np.clip(np.linalg.norm(mgv), 0., 2.)

    if abs(mgvAngle - thetaRob) < 0.1:
        z = 0.0
    else:
        z = np.clip(2.0 * (mgvAngle - thetaRob), -2., 2.)

    return x, z

class MotionController(Node):

    def __init__(self):
        super().__init__('motion_controller')
        self.setInitialPosition = False
        self.position = Pose()
        self.initialPosition = Pose()
        self.FarAwayPosition = Pose()
        self.FarAwayPosition.position.x = random() * 8 + 1.
        self.FarAwayPosition.position.y = random() * 8 + 1.
        # self.FarAwayPosition.angular.z = random() * 6.28
        self.FarAwayPositionVisited = False
        self.EndPosition = False
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive/odometry',
            self.listener_callback,
            10)

        self.x = 0.0
        self.z = 0.0
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        self.position = msg.pose.pose
        if not self.setInitialPosition:
            self.get_logger().info("Setting initial position")
            self.initialPosition = msg.pose.pose
            self.setInitialPosition = True
            self.get_logger().info(f"Going from position {self.initialPosition} to {self.FarAwayPosition}")

    def timer_callback(self):
        x = 0.0
        z = 0.0

        # Move away from the initial position
        if self.setInitialPosition:
            if not self.FarAwayPositionVisited:
                if abs(self.FarAwayPosition.position.x - self.position.position.x) < 0.1 and abs(self.FarAwayPosition.position.y - self.position.position.y) < 0.1:
                    self.FarAwayPositionVisited = True
                    self.get_logger().info(f"Arrived at position {self.FarAwayPosition}")
                else:
                    posGoal = np.array([self.FarAwayPosition.position.x, self.FarAwayPosition.position.y])
                    posRob = np.array([self.position.position.x, self.position.position.y])
                    theta = euler_from_quaternion(self.position.orientation.x, self.position.orientation.y, self.position.orientation.z, self.position.orientation.w)[2]
                    x, z = getMovement(posGoal, posRob, theta)

            # Move back to the initial position
            elif self.FarAwayPositionVisited and not self.EndPosition:
                if abs(self.initialPosition.position.x - self.position.position.x) < 0.1 and abs(self.initialPosition.position.y - self.position.position.y) < 0.1:
                    self.EndPosition = True
                    self.get_logger().info(f"Arrived at position {self.initialPosition}")
                else:
                    posGoal = np.array([self.initialPosition.position.x, self.initialPosition.position.y])
                    posRob = np.array([self.position.position.x, self.position.position.y])
                    theta = euler_from_quaternion(self.position.orientation.x, self.position.orientation.y, self.position.orientation.z, self.position.orientation.w)[2]
                    x, z = getMovement(posGoal, posRob, theta)

            # Stop turtle and end
            else:
                self.get_logger().info("Finished the program")
                rclpy.shutdown()

        msg = Twist()
        msg.linear.x = x
        msg.angular.z = z
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Currently publishing: {msg}")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MotionController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()