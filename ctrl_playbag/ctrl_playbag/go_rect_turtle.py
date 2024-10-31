import rclpy
from random import random
import numpy as np
from rclpy.node import Node
from turtlesim.msg import Pose  # For robot motion control
from geometry_msgs.msg import Twist

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
        self.FarAwayPosition.x = random() * 8 + 1.
        self.FarAwayPosition.y = random() * 8 + 1.
        self.FarAwayPosition.theta = random() * 6.28
        self.FarAwayPositionVisited = False
        self.EndPosition = False
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)

        self.x = 0.0
        self.z = 0.0
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        self.position = msg
        if not self.setInitialPosition:
            self.get_logger().info("Setting initial position")
            self.initialPosition = msg
            self.setInitialPosition = True
            self.get_logger().info(f"Going from position {self.initialPosition} to {self.FarAwayPosition}")

    def timer_callback(self):
        x = 0.0
        z = 0.0

        # Move away from the initial position
        if self.setInitialPosition:
            if not self.FarAwayPositionVisited:
                if abs(self.FarAwayPosition.x - self.position.x) < 0.1 and abs(self.FarAwayPosition.y - self.position.y) < 0.1:
                    self.FarAwayPositionVisited = True
                    self.get_logger().info(f"Arrived at position {self.FarAwayPosition}")
                else:
                    posGoal = np.array([self.FarAwayPosition.x, self.FarAwayPosition.y])
                    posRob = np.array([self.position.x, self.position.y])
                    x, z = getMovement(posGoal, posRob, self.position.theta)

            # Move back to the initial position
            elif self.FarAwayPositionVisited and not self.EndPosition:
                if abs(self.initialPosition.x - self.position.x) < 0.1 and abs(self.initialPosition.y - self.position.y) < 0.1:
                    self.EndPosition = True
                    self.get_logger().info(f"Arrived at position {self.initialPosition}")
                else:
                    posGoal = np.array([self.initialPosition.x, self.initialPosition.y])
                    posRob = np.array([self.position.x, self.position.y])
                    x, z = getMovement(posGoal, posRob, self.position.theta)

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