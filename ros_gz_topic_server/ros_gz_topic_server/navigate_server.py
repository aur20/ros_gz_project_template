# import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from action_tutorials_rosgz.action import Navigate  # The Navigate action definition
from sensor_msgs.msg import LaserScan  # Example sensor message type for detecting obstacles
from geometry_msgs.msg import Twist, Pose  # For robot motion control
from nav_msgs.msg import Odometry  # For robot location
import time
import math

import rclpy

from action_tutorials_rosgz.action import Navigate

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class NavigateActionServer(Node):

    def __init__(self):
        super().__init__('navigation_action_server')
        self.callback_group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        # Create publisher to control the robot's velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10, callback_group=self.callback_group)

        # Subscribe to a topic that provides distance to obstacles (e.g., LIDAR or laser scanner)
        self.obstacle_distance = float('inf')  # Initialize obstacle distance to a very high value
        self.create_subscription(LaserScan, '/diff_drive/scan', self.scan_callback, 10, callback_group=self.callback_group)

        # Subscribe to current robot state
        self.initial_pose = False
        self.robot_location = Pose()
        self.create_subscription(Odometry, '/diff_drive/odometry', self.location_callback, 10, callback_group=self.callback_group)

        # Stop flag
        self.stopped = False

    def scan_callback(self, msg):
        # Get the minimum distance to an obstacle in front of the robot
        # TODO: parse the message and extract the distance to the closest obstacle
        self.obstacle_distance = min(msg.ranges)

    def location_callback(self, msg):
        # Get the current location of the robot
        self.initial_pose = True
        self.robot_location = msg.pose.pose  # This is already a Pose type (geometry_msgs/Pose)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        rate = self.create_rate(10)
        while not self.initial_pose:
            self.get_logger().info(f'Awaiting initial pose...')
            rate.sleep()

        stop_reason = ""
        feedback_msg = Navigate.Feedback()
        feedback_msg.distance_to_target = goal_handle.request.target_distance
        feedback_msg.distance_to_obstacle_frontleft = 0.0
        feedback_msg.distance_to_obstacle_frontright = 0.0

        # Phase 0: Turn the robot to face the target direction
        turn = goal_handle.request.angle
        self.turn(turn)

        # Phase 1: Move straight until target distance or obstacle
        starting_pose = self.robot_location
        distance_traveled = 0.0
        target_distance = goal_handle.request.target_distance

        while distance_traveled < target_distance and not self.stopped:
            if self.obstacle_distance < 0.2:  # Stop if obstacle is too close (e.g., < 0.2 meters)
                stop_reason = "Obstacle detected within safety distance."
                break

            # Provide feedback to the client on the distance to the obstacle
            feedback_msg.distance_to_target = target_distance - distance_traveled
            feedback_msg.distance_to_obstacle_frontleft = self.obstacle_distance
            feedback_msg.distance_to_obstacle_frontright = self.obstacle_distance
            goal_handle.publish_feedback(feedback_msg)

            # Command the robot to move forward
            velocity_cmd = Twist()
            velocity_cmd.linear.x = 2.0
            self.cmd_vel_pub.publish(velocity_cmd)

            # Simulate distance traveled (in a real robot, you would use odometry feedback)
            distance_traveled = abs(self.robot_location.position.x - starting_pose.position.x)
            rate.sleep()

            # Check if the goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                self.stopped = True
                stop_reason = "Goal canceled by the client."

        # Stop the robot after moving straight
        velocity_cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(velocity_cmd)

        if not stop_reason:
            stop_reason = "Reached the target distance."

        # Action succeeded
        goal_handle.succeed()

        # Prepare result
        result = Navigate.Result()
        result.stop_reason = stop_reason
        result.final_distance_to_obstacle = self.obstacle_distance
        result.final_distance_to_target = target_distance - distance_traveled
        self.get_logger().info(f'Action completed: {stop_reason}')
        return result

    def turn(self, angle):
        # Command the robot to turn by publishing angular velocity commands
        velocity_cmd = Twist()
        rate = self.create_rate(10)

        if (angle > 0):
            velocity_cmd.angular.z = 2.
        else:
            velocity_cmd.angular.z = -2.


        (roll, pitch, initial_yaw) = euler_from_quaternion(self.robot_location.orientation.x, self.robot_location.orientation.y, self.robot_location.orientation.z, self.robot_location.orientation.w)

        self.get_logger().info(f'Turning to {initial_yaw + angle}')
        self.cmd_vel_pub.publish(velocity_cmd)

        while not self.stopped:
            (roll, pitch, yaw) = euler_from_quaternion(self.robot_location.orientation.x, self.robot_location.orientation.y, self.robot_location.orientation.z, self.robot_location.orientation.w)
            if abs(yaw - initial_yaw - angle) < 0.1:
                break
            self.get_logger().info(f'Curr rot: {roll} {pitch} {yaw}')
            rate.sleep()
        # Stop the robot after turning
        velocity_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(velocity_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigateActionServer()

    # Run the node with a multi-threaded executor to handle multiple callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
