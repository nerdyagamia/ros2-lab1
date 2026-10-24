#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_interfaces.srv import SetSpeeds
from turtlesim_interfaces.action import GoToPose

# --- State Pattern Interfaces ---
class MotionState:
    def execute(self, context):
        raise NotImplementedError

class TranslatingState(MotionState):
    def execute(self, context):
        dx = context.target.x - context.current_pose.x
        dy = context.target.y - context.current_pose.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            context.publish_vel(0.0, 0.0)
            time.sleep(0.5)
            context.current_state = RotatingState()
            return True

        angle_to_goal = math.atan2(dy, dx)
        angle_error = context.normalize_angle(angle_to_goal - context.current_pose.theta)

        linear_vel = context.Kp_linear * distance
        angular_vel = context.Kp_angular * angle_error

        max_linear = context.get_parameter('max_linear_speed').value
        max_angular = context.get_parameter('max_angular_speed').value

        v_l = max(min(linear_vel, max_linear), -max_linear)
        v_a = max(min(angular_vel, max_angular), -max_angular)

        context.publish_vel(v_l, v_a)
        context.publish_feedback(distance)
        return True

class RotatingState(MotionState):
    def execute(self, context):
        yaw_error = context.normalize_angle(context.target.yaw - context.current_pose.theta)
        if abs(yaw_error) < 0.05:
            context.publish_vel(0.0, 0.0)
            context.current_state = FinishedState()
            return True

        angular_vel = context.Kp_angular * yaw_error
        max_angular = context.get_parameter('max_angular_speed').value
        v_a = max(min(angular_vel, max_angular), -max_angular)

        context.publish_vel(0.0, v_a)
        context.publish_feedback(0.0)
        return True

class FinishedState(MotionState):
    def execute(self, context):
        return False

# --- Context Node ---
class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('go_to_pose_server')
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.srv = self.create_service(SetSpeeds, 'set_speeds', self.set_speeds_callback)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = None
        
        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, GoToPose, 'go_to_pose', self.execute_callback, callback_group=self.cb_group
        )
        
        # Controller gains
        self.Kp_linear = 1.5
        self.Kp_angular = 4.0
        
        # State pattern variables
        self.current_state = None
        self.target = None
        self.goal_handle = None

        self.get_logger().info('GoToPose action server (State Pattern) is ready.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def set_speeds_callback(self, request, response):
        if request.max_linear <= 0.0 or request.max_angular <= 0.0:
            response.success = False
            return response
        self.set_parameters([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, request.max_linear),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, request.max_angular)
        ])
        response.success = True
        return response

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def publish_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)

    def publish_feedback(self, distance):
        feedback = GoToPose.Feedback()
        feedback.distance_remaining = distance
        self.goal_handle.publish_feedback(feedback)

    def execute_callback(self, goal_handle):
        while self.current_pose is None and rclpy.ok():
            time.sleep(0.1)

        self.goal_handle = goal_handle
        self.target = goal_handle.request.target_pose
        self.current_state = TranslatingState()

        # State machine execution loop
        while rclpy.ok():
            is_running = self.current_state.execute(self)
            if not is_running:
                break
            time.sleep(0.05)

        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()