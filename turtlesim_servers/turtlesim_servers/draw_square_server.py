#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from turtlesim_interfaces.action import GoToPose, DrawSquare

class DrawSquareServer(Node):
    def __init__(self):
        super().__init__('draw_square_server')
        self.cb_group = ReentrantCallbackGroup()
        
        # Service Client to reset the turtle
        self.reset_client = self.create_client(Empty, '/reset_turtle', callback_group=self.cb_group)
        
        # Action Client to send goals to goToPose
        self.goto_client = ActionClient(self, GoToPose, '/go_to_pose', callback_group=self.cb_group)
        
        # Action Server for draw_square
        self._action_server = ActionServer(
            self, DrawSquare, 'draw_square', self.execute_callback, callback_group=self.cb_group
        )
        self.get_logger().info('DrawSquare action server is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received request to draw square...')
        side_length = goal_handle.request.side_length
        feedback_msg = DrawSquare.Feedback()
        
        # 1. Reset Turtle First
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            self.reset_client.call(Empty.Request())
            self.get_logger().info('Turtle reset successful.')
            time.sleep(1.0)  # Brief pause to let the teleport settle
            
        # 2. Wait for GoToPose action server
        self.goto_client.wait_for_server()
        
        # 3. Define the 4 corners relative to the center (5.54, 5.54)
        start_x, start_y = 5.544445, 5.544445
        
        # Sequence: Move Right, Move Up, Move Left, Move Down
        # Yaw points toward the *next* direction
        corners = [
            (start_x + side_length, start_y, 1.5708),                 # Corner 1
            (start_x + side_length, start_y + side_length, 3.14159),  # Corner 2
            (start_x, start_y + side_length, -1.5708),                # Corner 3
            (start_x, start_y, 0.0)                                   # Corner 4
        ]

        # 4. Visit each corner sequentially
        for i, (x, y, yaw) in enumerate(corners):
            feedback_msg.side_number = i + 1
            goal_handle.publish_feedback(feedback_msg)
            
            goal_msg = GoToPose.Goal()
            goal_msg.target_pose.x = float(x)
            goal_msg.target_pose.y = float(y)
            goal_msg.target_pose.yaw = float(yaw)
            
            self.get_logger().info(f'Moving to corner {i+1}: ({x:.2f}, {y:.2f})')
            
            # Send the goal asynchronously
            send_goal_future = self.goto_client.send_goal_async(goal_msg)
            while rclpy.ok() and not send_goal_future.done():
                time.sleep(0.05)
            
            goto_goal_handle = send_goal_future.result()
            if not goto_goal_handle.accepted:
                self.get_logger().error(f'Corner {i+1} goal rejected!')
                goal_handle.abort()
                return DrawSquare.Result()
            
            # Wait for the turtle to finish moving to this corner
            get_result_future = goto_goal_handle.get_result_async()
            while rclpy.ok() and not get_result_future.done():
                time.sleep(0.05)
        
        goal_handle.succeed()
        result = DrawSquare.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()