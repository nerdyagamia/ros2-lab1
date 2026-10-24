#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen

class ResetServer(Node):
    def __init__(self):
        super().__init__('reset_server')
        self.cb_group = ReentrantCallbackGroup()
        
        # Service Clients (calling turtlesim services)
        self.clear_client = self.create_client(Empty, '/clear', callback_group=self.cb_group)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=self.cb_group)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen', callback_group=self.cb_group)
        
        # Service Server (the one you will call)
        self.srv = self.create_service(Empty, 'reset_turtle', self.reset_callback, callback_group=self.cb_group)
        self.get_logger().info('Reset service (/reset_turtle) is ready.')

    def reset_callback(self, request, response):
        self.get_logger().info('Resetting turtle...')
        
        # 1. Turn off the pen
        pen_req = SetPen.Request()
        pen_req.r, pen_req.g, pen_req.b, pen_req.width, pen_req.off = 255, 255, 255, 3, 1
        self.pen_client.call(pen_req)
        
        # 2. Teleport back to center (5.54, 5.54) at angle 0
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = 5.544445
        teleport_req.y = 5.544445
        teleport_req.theta = 0.0
        self.teleport_client.call(teleport_req)
        
        # 3. Turn the pen back on
        pen_req.off = 0
        self.pen_client.call(pen_req)
        
        # 4. Clear the screen
        self.clear_client.call(Empty.Request())
        
        self.get_logger().info('Turtle reset complete.')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()