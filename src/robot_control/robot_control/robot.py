#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
from std_msgs.msg import Int8

class RobotControl(Node):
    def __init__(self):
        super().__init__("robot_command")
        # Subscription topic
        self.ball_pos  = self.create_subscription(Point, "/ball_pos",self.timer_callback_ball, 10)
        self.silo_pos  = self.create_subscription(Point, "/silo_pos",self.timer_callback_silo, 10)
        self.state_ball = self.create_subscription(Int8, "/robot_state", self.timer_callback_state, 10)
        # Publisher topic
        self.robot_speed = self.create_publisher (Twist, "/robot_speed", 10)
        # Global Variable
        self.speed_x_ball = 0
        self.speed_y_ball = 0
        self.speed_x_silo = 0
        self.speed_y_silo = 0
        self.state = 0

    def timer_callback_state(self, robot_state : Int8):
        # Get Command from Robot
        # state = 0 Find Ball 
        # state = 1 Find Silo
        self.state = robot_state.data # Get robot state

    def timer_callback_ball(self, ball : Point):
        # Get Ball Position
        ball_pos_x = ball.x
        ball_pos_y = ball.y
        ball_distance = ball.z
        # State for Control
        while(self.state == 0):
            if(ball_pos_x != 320 and ball_pos_x > 320):
                self.speed_x_ball = (ball_pos_x - 320) * 10
                self.speed_y_ball = ball_distance * 1500
            elif(ball_pos_x != 320 and ball_pos_x < 320):
                self.speed_x_ball = (ball_pos_x - 320) * 10
                self.speed_y_ball = ball_distance * 1500
            elif(ball_pos_x > 310 and ball_pos_x < 330):
                self.speed_x_ball = 0
                self.speed_y_ball = ball_distance * 3000  # speed_y (m/s)
            if(ball_pos_x == 0 and ball_pos_y == 0):
                self.speed_x_ball = 0
                self.speed_y_ball = 0
            self.timer_callback_speed(self.speed_x_ball, self.speed_y_ball)
            self.get_logger().info(f"find_ball!!!")

    def timer_callback_silo(self, silo : Point):
        # Get Ball Position
        silo_pos_x = silo.x
        silo_pos_y = silo.y
        silo_distance = silo.z
        # State for Control
        while(self.state == 1):
            if(silo_pos_x != 320 and silo_pos_x > 320):
                self.speed_x_silo = (silo_pos_x - 320) * 10
                self.speed_y_silo = -silo_distance * 500
            elif(silo_pos_x != 320 and silo_pos_x < 320):
                self.speed_x_silo = (silo_pos_x - 320) * 10
                self.speed_y_silo = -silo_distance * 500
            elif(silo_pos_x > 315 and silo_pos_x < 325):
                self.speed_x_silo = 0
                self.speed_y_silo = -1800 # speed_y (m/s)
            if(silo_pos_x == 0 and silo_pos_y == 0):
                self.speed_x_silo = 0
                self.speed_y_silo = 0
            self.timer_callback_speed(self.speed_x_silo, self.speed_y_silo)
            self.get_logger().info(f"find_ball!!!")

    def timer_callback_speed(self, linear_x, linear_y):
        # Speed Send
        robot_command_ball = Twist()
        robot_command_ball.linear.x = float(linear_x)
        robot_command_ball.linear.y = float(linear_y)
        self.robot_speed.publish(robot_command_ball)

def main(args =None):
    rclpy.init(args = None)
    node = RobotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()