#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs # type: ignore
import numpy as np
from ultralytics import YOLO  # type: ignore
from ament_index_python.packages import get_package_share_directory

class BallTrackingBlue(Node):
    def __init__(self):
        super().__init__("ball_tracking_blue")
        # Publisher topic
        self.distance_pub = self.create_publisher(Point, "/ball_pos", 10)
        # Path to model
        weights = "/INT8.engine"
        # File Train Directory
        package = get_package_share_directory('ball_tracking')
        full_path_ball = package + weights
        self.model_ball = YOLO(full_path_ball, task = 'detect')
        # Object Distance
        self.distance = 0
        # Delay
        timer_period = 0.001
        # Serial_number Camera D455
        
        self.id_cam = "147122075475"
        # self.id_cam = "234222302984"
        
        try:
            self.pipe = rs.pipeline()
            self.cfg  = rs.config()
            self.cfg.enable_device(self.id_cam)
            self.cfg.enable_stream(rs.stream.color, 640,360, rs.format.bgr8, 30) # USB 3.0 RGB
            self.cfg.enable_stream(rs.stream.depth, 640,360, rs.format.z16, 30)  # USB 3.0 Depth
            self.pipe.start(self.cfg)
            self.timer_rgb = self.create_timer(timer_period, self.timer_callback_pub)
        except Exception as e:
            print(e)
            self.get_logger().error("INTEL REALSENSE IS NOT CONNECTED")
        
    def timer_callback_pub(self):
        object_distance = Point()
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        data = []
        # Get Data from Camera
        color_image = np.asanyarray(color_frame.get_data())
        # Results
        result = self.model_ball.track(source=color_image, show = False, conf = 0.3, classes = 0) # predict ball
        if len(result[0].boxes) != 0:
            for box in result:
                boxes_ball = box.boxes
                for ball in boxes_ball:
                    frame_ball = ball.xyxy[0].to('cpu').detach().numpy().copy()
                    x1, y1, x2, y2 = frame_ball
                    self.distance = depth_frame.get_distance(int((x1+x2)/2), int((y1+y2)/2)) # ((x1 + x2)/2 ,((y1 + y2)/2))
                    data.append([int((x1+x2)/2), int((y1+y2)/2),self.distance])
                    data.sort(key=lambda item:item[2])
                    # Publish Object pos 
                object_distance.x = float(data[0][0])   # X ---> Frame
                object_distance.y = float(data[0][1])   # Y ---> Frame
                object_distance.z = float(data[0][2]+0.2)   # Distance of Object
                self.distance_pub.publish(object_distance)
        else:
            object_distance.x = 0.0
            object_distance.y = 0.0
            object_distance.z = 0.0
            self.distance_pub.publish(object_distance)
                
def main(args=None):
    rclpy.init(args=None)
    ball_tracking = BallTrackingBlue()
    rclpy.spin(ball_tracking)
    ball_tracking.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()