#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs # type: ignore
import numpy as np
from ultralytics import YOLO  # type: ignore
from ament_index_python.packages import get_package_share_directory

class SiloTrackingRed(Node):
    def __init__(self):
        super().__init__("silo_tracking_red")
        # Publisher topic
        self.distance_pub = self.create_publisher(Point, "/silo_pos", 10)
        # Path to model
        weight = "/silo_l.engine"
        # weight = "/best.pt"
        path = get_package_share_directory('silo_tracking')
        full_path_silo = path + weight
        # Get AI
        self.model_silo = YOLO(full_path_silo, task = 'detect')
        # Object Distance
        self.distance = 0
        # Delay time
        timer_period = 0.001
        # Serial_number Camera D415
        self.id_cam = "309122300259"

        try:
            self.pipe = rs.pipeline()
            self.cfg  = rs.config()
            self.cfg.enable_device(self.id_cam)
            # self.cfg.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8, 30) # USB 3.0 RGB
            # self.cfg.enable_stream(rs.stream.depth, 1280,720, rs.format.z16, 30)  # USB 3.0 Depth
            self.cfg.enable_stream(rs.stream.color, 640,360, rs.format.bgr8, 30) # USB 3.0 RGB
            self.cfg.enable_stream(rs.stream.depth, 640,360, rs.format.z16, 30)     # USE 2.0 Depth
            # Config Timer
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
        # Case Tracking
        empty_ball = self.model_silo.predict(source = color_image, show = False, conf = 0.1, classes = 0) # predict silo
        two_ball = self.model_silo.predict(source = color_image, show = False, conf = 0.1, classes = 3) # predict silo
        one_ball = self.model_silo.predict(source = color_image, show = False, conf = 0.1, classes = 1) # predict silo
        # Case Silo 2-ball
        if len(two_ball[0].boxes) != 0:
            for box in two_ball:
                boxes_silo = box.boxes
                for silo in boxes_silo:
                    frame_silo = silo.xyxy[0].to('cpu').detach().numpy().copy()
                    x1, y1, x2, y2 = frame_silo
                    self.distance = depth_frame.get_distance(int((x1+x2)/2), int(int(y2)-int((y2/y2)*5))) # ((x1 + x2)/2 ,((y1 + y2)/2))
                    data.append([int((x1+x2)/2), int((y1+y2)/2),self.distance])
                    data.sort(key=lambda item:item[0], reverse = True)
                    # Tracking Codinate of Object
                    # Publish Object pos 
                object_distance.x = float(data[0][0])   # X ---> Frame
                object_distance.y = float(data[0][1])   # Y ---> Frame
                object_distance.z = float(data[0][2])   # Distance of Object
                self.distance_pub.publish(object_distance)
        # Case Silo 0-ball
        elif len(empty_ball[0].boxes) != 0:
            for box in empty_ball:
                boxes_silo = box.boxes
                for silo in boxes_silo:
                    frame_silo = silo.xyxy[0].to('cpu').detach().numpy().copy()
                    x1, y1, x2, y2 = frame_silo
                    self.distance = depth_frame.get_distance(int((x1+x2)/2), int(int(y2)-int((y2/y2)*5))) # ((x1 + x2)/2 ,((y1 + y2)/2))
                    data.append([int((x1+x2)/2), int((y1+y2)/2),self.distance])
                    data.sort(key=lambda item:item[0])
                    # Tracking Codinate of Object
                    # Publish Object pos
                object_distance.x = float(data[0][0])   # X ---> Frame
                object_distance.y = float(data[0][1])   # Y ---> Frame
                object_distance.z = float(data[0][2])   # Distance of Object
                self.distance_pub.publish(object_distance)
        # Case Silo 1-ball
        elif len(one_ball[0].boxes) != 0:
            for box in one_ball:
                boxes_silo = box.boxes
                for silo in boxes_silo:
                    frame_silo = silo.xyxy[0].to('cpu').detach().numpy().copy()
                    x1, y1, x2, y2 = frame_silo
                    self.distance = depth_frame.get_distance(int((x1+x2)/2), int(int(y2)-int((y2/y2)*5))) # ((x1 + x2)/2 ,((y1 + y2)/2))
                    data.append([int((x1+x2)/2), int((y1+y2)/2),self.distance])
                    data.sort(key=lambda item:item[0])
                    # Tracking Codinate of Object
                    # Publish Object pos 
                object_distance.x = float(data[0][0])   # X ---> Frame
                object_distance.y = float(data[0][1])   # Y ---> Frame
                object_distance.z = float(data[0][2])   # Distance of Object
                self.distance_pub.publish(object_distance)
        else:
            object_distance.x = 0.0
            object_distance.y = 0.0
            object_distance.z = 0.0
            self.distance_pub.publish(object_distance)
                
def main(args = None):
    rclpy.init(args = None)
    silo_tracking = SiloTrackingRed()
    rclpy.spin(silo_tracking)
    silo_tracking.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()