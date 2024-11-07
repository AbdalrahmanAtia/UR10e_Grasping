#! /usr/bin/env python3

'''this document is to help convert a calibrated transform (probably attained from between base to camera_color_optical_frame), to a transform between base to camera_link.
i.e. images captured/calibrated using roslaunch has a header.frame_id of 'camera_color_optical_frame', and that the robot poses are from tool0_controller which has the parent 'base'
the values printed as xyzquat should then be used in your program as a transform between 'base' to 'camera_link'. 
It assumes that 'camera_link' is the highest link for the camera itself after roslaunching it. Refer to calibration_step_0 and step_1 as well.
The xyzquat values attained from step_1 should be copied and pasted into 'calib_data'''
'''due to the way the folder paths are constructed, this script is not meant to be run using ros2 run. Just run it directly from inside vscode'''


import numpy as np
from time import sleep

import rclpy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import Pose
from myframe import Myframe
from myfileorganiser import FileLoader
from pathlib import Path
from enum import Enum, auto



class CalibConversion:
  def __init__(self, full_path, cam_name):
    self.node = rclpy.create_node("Calib_convert")
    self.full_path = full_path
    self.FL = FileLoader(self.full_path)
    self.FL.get_results_dictionary()
    self.cam_name = cam_name
    self.calib_data = self.FL.results_dict['Extrinsics']['xyzquat']

    
    self.bridge = CvBridge()
    # self.subscriber = rclpy.Subscriber("/camera/color/image_raw", Image, self.cb_imager, queue_size=10)
    # self.subscriber2 = rclpy.Subscriber("/camera/depth/image_rect_raw", Image, self.cb_depth, queue_size=10)
    # self.subscriber3 = rclpy.Subscriber("/camera/depth/color/points", PointCloud2, self.cb_pcd, queue_size=10)
    self.tf_buffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
    
    #this represents the frame that will be set as a child of 'tool0'. It should correspond to the highest parent of your camera frames.
    self.desired_child = f"{self.cam_name}_link"

    #this represent the frame in which the image/depth/cloud data came from during the calibration. i.e. the depth image was taken from this frame.
    self.original_child = self.FL.get_image_frame_id()

    self.timer = self.node.create_timer(1.0, self.cb_timer)

  def cb_timer(self):
    self.timer.cancel()
    try:
      value = self.tf_buffer.lookup_transform(self.original_child, self.desired_child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3))
      
    except tf2_ros.TransformException as ex:
      print(f"Could not transform between {self.cam_name}_link and {self.cam_name}_color_optical_frame': {ex}")

    
    F2 = Myframe.from_transformstamped(value)
    F1 = Myframe.from_xyzquat(self.calib_data)
    F_cam_link = Myframe.from_Tmat(np.matmul(F1.Tmat, F2.Tmat))
    
    key = "Final_handeye"
    value = {"xyzquat": F_cam_link.as_xyzquat(), "parent": "world/tool0", "child": f"{self.cam_name}_link"}
    self.FL.append_final_handeye_results(key, value)
    print("Final handeye:")
    print(f"xyzquat: {value['xyzquat']}")
    print(f"parent: {value['parent']}")
    print(f"child: {value['child']}")
    print("Please exit manually.")
    self.node.destroy_timer(self.timer)
    self.node.destroy_node()

  def ros_framework_stuff(self):
    self.br = tf2_ros.TransformBroadcaster()
    
  def cb_imager(self, data):
    print(data.header.frame_id)
    # pass

  def cb_depth(self, data):
    print(data.header.frame_id)
    # pass

  def cb_pcd(self, data):
    # print("pcd")
    print(data.header.frame_id)
    # pass

def main(args=None):
  rclpy.init(args=args)

  # cam_name = "Basler"
  cam_name = "D455"

  # full_path = Path(__file__).parents[2].joinpath("my_robot", "data", "20230914_1708H")  #Basler
  # full_path = Path(__file__).parents[2].joinpath("my_robot", "data", "20230914_1738H")  #Basler 
  # full_path = Path(__file__).parents[2].joinpath("my_robot", "data", "20230914_1758H")  #Basler 
  # full_path = Path(__file__).parents[2].joinpath("my_robot", "data", "20230914_1811H")  #Basler 
  # full_path = Path(__file__).parents[2].joinpath("my_robot", "data", "20231114_1450H")  #Basler 1638 * 1200
  # full_path = Path(__file__).parents[2].joinpath("my_robot", "data", "20231114_1505H")  #Basler 1638 * 1200
  
  # full_path = Path(__file__).parents[2].joinpath("my_robot", "data", "20230817_1904H")  #D455

  full_path = Path(__file__).parents[1].joinpath("data", "20241007_184458H")  #D455   mounted based on 3d printed part (furthest)
  
  abc = CalibConversion(full_path, cam_name)
  rclpy.spin(abc.node)
  
  rclpy.shutdown()


if __name__ == "__main__":
  main()