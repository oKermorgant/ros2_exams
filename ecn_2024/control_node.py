#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(get_package_share_directory('ecn_2024'))
from tracker import Tracker
from nav_msgs.msg import Path

# TODO import relevant message and service types



class ControlNode(Node):
  def __init__(self):
    super().__init__('control')

    # helper class to perform the control part
    self.tracker = Tracker(self)

    # the path to publish will be in the map frame
    self.path = Path()
    self.path.header.frame_id = "map"

    # declaring a string parameter
    frame = self.declare_parameter('frame', 'base_link').value

    # TODO init the frame of the robot from a parameter
    self.tracker.setFrame("some_frame")

    # TODO init publishers for cmd_vel and (optional) path

    # TODO create a Letter request and fill it with parameters

    # TODO call the service and give a callback function for the result

    # TODO init timer for control


  def letter_callback(self, future):
    # TODO use future.result() to init the tracker
    pass


  def track(self):
    # TODO call this function in the timer
    pass





if __name__ == '__main__':
  # boilerplate main function
  rclpy.init()
  node = ControlNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

