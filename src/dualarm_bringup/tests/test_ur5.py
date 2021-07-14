#!/usr/bin/env python

import time
import numpy as np
from pyrobot import Robot
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf

class simple_class:
  def __init__(self):
    self.sub = rospy.Subscriber('ur5_move', Pose, self.callback)
    self.bot = Robot( "ur5", use_arm=True, use_base=False, use_camera=False, use_gripper=False)
    self.bot.arm.move_to_neutral()
    self.count = 0

  def callback(self, msg):
    pose_position = np.array([msg.position.x, msg.position.y, msg.position.z])

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

    pose_orientation = np.array([roll, pitch, yaw])

    target_pose_1 = [
        {
            "position": pose_position,
            "orientation": pose_orientation,
        },
    ]

    for pose in target_pose_1:
        self.bot.arm.set_ee_pose(plan=True, **pose)

def main():
  obc = simple_class()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
