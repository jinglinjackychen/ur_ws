#!/usr/bin/env python2

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import *
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from moveit_commander.conversions import pose_to_list
from medical_msgs.msg import *
from medical_msgs.srv import *
from interbotix_sdk.msg import SingleCommand

class gripper_feedback:
	def __init__(self):
		# subscriber
		rospy.Subscriber("/vx300s/joint_states", JointState, self.callback, queue_size = 1)

		# publisher
		self.pub_joint_command = rospy.Publisher("/vx300s/single_joint/command", SingleCommand, queue_size=1) 

		self.old_position = 0 
		self.count = 0
		self.lock = 0

	def callback(self, msg):
		if msg.position[6] < 0.9 and self.lock == 0:
			if abs(self.old_position - msg.position[6]) <= 0.001:
				self.count +=1
				if self.count == 3:
					cmd = SingleCommand()
					cmd.joint_name = "gripper"
					cmd.cmd = msg.position[6] + 0.06
					self.pub_joint_command.publish(cmd)
					self.lock = 1
					self.count = 0
		if msg.position[6] > 1.1:
			self.lock = 0

		self.old_position = msg.position[6]
		rospy.sleep(0.02)
 
if __name__ == "__main__":
	rospy.init_node('gripper_feedback')
	gripper_feedback = gripper_feedback()
	rospy.spin()