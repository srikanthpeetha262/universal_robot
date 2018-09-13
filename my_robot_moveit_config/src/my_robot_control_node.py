#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley
## Editor: Srikanth Peetha

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.

import sys
import copy
import rospy
import warnings
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import Pose
from std_msgs.msg import String

rand_pose = Pose()
target_pose = Pose()

def move_group_python_interface_tutorial():
	global rand_pose, target_pose
	
	print "============ Starting manual control node"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('ur5_custom_controller_node', anonymous=True)

	robot = moveit_commander.RobotCommander()
	
	#print "============ Robot Groups:"
	#print robot.get_group_names()
	
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("manipulator")

	display_trajectory_publisher = rospy.Publisher(
							'/move_group/display_planned_path',
							moveit_msgs.msg.DisplayTrajectory,
							queue_size=20)


	## List of all the groups in the robot
	print "============ Robot Groups:"
	print robot.get_group_names()

	print "============ Printing robot pose"
	print group.get_current_pose()
	print "============\n"

	group.clear_pose_targets()


	## Adding/Removing Objects and Attaching/Detaching Objects
	## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = robot.get_planning_frame()
	#box_pose.pose.position.x = 0.5
	#box_pose.pose.position.y = 0
	box_pose.pose.position.z = 0.25
	box_pose.pose.orientation.w = 1.0
	box_name = "box"
	print "============ Adding  the collision object"
	scene.add_box(box_name, box_pose, size=(2, 0.7, 1.5))
	rospy.sleep(3)



	# Set a new pose for the arm to move
	## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	target_pose.position.x = 0.729975241828
	target_pose.position.y = 0.334318680207
	target_pose.position.z = 1.01582207722

	target_pose.orientation.x = -0.0521360287869
	target_pose.orientation.y = 0.544095301678
	target_pose.orientation.z = 0.833607238883
	target_pose.orientation.w = 0.0796310773232

	print "============ Waiting while RVIZ to display plan"
	group.set_pose_target(target_pose)
	plan1 = group.plan()

	with warnings.catch_warnings(record=True) as w:
		# Cause all warnings to always be triggered.
		warnings.simplefilter("always")
		# Verify some things
		if( "No execution attempted" in str(w) ):
			error = True
		else:
			error = False

	if (error == False):
		## Move the robot in gazebo
		## ^^^^^^^^^^^^^^^^^^^^^^^^
		print "============ Waiting while Robot to reach it's destination"
		group.go(wait=True)
		rospy.sleep(5)
	
	#group.clear_pose_targets()

	remove_box = str(raw_input("would you like to remove the box:"))
	if(remove_box == "y"):
		scene.remove_world_object(box_name)
	else:
		print "box not removed"

	## When finished shut down moveit_commander.
	moveit_commander.roscpp_shutdown()
	print "============ STOPPING"


if __name__=='__main__':
  try:
	move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
	pass


'''
## Pose_1 ##

pose: 
  position: 
    x: 0.729975241828
    y: 0.334318680207
    z: 1.01582207722
  orientation: 
    x: -0.0521360287869
    y: 0.544095301678
    z: 0.833607238883
    w: 0.0796310773232



'''

