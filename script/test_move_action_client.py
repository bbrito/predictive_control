#!/usr/bin/python

import rospy
import actionlib

from predictive_control.msg import *
from predictive_control.srv import *

def move_action_client():

    rospy.loginfo("Starting move client function calling ...")
    move_client = actionlib.SimpleActionClient('/arm/move_action', predictive_control.msg.moveAction)
    move_client.wait_for_server()

    goal = predictive_control.msg.moveGoal()
    goal.target_frame_id = "object_1"

    # header
    goal.target_endeffector_pose.header.frame_id = "world"
    goal.target_endeffector_pose.header.stamp = rospy.Time.now()

    # position
    goal.target_endeffector_pose.pose.position.x = 0.30605
    goal.target_endeffector_pose.pose.position.y = -0.3269
    goal.target_endeffector_pose.pose.position.z = 0.70605

    # orientation
    goal.target_endeffector_pose.pose.orientation.w = 0.99997
    goal.target_endeffector_pose.pose.orientation.x = 0.00120
    goal.target_endeffector_pose.pose.orientation.y = 0.005199
    goal.target_endeffector_pose.pose.orientation.z = 0.00450

    # Sends the goal to the action server
    rospy.loginfo("Now sending goal")
    move_client.send_goal(goal)

    # Waits for the server to finish performing the action
    finished_before_timeout = move_client.wait_for_result(rospy.Duration(50, 0))

    if finished_before_timeout:
        state=move_client.get_state()
        print "Action finished: %s"%state
    else:
        print "Action did not finish within timeout"
    return

if __name__ == '__main__':
    rospy.init_node("pd_move_action_client")
    move_action_client()