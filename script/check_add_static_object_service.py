#!/usr/bin/python

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from predictive_control.msg import *
from predictive_control.srv import *

import sys
import tf

# static trans
import tf2_ros
import geometry_msgs.msg
from numpy import linalg as LA
import random


def add_environment():
    rospy.loginfo("Calling static object service ... ")
    rospy.wait_for_service("/arm/predictive_control/StaticCollision/add_static_object")
    rospy.loginfo("Now all services are available ... ")

    try:
        client = rospy.ServiceProxy("/arm/predictive_control/StaticCollision/add_static_object",
                                    predictive_control.srv.StaticCollisionObject)
        request = predictive_control.srv.StaticCollisionObjectRequest()

        request.collision_object.object_name = "box"
        request.collision_object.object_id = "box"

        # object pose
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time().now()

        # position
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.10

        # object orientation
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        request.collision_object.primitive_poses.append(pose)

        rospy.loginfo("Now sending goal requst to service server")
        # call service to add static object into environments
        success = client(request)

        if (success):
            rospy.loginfo("Successfully added " + request.collision_object.object_id + " into environment")

        else:
            rospy.logerr("Failed to add " + request.collision_object.object_id + " into environment")

    except rospy.ServiceException as exc:
        rospy.logerr(" Service did not process request " + str(exc))



if __name__ == '__main__':
    rospy.init_node("pd_static_object_service")
    add_environment()
