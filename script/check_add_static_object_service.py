#!/usr/bin/python

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from predictive_control.msg import *
from predictive_control.srv import *

import sys
import tf

#static trans
import tf2_ros
import geometry_msgs.msg
from numpy import linalg as LA
import random

def add_environment():
    rospy.loginfo("Calling static object service ... ")
    rospy.wait_for_service("/arm/predictive_control/StaticCollision/add_static_object")
    rospy.loginfo("Now all services are available ... ")

    try:
        client = rospy.ServiceProxy("/arm/predictive_control/StaticCollision/add_static_object", predictive_control.srv.StaticCollisionObject)
        request = predictive_control.srv.StaticCollisionObjectRequest()
        
        request.object_name = "box"
        request.object_id = "box"

        # dimension of object
        dimension = geometry_msgs.msg.Vector3()
        dimension.x = 1.30
        dimension.y = 1.30
        dimension.z = 0.10
        request.dimension = dimension

        # set object position and orientation
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

        request.primitive_pose = pose

        # call service to add static object into environments
        success = client(request)

        if (success):
            rospy.loginfo("Successfully added " + request.object_id + " into environment")

        else:
            rospy.logerr("Failed to add " + request.object_id + " into environment")   

        response = predictive_control.srv.StaticCollisionObjectResponse()
        print (response.success)
        print response.message

    except rospy.ServiceException as exc:
            rospy.logerr(" Service did not process request " + str(exc))


def remove_environment():
    rospy.loginfo("Calling static object service ... ")
    rospy.wait_for_service("/arm/predictive_control/StaticCollision/remove_static_object")
    rospy.loginfo("Now all services are available ... ")

    try:
        client = rospy.ServiceProxy("/arm/predictive_control/StaticCollision/remove_static_object",
                                    predictive_control.srv.StaticCollisionObject)
        request = predictive_control.srv.StaticCollisionObjectRequest()

        request.object_name = "box"
        request.object_id = "box"

        # call service to add static object into environments
        success = client(request)

        if (success):
            rospy.loginfo("Successfully removed " + request.object_id + " into environment")

        else:
            rospy.logerr("Failed to remove " + request.object_id + " into environment")


    except rospy.ServiceException as exc:
        rospy.logerr(" Service did not process request " + str(exc))


if __name__ == '__main__':
    rospy.init_node("pd_static_object_service")
    add_environment()
    rospy.sleep(3.0)
    remove_environment()