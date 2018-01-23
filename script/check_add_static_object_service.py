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

def add_environment(object_id):
    rospy.loginfo("Calling static object service ... ")
    rospy.wait_for_service("/arm/predictive_control/StaticCollision/add_static_object")
    rospy.loginfo("Now all services are available ... ")

    try:
        client = rospy.ServiceProxy("/arm/predictive_control/StaticCollision/add_static_object", predictive_control.srv.StaticCollisionObject)
        request = predictive_control.srv.StaticCollisionObjectRequest()

        request.object_name = object_id
        request.object_id = object_id

        # dimension of object
        dimension = geometry_msgs.msg.Vector3()
        dimension.x = 0.40
        dimension.y = 0.60
        dimension.z = 0.40
        request.dimension = dimension

        # set object position and orientation
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time().now()
        
        # position
        pose.pose.position.x = 0.30 + dimension.x * 0.5
        pose.pose.position.y = -0.30 + dimension.y * 0.5
        pose.pose.position.z = 0.00 + dimension.z * 0.5

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


def add_environment_from_file(object_id):
    rospy.loginfo("Calling static object service ... ")
    rospy.wait_for_service("/arm/predictive_control/StaticCollision/add_static_object")
    rospy.loginfo("Now all services are available ... ")

    try:
        client = rospy.ServiceProxy("/arm/predictive_control/StaticCollision/add_static_object",
                                    predictive_control.srv.StaticCollisionObject)
        request = predictive_control.srv.StaticCollisionObjectRequest()

        request.object_name = object_id
        request.object_id = object_id

        request.file_name = object_id

        # set object position and orientation
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time().now()

        # position
        pose.pose.position.x = -0.53 # half of dimension of box(0.20) + some distance
        pose.pose.position.y = 0.53
        pose.pose.position.z = 0.00 # half of dimension of box(0.40)

        # object orientation
        pose.pose.orientation.w = 0.0
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


def remove_environment(object_id):
    rospy.loginfo("Calling static object service ... ")
    rospy.wait_for_service("/arm/predictive_control/StaticCollision/remove_static_object")
    rospy.loginfo("Now all services are available ... ")

    try:
        client = rospy.ServiceProxy("/arm/predictive_control/StaticCollision/remove_static_object",
                                    predictive_control.srv.StaticCollisionObject)
        request = predictive_control.srv.StaticCollisionObjectRequest()

        # Note: problem with removing cylindrical objects
        request.object_name = object_id
        request.object_id = object_id

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

    object_id = "box"
    add_environment_from_file(object_id=object_id)
    #add_environment(object_id)
    #rospy.sleep(3.0)
    #remove_environment(object_id)