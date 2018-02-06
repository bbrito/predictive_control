#!/usr/bin/env python

import sys
import math
from copy import deepcopy

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from predictive_control.msg import *
from predictive_control.srv import *


class StaticObstacles:
    def __init__(self):
        pass


    def add_static_obstacles(self, object_id):

        rospy.loginfo("Calling static obstacle service ... ")
        rospy.wait_for_service("/arm/pd_control/add_static_obstacles")
        rospy.loginfo("Now all services are available ... ")

        print ('\033[94m' + " ----- Add static collision obstracles... ----- " + '\033[0m')
        client = rospy.ServiceProxy("/arm/pd_control/add_static_obstacles", predictive_control.srv.StaticObstacle)
        request = predictive_control.srv.StaticObstacleRequest()
        response = predictive_control.srv.StaticObstacleResponse()

        co = moveit_msgs.msg.CollisionObject()

        co.id = object_id
        co.header.frame_id = "world"
        co.operation = moveit_msgs.msg.CollisionObject.ADD

        primitive = shape_msgs.msg.SolidPrimitive()
        primitive.type = shape_msgs.msg.SolidPrimitive.SPHERE
        primitive.dimensions = [0.07]  # extent x, y, z
        co.primitives.append(primitive)

        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.4
        pose.position.y =  0.3
        pose.position.z =  0.8
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)

        # fill request of static collision obstacle
        request.static_collision_object = co

        # call service to add static obstacle into environments
        success = client(request)

        if (success):
            rospy.loginfo("Successfully added " + object_id + " into environment")

        else:
            rospy.logerr("Failed to add " + object_id + " into environment")

        print (response.success)
        print (response.message)

    def delete_static_obstacles(self, object_id):

        rospy.loginfo("Calling static obstacle service ... ")
        rospy.wait_for_service("/arm/pd_control/delete_static_obstacles")
        rospy.loginfo("Now all services are available ... ")

        print ('\033[94m' + " ----- Delete static collision obstracles... ----- " + '\033[0m')
        client = rospy.ServiceProxy("/arm/pd_control/delete_static_obstacles", predictive_control.srv.StaticObstacle)
        request = predictive_control.srv.StaticObstacleRequest()
        response = predictive_control.srv.StaticObstacleResponse()

        co = moveit_msgs.msg.CollisionObject()

        co.id = object_id
        co.header.frame_id = "world"
        co.operation = moveit_msgs.msg.CollisionObject.REMOVE

        primitive = shape_msgs.msg.SolidPrimitive()
        primitive.type = shape_msgs.msg.SolidPrimitive.SPHERE
        primitive.dimensions = [0.07]  # extent x, y, z
        co.primitives.append(primitive)

        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.4
        pose.position.y =  0.3
        pose.position.z =  0.8
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)

        # fill request of static collision obstacle
        request.static_collision_object = co

        # call service to add static obstacle into environments
        success = client(request)

        if (success):
            rospy.loginfo("Successfully added " + object_id + " into environment")

        else:
            rospy.logerr("Failed to add " + object_id + " into environment")

        print (response.success)
        print (response.message)

    def add_environment_from_file(self, object_id):
        rospy.loginfo("Calling static object service ... ")
        rospy.wait_for_service("/arm/pd_control/add_static_obstacles")
        rospy.loginfo("Now all services are available ... ")

        try:
            client = rospy.ServiceProxy("/arm/pd_control/add_static_obstacles", predictive_control.srv.StaticObstacle)
            request = predictive_control.srv.StaticObstacleRequest()
            response = predictive_control.srv.StaticObstacleResponse()

            co = moveit_msgs.msg.CollisionObject()

            # at this moment, object_id same as frame of object
            co.id = object_id

            request.static_collision_object = co
            #request.file_name = "bookshelves"
            request.file_name = "box"

            # call service to add static object into environments
            success = client(request)

            if (success):
                rospy.loginfo("Successfully added " + object_id + " into environment")

            else:
                rospy.logerr("Failed to add " + object_id + " into environment")

            print (response.success)
            print response.message

        except rospy.ServiceException as exc:
            rospy.logerr(" Service did not process request " + str(exc))


if __name__ == '__main__':
    rospy.init_node("pd_static_obstacles_service")

    object_id = "box" #"bookshelves"
    OBJECT = StaticObstacles()
    OBJECT.add_environment_from_file(object_id=object_id)
    #OBJECT.add_static_obstacles(object_id=object_id)
    #rospy.sleep(15.0)
    #OBJECT.delete_static_obstacles(object_id=object_id)
