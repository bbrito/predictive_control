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

class moveActionClient:
    def __init__(self):
        pass


    def move_to_pregasping_pose(self, object_name, object_list):
        data_info_obj = ReadDataFromList(item_list=object_list.items_list, object_name=object_name)
        [object_position, object_orientation] = data_info_obj.getObjectInfo()

        self.move_action_client(object_position=object_position, object_orientation=object_orientation,object_name=object_name)

    def Run(self):
        runs = int(sys.argv[1])
        start_random = int(sys.argv[2])
        end_random = int(sys.argv[3])

        rospy.loginfo("Starting move client function calling ...")
        self.move_client = actionlib.SimpleActionClient('/arm/move_action', predictive_control.msg.moveAction)
        self.move_client.wait_for_server()

        object = ReadDataFromServer("queries")

        # create static frame of all object
        #self.createStaticFrame(obj_list=object)

        #rospy.spin()

        for i in range(0, runs, 1):
            print ('\033[94m' + "Current no. of runs is " + str(i) + '\033[0m')
            grasp_object = "pose_" + str(i+1) #str(random.randint(start_random, end_random))
            print "object_name: ", grasp_object

            # move to pregraping position
            self.move_to_pregasping_pose(object_name="pose_0", object_list=object)

            # extract information of object
            data_info_obj = ReadDataFromList(item_list=object.items_list, object_name=grasp_object)
            [object_position, object_orientation] = data_info_obj.getObjectInfo()

            self.move_action_client(object_position=object_position, object_orientation=object_orientation, object_name=grasp_object)

    """
        move action client function test different object position and orientation
    """
    def move_action_client(self, object_position, object_orientation, object_name):

        goal = predictive_control.msg.moveGoal()
        goal.target_frame_id = object_name

        # header
        goal.target_endeffector_pose.header.frame_id = "world"
        goal.target_endeffector_pose.header.stamp = rospy.Time.now()

        # position
        goal.target_endeffector_pose.pose.position.x = object_position[0]
        goal.target_endeffector_pose.pose.position.y = object_position[1]
        goal.target_endeffector_pose.pose.position.z = object_position[2]

        # orientation, -0.128, -0.628, -0.034, 0.767
        goal.target_endeffector_pose.pose.orientation.x = object_orientation[0] #0.00120
        goal.target_endeffector_pose.pose.orientation.y = object_orientation[1] #0.005199
        goal.target_endeffector_pose.pose.orientation.z = object_orientation[2] #0.00450
        goal.target_endeffector_pose.pose.orientation.w = object_orientation[3]  # 0.99997

        # Sends the goal to the action server
        rospy.loginfo("Now sending goal")
        self.move_client.send_goal(goal)

        # Waits for the server to finish performing the action
        finished_before_timeout = self.move_client.wait_for_result(rospy.Duration(50, 0))

        if finished_before_timeout:
            state = self.move_client.get_state()
            print "Action finished: %s"%state
        else:
            print "Action did not finish within timeout"

        while state != 3:
            print
        return

    # -----------------------------------------------------------------------------------------------------------------------
    """
        @description: create static frame of all list of object 
        @param: obj_list: list of object
        @ouput: view tf frame of given/list objects
    """

    def createStaticFrame(self, obj_list):

        broadcaster = tf2_ros.StaticTransformBroadcaster()

        # list of static frame object that want to broadcast
        static_frame_list = []
        for i in range(0, len(obj_list.items_list)):
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_frame_list.append(static_transformStamped)

        # iterate every object position inside the list
        for i in range(0, len(obj_list.items_list)):
            static_frame_list[i].header.stamp = rospy.Time.now()
            static_frame_list[i].header.frame_id = "world"

            # get frame information
            frame_name = "pose_" + str(i + 1)
            static_frame_list[i].child_frame_id = frame_name
            data_info_obj = ReadDataFromList(item_list=obj_list.items_list, object_name=frame_name)
            [obj_pose, obj_orient] = data_info_obj.getObjectInfo()

            # create frame
            static_frame_list[i].transform.translation.x = obj_pose[0]
            static_frame_list[i].transform.translation.y = obj_pose[1]
            static_frame_list[i].transform.translation.z = obj_pose[2]

            norm_orient = LA.norm(obj_orient)
            # print norm_orient

            static_frame_list[i].transform.rotation.x = float(obj_orient[0] / norm_orient)
            static_frame_list[i].transform.rotation.y = float(obj_orient[1] / norm_orient)
            static_frame_list[i].transform.rotation.z = float(obj_orient[2] / norm_orient)
            static_frame_list[i].transform.rotation.w = float(obj_orient[3] / norm_orient)

        # list of frames that have to broadcast
        brd_list = []
        for i in range(0, len(obj_list.items_list)):
            brd_list.append(static_frame_list[i])

        # broadcast all frame together
        broadcaster.sendTransform(brd_list)

########################################################################################################################
class Utilities:
    """"
        Set pre-define value or trajectory for simulation
    """
    def __init__(self):
        self.joint_states = [0., 0., 0., 0., 0., 0.];
        rospy.Subscriber("/arm/joint_states", JointState, self.jointStateCallback)

        self.arm_pre_grasp = [-1.2194000015606932, -0.7560003329475764, 2.645999934451746, -1.2073996232774817, -0.1763999501594018, -1.058399962604783, 2.578178506240647e-07]
        self.trajectory_pregrasp_to_drop = [
            [-0.4189751755093116, -0.7189265831828818, -1.5779628932878642, -4.057396345572524, -1.6429464228615416, 0.0],
            [1.2668177979117803, -0.8545146464803002, -1.467468339660507, -4.056883439921334, -1.6429020332924926, 0.0],
            [1.8946927589099885, -1.2498882108994205, -1.4212811368752756, -3.621294495077807, -1.642768864585344, 0.0],
            [2.257686430482939, -1.8189895672673213, -1.818749940086222, -3.621461152503031, -1.6427599866715346, 0.0],
            [2.2574481585254746, -1.8189895672673213, -1.8189896245843291, -3.9631237154760623, -1.6427599866715346, 0.0],
        ]

        self.trajectory_drop_to_pregrasp = [
            [2.2574481585254746, -1.8189895672673213, -1.8189896245843291, -3.9631237154760623, -1.6427599866715346, 0.0],
            [2.257686430482939, -1.8189895672673213, -1.818749940086222, -3.621461152503031, -1.6427599866715346, 0.0],
            [1.8946927589099885, -1.2498882108994205, -1.4212811368752756, -3.621294495077807, -1.642768864585344, 0.0],
            [1.2668177979117803, -0.8545146464803002, -1.467468339660507, -4.056883439921334, -1.6429020332924926, 0.0],
            [-0.4189751755093116, -0.7189265831828818, -1.5779628932878642, -4.057396345572524, -1.6429464228615416, 0.0],
            self.arm_pre_grasp
        ]

# receives the joint states
    def jointStateCallback(self, msg):
        for i in range(0, len(msg.position)):
            if "arm_shoulder_pan_joint" == msg.name[i]:
                self.joint_states[0] = msg.position[i]
            elif "arm_shoulder_lift_joint" == msg.name[i]:
                self.joint_states[1] = msg.position[i]
            elif "arm_elbow_joint" == msg.name[i]:
                self.joint_states[2] = msg.position[i]
            elif "arm_wrist_1_joint" == msg.name[i]:
                self.joint_states[3] = msg.position[i]
            elif "arm_wrist_2_joint" == msg.name[i]:
                self.joint_states[4] = msg.position[i]
            elif "arm_wrist_3_joint" == msg.name[i]:
                self.joint_states[5] = msg.position[i]


class ReadDataFromServer:
    """
        Read item list from parameter server (object_location.yaml)
    """

    # Default constructor of ReadDataFromServer Class
    def __init__(self):
      pass

    # Read data from parameter server and Store it into list
    #@param param_name - name of parameter
    def __init__(self, param_name):
        self.items_list = []

        if rospy.has_param("/"+param_name):
            self.items_list = rospy.get_param("/"+param_name)
        else:
            rospy.logerr("ReadDataFromFile --> given param_name is not available on parameter server")

########################################################################################################################

class ReadDataFromList:
    """
        Get position, orientation, box_size and shelf_id where object is located from item list
    """

    # Default constructor of ReadDataFromList Class
    def __init__(self):
      pass

    # User made constructor
    # @param item_list - all items info store in the list
    # @param object_name - name of object which info want to retrieve
    def __init__(self, item_list, object_name):
       # self.products_name = self.getProductName(items_list=item_list, object_name=object_name)
        self.object_pose = self.getProductPosition(items_list=item_list, object_name=object_name)
        self.object_rpy = self.getProductOrientation(items_list=item_list, object_name=object_name)

    # @return position, rpy, box_size, shelf_id of object
    def getObjectInfo(self):
        #return self.object_pose, self.object_rpy, self.box_size, self.shelf_id
        return self.object_pose, self.object_rpy

    def getProductName(self, items_list, object_name):
        for item_name in items_list:
            object_name.append(item_name)

    def getProductPosition(self, items_list, object_name):
        data_product = items_list[object_name]
        one_pose_list = data_product['position']
        return one_pose_list

    def getProductOrientation(self, items_list, object_name):
        data_product = items_list[object_name]
        one_orient_list = data_product['orientation']
        return one_orient_list

############################################## END OF FILE #############################################################


if __name__ == '__main__':
    rospy.init_node("pd_move_action_client")
    SCRIPT = moveActionClient()
    SCRIPT.Run()