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

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

import pandas
import argparse
import pandas as pd
import time

import thread
import threading

class moveActionClient:
    def __init__(self):
        print ('\033[94m' + " ----- Start extraction and store trajectory script... ----- " + '\033[0m')
        self.traj_data = None
        self.poses_to_export = []
        self.pose_plus_quat = []
        self.base_frame = ""
        self.end_effector_frame = ""
        self.file_path = ""
        self.object_name = "pose_0"
        self.reach_goal = False
        self.runs = 0
        self.start_random = 0
        self.end_random = 0

        self.getTransformListener()

    def getTransformListener(self):
        with threading.Lock():
            self.listener = tf.TransformListener(True, rospy.Duration(40.0))

    def argumentParser(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--folder', '-f', help='file path where to store .csv file')
        parser.add_argument('--base_frame', '-bf', default='base_link', help='base link of robot')
        parser.add_argument('--eef_frame', '-ef', default='gripper', help='End-Effector Frame to track in cartesian space')
        parser.add_argument('--runs', '-r', default='10', help='iteration to runs script')
        parser.add_argument('--start_random', '-sr', default='1', help='starting of random number')
        parser.add_argument('--end_random', '-er', default='8', help='end of random number')
        args = parser.parse_args()

        self.base_frame = args.base_frame
        self.end_effector_frame = args.eef_frame
        self.runs = int(args.runs)
        self.start_random = int(args.start_random)
        self.end_random = int(args.end_random)

        if args.eef_frame is None:
            if rospy.get_param('/arm/chain_tip_link', default=''):
                self.end_effector_frame = rospy.get_param('/arm/chain_tip_link', default='')
            else:
                print('End effector not set... Defining as default arm_7_link ')
                self.end_effector_frame = 'arm_7_link'

        if args.base_frame is None:
            if rospy.get_param('/arm/chain_base_link', default=''):
                self.base_frame = rospy.get_param('/arm/chain_base_link', default='')
            else:
                print('Base frame not set... Defining as default arm_base_link ')
                self.base_frame = 'arm_base_link'

        # path of file to store output
        self.file_path = args.folder


    def move_to_pregasping_pose(self, object_list , object_name):
        data_info_obj = ReadDataFromList(item_list=object_list, object_name=object_name)
        [object_position, object_orientation] = data_info_obj.getObjectInfo()

        return self.move_action_client(object_position=object_position, object_orientation=object_orientation,object_name=object_name)

    def getTrajectoryCB(self, msg):
        #print ('\033[1m' + '\033[92m' + "######### " + " Got a new STOMP trajectory" + " ########### " + '\033[0m')
        rospy.sleep(0.5)
        self.traj_data = msg
        self.robotCurrentPose()

    def extractDataFromMarker(self, traj_data):
        px = []; py = []; pz = []
        for way_point in traj_data.pose.position:
            px.append(way_point.x)
            py.append(way_point.y)
            pz.append(way_point.z)
        return px, py, pz

    def getCurrentPoseOfeefFrame(self, marker_array):
        #for marker in marker_array.markers:
            #pose_plus_quat = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
            #                  marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z]
            #return pose_plus_quat
        print len(marker_array.markers)
        print marker_array.markers.size()
        last_element = len(marker_array.markers)-1
        self.pose_plus_quat = [marker_array.markers[last_element].pose.position.x, marker_array.markers[last_element].pose.position.y, marker_array.markers[last_element].pose.position.z,
                               marker_array.markers[last_element].pose.orientation.x, marker_array.markers[last_element].pose.orientation.y, marker_array.markers[last_element].pose.orientation.z,
                               marker_array.markers[last_element].pose.orientation.w]

    # get the current position of the robot
    def robotCurrentPose(self):
        t = rospy.Time(0)
        print self.object_name
        self.listener.waitForTransform(self.object_name, self.end_effector_frame, t, rospy.Duration(10))
        (trans, rot) = self.listener.lookupTransform(self.object_name, self.end_effector_frame, t)
        cartesian_error = [trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]]
        print cartesian_error
        self.reach_goal = self.checkInfitisimalPose(cartesian_error=cartesian_error, max_tolerance=0.01)

#-----------------------------------------------------------------------------------------------------------------------
    def extractDataFromMarkerArray(self, marker_array):
        for marker in marker_array.markers:
            pose = list()
            pose_plus_quat = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                              marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z]
            self.poses_to_export.append(pose_plus_quat)

    def storeTrajectoryWithPlanSuccess(self):

        self.extractDataFromMarkerArray(self.traj_data)
        self.storeToCSV()

# -----------------------------------------------------------------------------------------------------------------------
    def storeToCSV(self):
        axis_name = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']
        object_name = self.object_name.replace('_','-')
        filename_with_path = self.file_path  + object_name + '_' + str(time.strftime("%d-%m-%Y-%H:%M:%S")) + str('.csv')
        # filename_with_path = "/home/bfb-ws/gstomp_ws/src/gstomp/gstomp_experiments/output_data/" + self.file_path + str('.csv')

        # print('\033[1m' + '\033[31m' + "############ " + " Exporting File " + "############## " + '\033[0m')
        print('\033[32m' + 'Exported to files: ' + str(filename_with_path) + '\033[0m')

        reference_frames = pd.DataFrame([[self.base_frame, self.end_effector_frame]],
                                        columns=['base frame', 'end effector frame'])
        all_poses = pd.DataFrame(self.poses_to_export, columns=axis_name)

        # with open(filename_with_path, 'w') as f_out:
        f_out = open(filename_with_path, 'w+')
        reference_frames.to_csv(f_out)
        all_poses.to_csv(f_out)

        # print(''.join(axis_name) + ' saved to ' + filename_with_path)


    def Run(self):

        # parser the data
        self.argumentParser()

        runs = self.runs #int(sys.argv[1])
        start_random = self.start_random #int(sys.argv[2])
        end_random = self.end_random #int(sys.argv[3])

        rospy.loginfo("Starting move client function calling ...")
        self.move_client = actionlib.SimpleActionClient('/arm/move_action', predictive_control.msg.moveAction)
        self.move_client.wait_for_server()

        rospy.Subscriber("/arm/pd_trajectory", MarkerArray, self.getTrajectoryCB)
        rospy.sleep(0.2)
        rospy.loginfo("All services and action server are available")

        item_list_obj = rospy.get_param("/queries")

        # create static frame of all object
        #self.createStaticFrame(obj_list=item_list_obj)


        for i in range(0, runs, 1):
            print ('\033[94m' + "Current no. of runs is " + str(i) + '\033[0m')
            grasp_object = "pose_" + str(i+1) #str(random.randint(start_random, end_random))
            print "object_name: ", grasp_object

            # move to pre grasping position and orientation
            self.object_name = "pose_0"
            pre_grasp_success = self.move_to_pregasping_pose(object_list=item_list_obj, object_name="pose_0")
            
            # block untill reach to goal pose
            while pre_grasp_success is False or self.reach_goal is False:
                print

            self.object_name = grasp_object

            # extract information of object
            data_info_obj = ReadDataFromList(item_list=item_list_obj, object_name=grasp_object)
            [object_position, object_orientation] = data_info_obj.getObjectInfo()

            success = self.move_action_client(object_position=object_position, object_orientation=object_orientation, object_name=grasp_object)

            self.reach_goal = False

            # block untill reach to goal pose
            while pre_grasp_success is False or self.reach_goal is False:
                print

            # store trajectory
            self.storeTrajectoryWithPlanSuccess()
            self.reach_goal = False
            #rospy.sleep(5)

            rospy.loginfo("====================================================================")
            rospy.loginfo("One object successfully picked and placed")
            rospy.loginfo("====================================================================")


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
            return True
        else:
            print "Action did not finish within timeout"
            return False

        #while state != 3:
        #    print
        #return


# ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Block call untill reach to goal position
    def checkArmReachToGoalPose(self, goal_pose, max_tolerance):
        print ('\033[1m' + '\033[92m' + "######### " + " Checking arm reach to goal position with correct orientation" + " ########### " + '\033[0m')

        print goal_pose
        current_pose = self.getCurrentPoseOfeefFrame(self.traj_data)
        print current_pose
        state = False
        while (not rospy.is_shutdown() and state is False):

            current_pose = self.getCurrentPoseOfeefFrame(self.traj_data)

            if abs(goal_pose[0] - current_pose[0] > max_tolerance):
                state = False
                continue
            elif abs(goal_pose[1] - current_pose[1] > max_tolerance):
                state = False
                continue
            elif abs(goal_pose[2] - current_pose[2] > max_tolerance):
                state = False
                continue
            elif abs(goal_pose[3] - current_pose[3] > max_tolerance):
                state = False
                continue
            elif abs(goal_pose[4] - current_pose[4] > max_tolerance):
                state = False
                continue
            elif abs(goal_pose[5] - current_pose[5] > max_tolerance):
                state = False
                continue
            elif abs(goal_pose[6] - current_pose[6] > max_tolerance):
                state = False
                continue
            else:
                state = True
                continue

        return state

    def checkInfitisimalPose(self, cartesian_error, max_tolerance):

        print ('\033[1m' + '\033[92m' + "### " + "checkInfitisimalPose" + "### " + '\033[0m')

        for error in cartesian_error:
            if error > max_tolerance:
                return False
        return True

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
        for i in range(0, len(obj_list)-1):
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_frame_list.append(static_transformStamped)

        # iterate every object position inside the list
        for i in range(0, len(obj_list)-1):
            static_frame_list[i].header.stamp = rospy.Time.now()
            static_frame_list[i].header.frame_id = "world"

            # get frame information
            frame_name = "pose_" + str(i + 1)
            static_frame_list[i].child_frame_id = frame_name
            data_info_obj = ReadDataFromList(item_list=obj_list, object_name=frame_name)
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
        for i in range(0, len(obj_list)-1):
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