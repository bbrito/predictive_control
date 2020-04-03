#!/usr/bin/env python
import time
import numpy as np
import rospy, rosnode
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
from lmpcc.srv import LMPCCReset, LMPCCResetRequest


class SimulationManager:
    def __init__(self, output_path_topic='predicted_path_marker'):
        self.other_agents_topic = rospy.get_param('~other_agents_topic', "/pedsim_visualizer/tracked_persons")
        self.robot_state_topic = rospy.get_param('~robot_state_topic', "/robot_state")
        self.reset_topic_ = rospy.get_param('~reset_topic', "reset")

        # State variables
        self.robot_state_ = Pose()
        self.other_pedestrians = []
        self.min_distance = 0.4

        # Statistic variables
        self.n_collisions = 0
        self.n_timeouts = 0
        self.n_success = 0
        self.n_trials = 0
        self.exp_time = time.time()
        self.n_simulations = 100
        self.timeout = 30 # seconds

        # ROS Subscribers
        rospy.Subscriber(self.other_agents_topic, TrackedPersons, self.other_agents_CB, queue_size=1)
        rospy.Subscriber(self.robot_state_topic, Pose, self.robot_state_CB, queue_size=1)

        # ROS Clients
        self.reset_client = rospy.ServiceProxy(self.reset_topic_,LMPCCReset)

    def robot_state_CB(self, msg):
        self.robot_state_ = msg

    def other_agents_CB(self, data):
        self.other_pedestrians = []
        self.n_pedestrians = len(data.tracks)
        for person_it in range(self.n_pedestrians):
            ped = TrackedPerson()
            ped.pose = data.tracks[person_it].pose.pose
            ped.track_id = data.tracks[person_it].track_id
            ped.twist = data.tracks[person_it].twist.twist
            self.other_pedestrians.append(ped)

        self.check_collision()

    def check_collision(self):
        distances = np.ones([self.n_pedestrians])
        for person_it in range(self.n_pedestrians):
            dist = np.linalg.norm(np.array([self.other_pedestrians[person_it].pose.position.x - self.robot_state_.position.x,
                                            self.other_pedestrians[person_it].pose.position.y - self.robot_state_.position.y]))
            distances[person_it] = dist

        if np.min(distances) < self.min_distance:
            rospy.loginfo("Number of collisions: " + str(self.n_collisions))
            self.n_collisions += 1
            self.n_trials += 1
            self.reset_planner()

    def check_is_over(self):
        # Check if simulation is over
        now = time.time()
        # Todo: adapt for more general cases
        if self.robot_state_.position.x > 18:
            rospy.loginfo("Number of successes: " + str(self.n_success))
            self.n_trials += 1
            self.n_success += 1
            self.reset_planner()
        if now - self.exp_time > self.timeout:
            rospy.loginfo("Number of timeouts: " + str(self.n_timeouts))
            self.n_timeouts += 1
            self.n_trials += 1
            self.reset_planner()
        if self.n_trials > self.n_simulations:
            self.print_statistics()
            self.end_simulation()
            return True

    def reset_planner(self):
        try:
            self.reset_client(LMPCCResetRequest())
        except rospy.ServiceException, e:
            print("Service call failed: %s" %e)
        rospy.sleep(1.0)
        self.exp_time = time.time()

    def print_statistics(self):
        rospy.loginfo("**************** Final Results ***********************")
        rospy.loginfo("Number of collisions: " + str(self.n_collisions))
        rospy.loginfo("Number of timeouts: " + str(self.n_timeouts))
        rospy.loginfo("Number of successes: " + str(self.n_success))
        rospy.loginfo("Number of trials: " + str(self.n_trials))

    def end_simulation(self):
        self.print_statistics()
        node_list = rosnode.get_node_names()
        rosnode.kill_nodes(node_list)

def main():
    rospy.init_node('simulation_manager')

    sim = SimulationManager()
    # Start experiment
    sim.reset_planner()
    start_time = time.time()
    while not rospy.is_shutdown():
        now = time.time()

        if now - start_time < 0.05:  # args.dt:

            # Check if simulation is over
            if sim.check_is_over():
                return

            rospy.sleep(0.05 - (now - start_time))
        else:
            print("not keeping up to rate")

        start_time = time.time()


if __name__ == '__main__':
    main()
