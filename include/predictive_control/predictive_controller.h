
#ifndef PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H
#define PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H

//This file containts read parameter from server, callback, call class objects, control all class

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>

// std includes
#include <iostream>
#include <string>
#include <assert.h>

// boost includes
#include <boost/shared_ptr.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// predicitve includes
#include <predictive_control/predictive_trajectory_generator.h>

class predictive_control_node
{
private:

	ros::NodeHandle nh;
	ros::Subscriber joint_state_sub;

	predictive_config new_config;

	std::vector<double> current_position;
	std::vector<double> current_velocity;

	void spin_node();

public:

	predictive_control_node();
	~predictive_control_node();

	// Read and initialize data member of predictive_config class from parameter server
	void read_predictive_parameters(predictive_config& new_param);

	// Get current position and velocity at each joint
	void joint_state_callBack(const sensor_msgs::JointState::ConstPtr& msg);

	void main_predictive_control(void);

};

#endif
