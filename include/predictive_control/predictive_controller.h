
#ifndef PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H
#define PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H

//This file containts read parameter from server, callback, call class objects, control all class

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// std includes
#include <assert.h>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <iostream>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// predicitve includes
#include <predictive_control/predictive_trajectory_generator.h>
#include <predictive_control/kinematic_calculations.h>


class predictive_control_node
{
private:

	ros::NodeHandle nh;
	ros::Subscriber joint_state_sub;
	ros::Publisher joint_velocity_pub;

	// Timer
    double update_rate_;
    ros::Timer timer_;
    unsigned int dof;

	// predictive configuration parameter
	predictive_config new_config;

	// current position and velocity from joint state callback
	std::vector<double> current_position;
	std::vector<double> current_velocity;

	// type of variable used to publish joint velocity
	std_msgs::Float64MultiArray joint_velocity_data;

	// kinematic calculation
	boost::shared_ptr<Kinematic_calculations> kinematic_solver_;

	// trajectory generator
	boost::shared_ptr<pd_frame_tracker> pd_frame_tracker_;

	void spin_node();

	void run_node(const ros::TimerEvent& event);

	void convert_std_To_Eigen_vector(const std::vector<double>& std_vec, Eigen::VectorXd& eigen_vec);
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
