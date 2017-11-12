
//This file containts read parameter from server, callback, call class objects, control all class

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

// std includes
#include <iostream>
#include <string>

// boost includes
#include <boost/shared_ptr.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <predictive_control/predictive_trajectory_generator.h>

class predictive_control_node
{
private:

	ros::NodeHandle nh;

public:

	predictive_control_node();
	~predictive_control_node();

	void read_predictive_parameters(predictive_config& new_param);

	void main_predictive_control(void);

};

