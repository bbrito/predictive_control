
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

// kdl includes
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>


class predictive_config
{

protected:

	ros::NodeHandle nh;

	// Kinematic solver config varible
	uint8_t dof;
	std::string base_link;
	std::string tip_link;
	std::string root_frame;
	std::vector<std::string> jnts_name;

	double limits_tolerance;


public:

	predictive_config();
	~predictive_config();

	bool initializeParam(void);


};

