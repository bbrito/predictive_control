
//This file containts cost function intsert in to generated trajectory.

#ifndef PREDICTIVE_CONTROL_PREDICITVE_TRAJECTORY_GENERATOR_H
#define PREDICTIVE_CONTROL_PREDICITVE_TRAJECTORY_GENERATOR_H

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

// std includes
#include <iostream>
#include <string>
#include <algorithm>

// boost includes
#include <boost/shared_ptr.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

class predictive_config
{

public:

	// Data members
	//------------------------------------------------------------
	ros::NodeHandle nh;

	// Dubug info
	bool activate_output;

	// Kinematic solver config varible
	unsigned int dof;
	std::string base_link;
	std::string tip_link;
	std::string root_frame;
	std::string target_frame;
	std::vector<std::string> jnts_name;

	// Minimum and maximum position limits
	double min_position_limit;
	double max_position_limit;
	double min_velocity_limit;
	double max_velocity_limit;
	double desired_velocity;
	double position_tolerance;
	double velocity_tolerance;
	bool position_tolerance_violate;
	bool velocity_tolerance_violate;

	// Discretization_steps, used in acado
	int min_discretization_steps;
	int max_discretization_steps;
	int discretization_steps;

	// Function members
	//----------------------------------------------------------------
	predictive_config();
	~predictive_config();

	void update_config_parameters(predictive_config& new_param);

	// Check position, velocity limit, enforce to keep in that limit
	//void enforce_min_position_limit(double current_position);
	//void enforce_max_position_limit(double current_position);
	void enforce_position_limit(double current_position, double& corrected_position);
	bool check_position_tolerance_violation(double current_position);
	//void enforce_min_velocity_limit(double current_velocity);
	//void enforce_max_velocity_limit(double current_velocity);
	void enforce_velocity_limit(double current_velocity, double& corrected_velocity);
	bool check_velocity_tolerance_violation(double current_velocity);

	void choose_discretization_steps();
	void print_data_member();

};


#endif
