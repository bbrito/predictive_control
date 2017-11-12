
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/predictive_controller.h>


predictive_control_node::predictive_control_node()
{
	nh = ("predictive_control");
}

void predictive_control_node::read_predictive_parameters(predictive_config& new_param)
{
	//Chain_base and chain tip links, root frame
		if (!nh.getParam ("chain_base_link", new_param.base_link) )
		{
			ROS_WARN(" Parameter 'chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
		}

		if (!nh.getParam ("chain_tip_link", new_param.tip_link) )
		{
			ROS_WARN(" Parameter 'chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
		}

		if (!nh.getParam ("root_frame", new_param.root_frame) )
		{
			ROS_WARN(" Parameter 'root_frame' not set on %s node " , ros::this_node::getName().c_str());
		}

		//Get joint names
		if (!nh.getParam ("joint_names", new_param.jnts_name) )
		{
			ROS_WARN(" Parameter 'joint names' not set on %s node " , ros::this_node::getName().c_str());
		}

		new_param.dof = new_param.jnts_name.size();

		// Get debug info, using active_output
		nh.param("activate_output", new_param.activate_output, bool(false));

		// Get limit parameters and tolerance
		nh.param("min_position_limit", new_param.min_position_limit, double(-3.14));
		nh.param("max_position_limit", new_param.max_position_limit, double( 3.14));
		nh.param("min_velocity_limit", new_param.min_velocity_limit, double( 0.00));
		nh.param("max_velocity_limit", new_param.max_velocity_limit, double( 2.00));
		nh.param("desired_velocity", new_param.desired_velocity, double(1.5));
		nh.param("position_tolerance", new_param.position_tolerance, double(1.0));
		nh.param("velocity_tolerance", new_param.velocity_tolerance, double(0.01));

		// Get discretization_steps
		nh.param("min_discretization_steps", new_param.min_discretization_steps, int(10));
		nh.param("max_discretization_steps", new_param.max_discretization_steps, int(20));
		nh.param("discretization_steps", new_param.discretization_steps, int(15));

}


void predictive_control_node::main_predictive_control()
{
	// Check ROS node start correctly.
	if (ros::ok())
	{
		predictive_config config;
		read_predictive_parameters(config);

	}

}
