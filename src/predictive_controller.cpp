
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/predictive_controller.h>


predictive_control_node::predictive_control_node()
{
	//nh = "predictive_control_node";

	nh = ros::this_node::getName();

	read_predictive_parameters(new_config);

	if (new_config.activate_output) new_config.print_data_member();

}

predictive_control_node::~predictive_control_node()
{
	current_position.clear();
	current_velocity.clear();
}

void predictive_control_node::spin_node()
{
	ROS_INFO(" Spinning Node");
	ros::spin();
}

void predictive_control_node::read_predictive_parameters(predictive_config& new_param)
{
	//Chain_base and chain tip links, root frame
	if (!nh.getParam ("/chain_base_link", new_param.base_link) )
	{
		ROS_WARN(" Parameter 'Chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/chain_tip_link", new_param.tip_link) )
	{
		ROS_WARN(" Parameter 'Chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/root_frame", new_param.root_frame) )
	{
		ROS_WARN(" Parameter 'Root_frame' not set on %s node " , ros::this_node::getName().c_str());
	}

	//Get joint names
	if (!nh.getParam ("/joint_names", new_param.jnts_name) )
	{
		ROS_WARN(" Parameter 'Joint names' not set on %s node " , ros::this_node::getName().c_str());
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

	//current_position.resize(new_param.dof, 0.0);
	//current_velocity.resize(new_param.dof, 0.0);
	// Initialize ROS interfaces
	joint_state_sub = nh.subscribe("/joint_states", 1, &predictive_control_node::joint_state_callBack, this);

}

void predictive_control_node::joint_state_callBack(const sensor_msgs::JointState::ConstPtr& msg)
{
	// Make sure both members are empty as used push_back
	current_position.clear();	current_velocity.clear();
	int count = 0;
	//assert( current_position.size() != current_velocity.size() );
	//ROS_ERROR("Position and velocity size is not same ... joint_state_callBack");

	ROS_DEBUG("Call joint_state_callBack function ... ");

	for (unsigned int i = 0; i < new_config.dof; ++i)
	{
		for (unsigned int j = 0; j < msg->name.size(); ++j)
		{
			if ( std::strcmp( msg->name[j].c_str(), new_config.jnts_name[i].c_str()) == 0 )
			{
				//current_position[i] = msg->position[j];
				current_position.push_back( msg->position[j] );
				current_velocity.push_back( msg->velocity[j] );
				count++;
			}
		}
	}

	ros::Duration(0.1).sleep();

	if (count != new_config.jnts_name.size())
	{
		ROS_WARN(" Joint names are mismatched, need to check yaml file or code ... joint_state_callBack ");
	}

	else
	{
		// Output is active
		if (new_config.activate_output)
		{
			std::cout << "Current joint position: [";
			for_each(current_position.begin(), current_position.end(), [](double& p)
														{ std::cout<< std::setprecision(5) << p << ", " ; }
			);
			std::cout<<"]"<<std::endl;

			std::cout << "Current joint velocity: [";
			for_each(current_velocity.begin(), current_velocity.end(), [](double& v)
														{ std::cout<< std::setprecision(5) << v << ", " ; }
			);
			std::cout<<"]"<<std::endl;
		}
	}
}


void predictive_control_node::main_predictive_control()
{
	// Check ROS node start correctly.
	if (ros::ok())
	{
		spin_node();

	}

}
