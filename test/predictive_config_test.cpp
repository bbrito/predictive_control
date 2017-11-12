
#include <ros/ros.h>

#include <predictive_control/predictive_trajectory_generator.h>

void read_predictive_parameters(predictive_config& new_param, ros::NodeHandle& nh)
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


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, ros::this_node::getName());
		ros::NodeHandle node_handler;

		if (node_handler.hasParam("/robot_description"))
		{
			predictive_config config;
			read_predictive_parameters(config, node_handler);
			config.update_config_parameters(config);
			config.print_data_member();

			std::cout<<std::endl;

			// Check position tolerance violation function
			double current_position = 3.00;
			ROS_INFO_STREAM("Test position tolerance violation function: " << std::boolalpha << config.check_position_tolerance_violation(current_position));

			double current_position1 = 3.04;
			ROS_INFO_STREAM("Test position tolerance violation function: " << std::boolalpha << config.check_position_tolerance_violation(current_position1));

			double violate_current_position = 3.26;
			ROS_INFO_STREAM("Test position tolerance violation function: " << std::boolalpha << config.check_position_tolerance_violation(violate_current_position));

			double violate_current_position1 = -3.85;
			ROS_INFO_STREAM("Test position tolerance violation function: " << std::boolalpha << config.check_position_tolerance_violation(violate_current_position1));

			// Check enforce position limit function
			double enforce_current_position = 3.00, corrected_position;
			config.enforce_position_limit( enforce_current_position, corrected_position);
			ROS_INFO("Test enforce position limit function with current position %f, enforced position %f", enforce_current_position, corrected_position );

			double enforce_current_position1 = 3.26, corrected_position1;
			config.enforce_position_limit( enforce_current_position1, corrected_position1);
			ROS_INFO("Test enforce position limit function with current position %f, enforced position %f", enforce_current_position1, corrected_position1 );

			double enforce_current_position2 = -3.85, corrected_position2;
			config.enforce_position_limit( enforce_current_position2, corrected_position2);
			ROS_INFO("Test enforce position limit function with current position %f, enforced position %f", enforce_current_position2, corrected_position2 );

			std::cout<< "\n ------------------------------------------------------- \n"<<std::endl;

			// Check velocity tolerance violation function
			double current_velocity = 1.00;
			ROS_INFO_STREAM("Test velocity tolerance violation function: " << std::boolalpha << config.check_velocity_tolerance_violation(current_velocity));

			double violate_current_velocity = 2.85;
			ROS_INFO_STREAM("Test position tolerance violation function: " << std::boolalpha << config.check_velocity_tolerance_violation(violate_current_velocity));

			double violate_current_velocity1 = -0.85;
			ROS_INFO_STREAM("Test position tolerance violation function: " << std::boolalpha << config.check_velocity_tolerance_violation(violate_current_velocity1));

			// Check enforce velocity limit function
			double enforce_current_velocity = 1.00, corrected_velocity;
			config.enforce_velocity_limit( enforce_current_velocity, corrected_position);
			ROS_INFO("Test enforce velocity limit function with current velocity %f, enforced velocity %f", enforce_current_velocity, corrected_velocity );

			double enforce_current_velocity1 = 2.85, corrected_velocity1;
			config.enforce_velocity_limit( enforce_current_position1, corrected_velocity1);
			ROS_INFO("Test enforce velocity limit function with current velocity %f, enforced velocity %f", enforce_current_velocity1, corrected_velocity1 );

			double enforce_current_velocity2 = -0.85, corrected_velocity2;
			config.enforce_velocity_limit( enforce_current_velocity2, corrected_velocity2);
			ROS_INFO("Test enforce velocity limit function with current velocity %f, enforced velocity %f", enforce_current_velocity2, corrected_velocity2 );

			std::cout<< "\n ------------------------------------------------------- \n"<<std::endl;

			// Check choose_discretization_steps function
			config.discretization_steps = 0.0;
			config.max_discretization_steps = 15;
			config.choose_discretization_steps();
			ROS_INFO("Discretization step choose using arithmatic mean: %u", config.discretization_steps);

		}

		else
		{
			ROS_ERROR("Robot_description not available");
			exit(1);
		}

	} catch (ros::Exception& e)

	{
		ROS_ERROR("%s", e.what());
		exit(1);
	}

return 0;
}
