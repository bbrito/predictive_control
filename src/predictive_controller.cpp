
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/predictive_controller.h>


predictive_control_node::predictive_control_node()
{
	//nh = "predictive_control_node";

	//nh = ros::this_node::getName();

}

predictive_control_node::~predictive_control_node()
{
	current_position.clear();
	current_velocity.clear();
}

void predictive_control_node::spin_node()
{
	ROS_INFO(" Predictive control node is running, now it's 'Spinning Node'");
	ros::spin();
}

void predictive_control_node::read_predictive_parameters(predictive_config& new_param)
{

	//ros::NodeHandle nh = std::string("arm");

	// Get chain_base and chain tip links, root frame, traget frame
	if (!nh.getParam ("/chain_base_link", new_param.base_link) )
	{
		ROS_WARN(" Parameter 'chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/chain_tip_link", new_param.tip_link) )
	{
		ROS_WARN(" Parameter 'chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/root_frame", new_param.root_frame) )
	{
		ROS_WARN(" Parameter 'root_frame' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/target_frame", new_param.target_frame) )
	{
		ROS_WARN(" Parameter 'target_frame' not set on %s node " , ros::this_node::getName().c_str());
	}

	//Get joint names
	if (!nh.getParam ("/joint_names", new_param.jnts_name) )
	{
		ROS_WARN(" Parameter 'joint names' not set on %s node " , ros::this_node::getName().c_str());
	}

	new_param.dof = new_param.jnts_name.size();

	// Get debug info, using active_output
	nh.param("/activate_output", new_param.activate_output, bool(false));

	nh.param("/update_rate", new_param.update_rate, double(50.0));	//hz

	// Get limit parameters and tolerance
	nh.param("/min_position_limit", new_param.min_position_limit, double(-3.14));
	nh.param("/max_position_limit", new_param.max_position_limit, double( 3.14));
	nh.param("/min_velocity_limit", new_param.min_velocity_limit, double( 0.00));
	nh.param("/max_velocity_limit", new_param.max_velocity_limit, double( 2.00));
	nh.param("/desired_velocity", new_param.desired_velocity, double(1.5));
	nh.param("/position_tolerance", new_param.position_tolerance, double(1.0));
	nh.param("/velocity_tolerance", new_param.velocity_tolerance, double(0.01));

	// Get discretization_steps
	nh.param("/min_discretization_steps", new_param.min_discretization_steps, int(10));
	nh.param("/max_discretization_steps", new_param.max_discretization_steps, int(20));
	nh.param("/discretization_steps", new_param.discretization_steps, int(15));

	ROS_INFO("predictive_control_node:read_predictive_parameters ... is finished");
	//current_position.resize(new_param.dof, 0.0);
	//current_velocity.resize(new_param.dof, 0.0);

}

void predictive_control_node::joint_state_callBack(const sensor_msgs::JointState::ConstPtr& msg)
{
	// Make sure both members are empty before use as used push_back command
	current_position.clear();	current_velocity.clear();
	int count = 0;
	//assert( current_position.size() != current_velocity.size() );
	//ROS_ERROR("Position and velocity size is not same ... joint_state_callBack");

	ROS_DEBUG("Call joint_state_callBack function ... ");

	for (unsigned int i = 0; i < dof; ++i)
	{
		for (unsigned int j = 0; j < msg->name.size(); ++j)
		{
			if ( std::strcmp( msg->name[j].c_str(), new_config.jnts_name[i].c_str()) == 0 )
			{
				//current_position[i] = msg->position[j];
				current_position.push_back( msg->position[j] );
				current_velocity.push_back( msg->velocity[j] );
				count++;
				break;
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
		// Output is active, than only print joint state values
		if (new_config.activate_output)
		{
			std::cout<< "\n --------------------------------------------- \n";
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
			std::cout<< "\n --------------------------------------------- \n";
		}
	}
}

void predictive_control_node::main_predictive_control()
{
	// Check ROS node start correctly.
	if (ros::ok())
	{
		// read data from parameter server
		read_predictive_parameters(new_config);
		if (new_config.activate_output) new_config.print_data_member();

		// update configuration parameter
		new_config.update_config_parameters(new_config);

		dof = new_config.dof;
		current_position.resize(dof,0.0);
		current_velocity.resize(dof,0.0);
		current_gripper_pose = geometry_msgs::PoseStamped();
		target_gripper_pose = geometry_msgs::PoseStamped();
		const int m = 6, n = dof;
		Jacobian_Mat.resize(m,n);
		Jacobian_Mat.setIdentity();

		// initialize helper classes
		kinematic_solver_.reset(new Kinematic_calculations());
		kinematic_solver_->initialize();
		pd_frame_tracker_.reset(new pd_frame_tracker());
		pd_frame_tracker_->initialization(new_config);

		// Initialize ROS interfaces
		joint_state_sub = nh.subscribe("joint_states", 1, &predictive_control_node::joint_state_callBack, this);
		joint_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);

		// initialize publishing data members
		joint_velocity_data.data.resize(dof, 0.0);
		for (int i = 0u; i < dof && !current_velocity.empty(); ++i)
		{
			joint_velocity_data.data[i] = current_velocity.at(i);
		}

	    timer_ = nh.createTimer(ros::Duration(1/new_config.update_rate), &predictive_control_node::run_node, this);
	    timer_.start();

	    ROS_WARN("%s INTIALIZED!!", ros::this_node::getName().c_str());

		//spin_node();
	}
}

void predictive_control_node::run_node(const ros::TimerEvent& event)
{
	ros::Duration period = event.current_real - event.last_real;

	std::vector<double> current_position_vec_copy = current_position;

	// current pose of gripper using fk, jacobian matrix
	kinematic_solver_->compute_gripper_pose_and_jacobian(current_position_vec_copy, current_gripper_pose, Jacobian_Mat);

	std::cout << Jacobian_Mat << std::endl;

	// target poseStamped
	pd_frame_tracker_->get_transform("/arm_base_link", new_config.target_frame, target_gripper_pose);

	// optimal problem solver
	//pd_frame_tracker_->solver(J_Mat, current_gripper_pose, joint_velocity_data);
	pd_frame_tracker_->optimal_control_solver(Jacobian_Mat, current_gripper_pose, target_gripper_pose, joint_velocity_data);

	// error poseStamped, computation of euclidean distance error
	geometry_msgs::PoseStamped tip_Target_Frame_error_stamped;
	pd_frame_tracker_->get_transform(new_config.tip_link, new_config.target_frame, tip_Target_Frame_error_stamped);

	pd_frame_tracker_->compute_euclidean_distance(tip_Target_Frame_error_stamped.pose.position, cartesian_dist);
	pd_frame_tracker_->compute_rotation_distance(tip_Target_Frame_error_stamped.pose.orientation, rotation_dist);

	std::cout<<"\033[36;1m" << "***********************"<< "cartesian distance: " << cartesian_dist << "***********************" << std::endl
							<< "***********************"<< "rotation distance: " << rotation_dist << "***********************" << std::endl
			<< "\033[0m\n" << std::endl;

	if (cartesian_dist < 0.05 && rotation_dist < 0.05)
	{
			publish_zero_jointVelocity();
	}
	else
	{
		joint_velocity_pub.publish(joint_velocity_data);
	}
}

void predictive_control_node::convert_std_To_Eigen_vector(const std::vector<double>& std_vec, Eigen::VectorXd& eigen_vec)
{
	//std::assert(("convert_std_To_Eigen_vector ... Should not be empty vector",std_vec.empty()));
	eigen_vec.setZero(std_vec.size());
	for (int i = 0u; i < std_vec.size(); ++i)
	{
		eigen_vec(i) = std_vec.at(i);
	}
}

void predictive_control_node::publish_zero_jointVelocity()
{
	joint_velocity_data.data.resize(dof, 0.0);
	joint_velocity_data.data[0] = 0.0;
	joint_velocity_data.data[1] = 0.0;
	joint_velocity_data.data[2] = 0.0;
	joint_velocity_data.data[3] = 0.0;
	joint_velocity_data.data[4] = 0.0;
	joint_velocity_data.data[5] = 0.0;
	joint_velocity_data.data[6] = 0.0;
	joint_velocity_pub.publish(joint_velocity_data);
}
