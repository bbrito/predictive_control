
//This file containts cost function intsert in to generated trajectory.

#include <predictive_control/predictive_trajectory_generator.h>
#include <iomanip>	//print false or true

predictive_config::predictive_config()
{
	nh =  ros::this_node::getName() ;

	activate_output = false;

	// Chain param
	dof = 7;
	base_link = "arm_base_link";
	tip_link = "arm_7_link";
	root_frame = "arm_base_link";
	target_frame = "target";

	position_tolerance_violate = false;
	velocity_tolerance_violate = false;

	min_discretization_steps = max_discretization_steps = discretization_steps = 10;
	min_position_limit = -1.0; max_position_limit = 1.0;
	min_velocity_limit = max_velocity_limit = desired_velocity = 1.5;
	position_tolerance = 1.0; velocity_tolerance = 0.1;

	//limits_tolerance = 10.0;

}

predictive_config::~predictive_config()
{
	jnts_name.clear();
}

void predictive_config::update_config_parameters(predictive_config& new_param)
{
	activate_output = new_param.activate_output;
	update_rate = new_param.update_rate;
	dof = new_param.dof;
	base_link = new_param.base_link;
	tip_link = new_param.tip_link;
	root_frame = new_param.root_frame;
	target_frame = new_param.target_frame;
	jnts_name = new_param.jnts_name;

	min_position_limit = new_param.min_position_limit;
	max_position_limit = new_param.max_position_limit;
	min_velocity_limit = new_param.min_velocity_limit;
	max_velocity_limit = new_param.max_velocity_limit;
	desired_velocity = new_param.desired_velocity;
	position_tolerance = new_param.position_tolerance;
	velocity_tolerance = new_param.velocity_tolerance;

	min_discretization_steps = new_param.min_discretization_steps;
	max_discretization_steps = new_param.max_discretization_steps;
	discretization_steps = new_param.discretization_steps;
}

void predictive_config::choose_discretization_steps()
{
	if (discretization_steps == 0)
	{
		discretization_steps = static_cast<int>( std::ceil((min_discretization_steps + max_discretization_steps)*0.5) );
	}
}

void predictive_config::enforce_position_limit(double current_position, double& corrected_position)
{
	ROS_DEBUG_STREAM("Maximum position limit with tolerance(max_position_limit + position_tolerance): "<< (max_position_limit+position_tolerance));
	ROS_DEBUG_STREAM("Manimum position limit with tolerance(min_position_limit - position_tolerance): "<< (min_position_limit-position_tolerance));
	ROS_DEBUG_STREAM("Absolute value of current position - min_position_limit: "<<std::abs(current_position - min_position_limit));
	ROS_DEBUG_STREAM("Absolute value of current position - max_position_limit: "<<std::abs(max_position_limit - current_position));

	// Current position is within defined minimum and maximum position limits
	if ( ((min_position_limit-position_tolerance) - current_position <0.0) &&  (current_position - (max_position_limit+position_tolerance) < 0.0))
	{
		corrected_position = current_position;
	}

	// Current position outside the range, and also near to minimum limit.
	else if ( ((min_position_limit-position_tolerance) >= current_position )
				&& (std::abs(current_position - min_position_limit) ) < std::abs(max_position_limit - current_position) )
	{
		corrected_position = min_position_limit;
	}

	// Current position outside the range, and also near to maximum limit.
	else if ( ((max_position_limit+position_tolerance) <= current_position)
			&& ((std::abs(current_position - min_position_limit) ) > std::abs(max_position_limit - current_position)) )
	{
		corrected_position = max_position_limit;
	}

	else
	{
		ROS_WARN("No position limit satisfied, need to check code and limit");
	}
}

bool predictive_config::check_position_tolerance_violation(double current_position)
{
	ROS_DEBUG_STREAM("Current position: "<<current_position);
	ROS_DEBUG_STREAM("Difference value current - (min limit - tol): " << current_position - (min_position_limit - position_tolerance));
	ROS_DEBUG_STREAM("Difference value current (max limit + tol): " <<current_position - (max_position_limit + position_tolerance));

	// Current position is below than minimum position limit + tolerance
	if ( ((min_position_limit-position_tolerance) >= current_position ))	//(current_position + (min_position_limit - position_tolerance)) < 0.0)
	{
		position_tolerance_violate = true;
		ROS_WARN("Position tolerance violate with current position %f required position %f ... check_position_tolerance_violation",
					current_position, (min_position_limit - position_tolerance) );
		return true;
	}

	// Current position is above than maximum position limit + tolerance
	else if ((max_position_limit+position_tolerance) <= current_position)	//( (current_position - (max_position_limit + position_tolerance)) >= 0.0)
	{
		position_tolerance_violate = true;
		ROS_WARN("Position tolerance violate with current position %f required position %f ... check_position_tolerance_violation",
					current_position, (max_position_limit + position_tolerance) );
		return true;
	}

	// Current position is within range of minimum and maximum position limit
	else if ( ((min_position_limit - current_position) <= 0.0) && ((current_position - max_position_limit) <= 0.0) )
	{
		ROS_INFO("Current position is within range of minimum and maximum position limit ... predictive_config::check_position_tolerance_violation");
	}

	return false;
}

/*
void predictive_config::enforce_velocity_limit(double current_velocity, double& corrected_velocity)
{
	// Current position inside the range.
	if ( (current_velocity - min_position_limit > 0) || (min_position_limit + position_tolerance < current_velocity) )
	{
		corrected_velocity = current_velocity;
	}

	// Current position outside the range, and also near to minimum limit.
	else if ( ((current_velocity - min_position_limit < 0)  || (min_position_limit + position_tolerance > current_velocity))
				&& (std::abs(current_velocity - min_position_limit) ) < std::abs(max_position_limit - current_velocity) )
	{
		corrected_velocity = min_position_limit;
	}

	// Current position inside the range.
	else if ( (  max_position_limit - current_velocity  > 0) || (max_position_limit - position_tolerance > current_velocity) )
	{
		corrected_velocity = current_velocity;
	}

	// Current position outside the range, and also near to maximum limit.
	else if ( ((max_position_limit - current_velocity  < 0)  || (max_position_limit - position_tolerance < current_velocity))
				&& (std::abs(current_velocity - min_position_limit) ) > std::abs(max_position_limit - current_velocity) )
	{
		corrected_velocity = max_position_limit;
	}

	else
	{
		ROS_WARN("No velocity limit satisfied, need to check code and limit ... enforce_velocity_limit");
	}
}
*/
void predictive_config::enforce_velocity_limit(double current_velocity, double& corrected_velocity)
{
	ROS_DEBUG_STREAM("Maximum velocity limit with tolerance(max_velocity_limit + velocity_tolerance): "<< (max_velocity_limit+velocity_tolerance));
	ROS_DEBUG_STREAM("Manimum velocity limit with tolerance(min_velocity_limit - velocity_tolerance): "<< (min_velocity_limit-velocity_tolerance));
	ROS_DEBUG_STREAM("Absolute value of current velocity - min_velocity_limit: "<<std::abs(current_velocity - min_velocity_limit));
	ROS_DEBUG_STREAM("Absolute value of current velocity - max_velocity_limit: "<<std::abs(max_velocity_limit - current_velocity));

	// Current velocity is within defined minimum and maximum position limits
	if ( ((min_velocity_limit-velocity_tolerance) - current_velocity <0.0) &&  (current_velocity - (max_velocity_limit+velocity_tolerance) < 0.0))
	{
		corrected_velocity = current_velocity;
	}

	// Current velocity outside the range, and also near to minimum limit.
	else if ( ((min_velocity_limit-velocity_tolerance) >= current_velocity )
				&& (std::abs(current_velocity - min_velocity_limit) ) < std::abs(max_velocity_limit - current_velocity) )
	{
		corrected_velocity = min_velocity_limit;
	}

	// Current velocity outside the range, and also near to maximum limit.
	else if ( ((max_velocity_limit+velocity_tolerance) <= current_velocity)
			&& ((std::abs(current_velocity - min_velocity_limit) ) > std::abs(max_velocity_limit - current_velocity)) )
	{
		corrected_velocity = max_velocity_limit;
	}

	else
	{
		ROS_WARN("No velocity limit satisfied, need to check code and limit");
	}
}

/*
bool predictive_config::check_velocity_tolerance_violation(double current_velocity)
{
	if ( current_velocity - (min_velocity_limit - velocity_tolerance) < 0)
	{
		velocity_tolerance_violate = true;
		ROS_WARN("Velocity tolerance violate with current velocity %f required velocity %f ... check_velocity_tolerance_violation",
				current_velocity, (min_velocity_limit - velocity_tolerance) );
		return true;
	}
	else if ( current_velocity - (max_velocity_limit + velocity_tolerance) > 0)
	{
		velocity_tolerance_violate = true;
		ROS_WARN("Velocity tolerance violate with current velocity %f required velocity %f ... check_velocity_tolerance_violation",
				current_velocity, (max_velocity_limit + velocity_tolerance) );
		return true;
	}

	return false;
}
*/

bool predictive_config::check_velocity_tolerance_violation(double current_velocity)
{
	ROS_DEBUG_STREAM("Current velocity: "<<current_velocity);
	ROS_DEBUG_STREAM("Difference value current - (min limit - tol): " << current_velocity - (min_velocity_limit - velocity_tolerance));
	ROS_DEBUG_STREAM("Difference value current (max limit + tol): " <<current_velocity - (max_velocity_limit + velocity_tolerance));

	// Current velocity is below than minimum velocity limit + tolerance
	if ((min_velocity_limit-velocity_tolerance) >= current_velocity )	//( (current_velocity + (min_velocity_limit - velocity_tolerance)) < 0.0)
	{
		position_tolerance_violate = true;
		ROS_WARN("Velocity tolerance violate with current position %f required position %f ... check_velocity_tolerance_violation",
				current_velocity, (min_velocity_limit - velocity_tolerance) );
		return true;
	}

	// Current velocity is above than maximum velocity limit + tolerance
	else if ((max_velocity_limit+velocity_tolerance) <= current_velocity)	//( (current_velocity - (max_velocity_limit + velocity_tolerance)) >= 0.0)
	{
		position_tolerance_violate = true;
		ROS_WARN("Velocity tolerance violate with current position %f required position %f ... check_velocity_tolerance_violation",
				current_velocity, (max_velocity_limit + velocity_tolerance) );
		return true;
	}

	// Current velocity is within range of minimum and maximum velocity limit
	else if ( ((min_velocity_limit-velocity_tolerance) - current_velocity <0.0)
			&&  (current_velocity - (max_velocity_limit+velocity_tolerance) < 0.0))	//(min_velocity_limit < current_velocity < max_velocity_limit)
	{
		ROS_INFO("Current velocity is within range of minimum and maximum velocity limit ... predictive_config::check_velocity_tolerance_violation");
	}

	return false;
}

void predictive_config::print_data_member()
{
	/*
	ROS_INFO("DOF: %f", dof);
	ROS_INFO("Base_link: %s", base_link);
	ROS_INFO("Tip_link: %s", tip_link);
	ROS_INFO("Root_frame: %s", root_frame);
	ROS_INFO("Min_position_limit: %d", min_position_limit);
	ROS_INFO("Max_position_limit: %d", max_position_limit);
	ROS_INFO("Min_velocity_limit: %d", min_velocity_limit);
	ROS_INFO("Max_velocity_limit: %d", max_velocity_limit);
	ROS_INFO("Desired_velocity: %d", desired_velocity);
	ROS_INFO("Position_tolerance: %d", position_tolerance);
	ROS_INFO("Velocity_tolerance: %d", velocity_tolerance);
	ROS_INFO("Min_discretization_steps: %u", min_discretization_steps);
	ROS_INFO("Max_discretization_steps: %u", max_discretization_steps);
	ROS_INFO("Discretization_steps: %u", discretization_steps);
*/

	std::cout<< " ------------------------------------------------------- \n"<<std::endl;
	ROS_INFO_STREAM("DOF: " << dof);
	ROS_INFO_STREAM("Base_link: " << base_link);
	ROS_INFO_STREAM("Tip_link: "<< tip_link);
	ROS_INFO_STREAM("Root_frame: "<< root_frame);
	ROS_INFO_STREAM("Target_frame: "<< target_frame);
	ROS_INFO_STREAM("Min_position_limit: "<< min_position_limit);
	ROS_INFO_STREAM("Max_position_limit: "<< max_position_limit);
	ROS_INFO_STREAM("Min_velocity_limit: "<< min_velocity_limit);
	ROS_INFO_STREAM("Max_velocity_limit: "<< max_velocity_limit);
	ROS_INFO_STREAM("Desired_velocity: "<< desired_velocity);
	ROS_INFO_STREAM("Position_tolerance: "<< position_tolerance);
	ROS_INFO_STREAM("Velocity_tolerance: "<< velocity_tolerance);
	ROS_INFO_STREAM("Min_discretization_steps: "<< min_discretization_steps);
	ROS_INFO_STREAM("Max_discretization_steps: "<< max_discretization_steps);
	ROS_INFO_STREAM("Discretization_steps: "<< discretization_steps);
	ROS_INFO_STREAM("Activation output: " << std::boolalpha << activate_output);
	ROS_INFO_STREAM("Position tolerance violation: " << std::boolalpha << position_tolerance_violate);
	ROS_INFO_STREAM("Velocity tolerance violation: " << std::boolalpha << velocity_tolerance_violate);

	std::cout << "Joint names: [";
	for_each(jnts_name.begin(), jnts_name.end(), [](std::string& str)
												{ std::cout << str << ", " ; }
	);
	std::cout<<"]"<<std::endl;

	std::cout<< " \n ------------------------------------------------------- "<<std::endl;
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------

bool pd_frame_tracker::initialization(const predictive_config& pd_config)
{
	base_link_ = pd_config.base_link;
	tip_link_ = pd_config.tip_link;
	root_frame_ = pd_config.root_frame;
	tracking_frame_ = pd_config.tip_link;
	target_frame_ = pd_config.target_frame;


	ROS_WARN("pd_frame_tracker INTIALIZED!!");
	return true;
}

void pd_frame_tracker::solver(const Eigen::MatrixXd& jacobian_mat, const Eigen::VectorXd& current_endeffector_pose, std_msgs::Float64MultiArray& updated_vel)
{
	ROS_WARN("Solving OCP problem using ACADO Toolkit");

    tf::StampedTransform transform_tf, target_frame_TO_root_frame;
    bool success = this->getTransform(tracking_frame_, target_frame_, transform_tf); //target_frame_
    bool success1 = this->getTransform("/arm_podest_link", target_frame_, target_frame_TO_root_frame); //target_frame_

    std::cout << jacobian_mat << std::endl;
    DMatrix Jac_Mat = jacobian_mat;
    const unsigned int m = 6;  // rows of jacobian matrix, 6 with 3 lin and 3 angular velocity
    const unsigned int n = 7;

    DifferentialState x("",m,1);                // position
    Control v("",n,1);                      // velocity

    x.clearStaticCounters();
    v.clearStaticCounters();

    DifferentialEquation f;             // Define differential equation
    f << dot(x) == Jac_Mat * v;

	DVector c_init(7), s_init(6);
	c_init.setAll(0.00);
	s_init.setAll(0.00);
	s_init(0) = current_endeffector_pose(0);		s_init(1) = current_endeffector_pose(1);		s_init(2) = current_endeffector_pose(2);
	s_init(3) = current_endeffector_pose(3);		s_init(4) = current_endeffector_pose(4);		s_init(5) = current_endeffector_pose(5);
	c_init(0) = updated_vel.data[0];	c_init(1) = updated_vel.data[1];	c_init(2) = updated_vel.data[2];
	c_init(3) = updated_vel.data[3];	c_init(4) = updated_vel.data[4];	c_init(5) = updated_vel.data[5];	c_init(6) = updated_vel.data[6];

	//s_init = current_endeffector_pose;
	//c_init = updated_vel.data;

	OCP ocp_problem(0.0, 1.0, 4);
    ocp_problem.minimizeMayerTerm( 2.0*(v.transpose()*v) + 10.0*(( (x(0)-target_frame_TO_root_frame.getOrigin().x())  * (x(0)-target_frame_TO_root_frame.getOrigin().x()) ) +
    							   ( (x(1)-target_frame_TO_root_frame.getOrigin().y())  * (x(1)-target_frame_TO_root_frame.getOrigin().y()) ) +
    							   ( (x(2)-target_frame_TO_root_frame.getOrigin().z())  * (x(2)-target_frame_TO_root_frame.getOrigin().z()) )
    							   ));
    ocp_problem.subjectTo(f);
    ocp_problem.subjectTo(-0.50 <= v <= 0.50);
    //ocp_problem.subjectTo(AT_START, v == 1.0);
    ocp_problem.subjectTo(AT_END , v == 0.0);

    RealTimeAlgorithm alg(ocp_problem, 0.025);
	alg.initializeControls(c_init);
	alg.initializeDifferentialStates(s_init);
    alg.set(MAX_NUM_ITERATIONS, 10);
    alg.set(LEVENBERG_MARQUARDT, 1e-5);
    alg.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    //alg.set( DISCRETIZATION_TYPE, COLLOCATION);
    alg.set( DISCRETIZATION_TYPE, COLLOCATION);
    alg.set(KKT_TOLERANCE, 1.000000E-06);

    Controller controller(alg);

    controller.init(0.0, s_init);
    controller.step(0.0, s_init);

    DVector cnt_jnt_vel;
    controller.getU(cnt_jnt_vel);
    cnt_jnt_vel.print();

	double cart_distance = sqrt(transform_tf.getOrigin().x()*transform_tf.getOrigin().x() +
								transform_tf.getOrigin().y()*transform_tf.getOrigin().y() +
								transform_tf.getOrigin().z()*transform_tf.getOrigin().z()
								);

  //print data on to console
	std::cout<<"\033[36;1m" // green console colour
			<< "***********************"<<std::endl
			<< "cartesian distance: " << cart_distance
			<< "***********************"
			<< "\033[0m\n" << std::endl;

	if( cart_distance < 0.05 )
	{
		updated_vel.data.resize(7,0.0);
		updated_vel.data[0] = 0;
		updated_vel.data[1] = 0;
		updated_vel.data[2] = 0;
		updated_vel.data[3] = 0;
		updated_vel.data[4] = 0;
		updated_vel.data[5] = 0;
		updated_vel.data[6] = 0;
	}
	else
	{
		updated_vel.data.resize(7,0.0);
		updated_vel.data[0] = cnt_jnt_vel(0);
		updated_vel.data[1] = cnt_jnt_vel(1);
		updated_vel.data[2] = cnt_jnt_vel(2);
		updated_vel.data[3] = cnt_jnt_vel(3);
		updated_vel.data[4] = cnt_jnt_vel(4);
		updated_vel.data[5] = cnt_jnt_vel(5);
		updated_vel.data[6] = cnt_jnt_vel(6);
	}

}

bool pd_frame_tracker::getTransform(const std::string& from, const std::string& to, tf::StampedTransform& stamped_tf)
{
    bool transform = false;

    try
    {
        tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.2));
        tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);
        transform = true;
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("CobFrameTracker::getTransform: \n%s", ex.what());
    }

    return transform;
}



