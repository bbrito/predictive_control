
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/mpcc_controller.h>

MPCC::MPCC()
{
    ;
}

MPCC::~MPCC()
{
    clearDataMember();
    //delete controller_config_;
    //delete kinematic_solver_;
    //delete collision_detect_;
}

void MPCC::spinNode()
{
    ROS_INFO(" Predictive control node is running, now it's 'Spinning Node'");
    ros::spin();
}

// diallocated memory
void MPCC::clearDataMember()
{
    //current_position_ = Eigen::VectorXd(1.0);
    last_position_ = Eigen::VectorXd(3);
    //current_velocity_ = Eigen::VectorXd(1.0);
    last_velocity_ = Eigen::VectorXd(3);

}

// initialize all helper class of predictive control and subscibe joint state and publish controlled joint velocity
bool MPCC::initialize()
{
    // make sure node is still running
    if (ros::ok())
    {
        // initialize helper classes, make sure pd_config should be initialized first as mother of other class
        controller_config_.reset(new predictive_configuration());
        bool controller_config_success = controller_config_->initialize();

        //TO BE REPLACED BY CAR CALCULATION
        //kinematic_solver_.reset(new Kinematic_calculations());
        //bool kinematic_success = kinematic_solver_->initialize();
        bool kinematic_success = true;

        // COMMENTED OUT FOR NOW
        /*collision_detect_.reset(new CollisionRobot());
        bool collision_success = collision_detect_->initializeCollisionRobot();

        collision_avoidance_.reset(new CollisionAvoidance());
        bool collision_avoidance_success = collision_avoidance_->initialize(controller_config_);

        static_collision_avoidance_.reset(new StaticCollision());
        bool static_collision_success = static_collision_avoidance_->initializeStaticCollisionObject();
        */

        pd_trajectory_generator_.reset(new pd_frame_tracker());
        bool pd_traj_success = pd_trajectory_generator_->initialize();

        /* check successfully initialization of all classes
        if (controller_config_success == false || kinematic_success == false || collision_avoidance_success == false
                || collision_success == false || static_collision_success == false || pd_traj_success == false || controller_config_->initialize_success_ == false)
        */
        if (pd_traj_success == false || controller_config_success == false)
         {
            ROS_ERROR("MPCC: FAILED TO INITILIZED!!");
            std::cout << "States: \n"
                                << " pd_config: " << std::boolalpha << controller_config_success << "\n"
                                //<< " kinematic solver: " << std::boolalpha << kinematic_success << "\n"
                                //<< " collision avoidance: " << std::boolalpha << collision_avoidance_success << "\n"
                                //<< " collision detect: " << std::boolalpha << collision_success << "\n"
                                //<< " static collision avoidance: " << std::boolalpha << static_collision_success << "\n"
                                << " pd traj generator: " << std::boolalpha << pd_traj_success << "\n"
                                << " pd config init success: " << std::boolalpha << controller_config_->initialize_success_
                                << std::endl;
            return false;
        }

        // initialize data member of class
        clock_frequency_ = controller_config_->clock_frequency_;

        //DEBUG
        activate_debug_output_ = controller_config_->activate_debug_output_;
        tracking_ = true;
        move_action_result_.reach = false;
        plotting_result_ = controller_config_->plotting_result_;

        /// INFO: static function called transformStdVectorToEigenVector define in the predictive_trajectory_generator.h
        min_velocity_limit_ = pd_frame_tracker::transformStdVectorToEigenVector<double>(controller_config_->vel_min_limit_);
        max_velocity_limit_ = pd_frame_tracker::transformStdVectorToEigenVector<double>(controller_config_->vel_max_limit_);

        // DEBUG
        if (controller_config_->activate_controller_node_output_)
        {
            ROS_WARN("===== DEBUG INFO ACTIVATED =====");
        }

        // resize position and velocity velocity vectors
        current_state_ = Eigen::Vector3d(0,0,0);
        last_state_ = Eigen::Vector3d(0,0,0);
		prev_pose_ = Eigen::Vector3d(0,0,0);
		goal_pose_ = Eigen::Vector3d(0,0,0);
		next_pose_= Eigen::Vector3d(0,0,0);
		prev_pose_.setZero();
        goal_pose_.setZero();
        // initialize KINEMATICS
        // TO BE DONE

        // ros interfaces
        static const std::string MOVE_ACTION_NAME = "move_action";
        move_action_server_.reset(new actionlib::SimpleActionServer<predictive_control::moveAction>(nh, MOVE_ACTION_NAME, false));
        move_action_server_->registerGoalCallback(boost::bind(&MPCC::moveGoalCB, this));
        move_action_server_->registerPreemptCallback(boost::bind(&MPCC::movePreemptCB, this));
        move_action_server_->start();

	    // MOVEIT interfaces
	    static const std::string MOVEIT_ACTION_NAME = "fake_base_controller";
	    moveit_action_server_.reset(new actionlib::SimpleActionServer<predictive_control::trajAction>(nh, MOVEIT_ACTION_NAME, false));
	    moveit_action_server_->registerGoalCallback(boost::bind(&MPCC::moveitGoalCB, this));
	    moveit_action_server_->start();

        robot_state_sub_ = nh.subscribe(controller_config_->robot_state_topic_, 1, &MPCC::StateCallBack, this);

        //To be implemented
        traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory",1);
        controlled_velocity_pub_ = nh.advertise<geometry_msgs::Twist>(controller_config_->output_cmd,1);

        ros::Duration(1).sleep();

        timer_ = nh.createTimer(ros::Duration(1/clock_frequency_), &MPCC::runNode, this);
        timer_.start();

		//Initialize trajectory variables
		next_point_dist = 0;
		goal_dist = 0;
		prev_point_dist = 0;
		idx = 1;
		idy = 1;
		epsilon_ = 0.01;

		moveit_msgs::RobotTrajectory j;
		traj = j;

        ROS_WARN("PREDICTIVE CONTROL INTIALIZED!!");
        return true;
    }
    else
    {
        ROS_ERROR("MPCC: Failed to initialize as ROS Node is shoutdown");
        return false;
    }
}

// update this function 1/colck_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{
    if (activate_debug_output_)
    {
        ROS_INFO("MPCC::runNode");
    }

	if(((int)traj.multi_dof_joint_trajectory.points.size()) > 1){

		Eigen::Vector3d dist = current_state_-goal_pose_;
		if(dist.norm() < epsilon_){
			idx++;
			pd_trajectory_generator_->s_=0;
			ROS_INFO_STREAM("TRajectory point " << idx);
		}

		//assigning next point as goal pose since initial point of trajectory is actual pose
		prev_pose_(0) = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].translation.x;
		prev_pose_(1) = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].translation.y;
		prev_pose_(2) = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].rotation.z;
		next_pose_(0) = traj.multi_dof_joint_trajectory.points[idx ].transforms[0].translation.x;
		next_pose_(1) = traj.multi_dof_joint_trajectory.points[idx ].transforms[0].translation.y;
		next_pose_(2) = traj.multi_dof_joint_trajectory.points[idx ].transforms[0].rotation.z;
		//assigning next point as goal pose since initial point of trajectory is actual pose
		goal_pose_(0) = traj.multi_dof_joint_trajectory.points[idx+1].transforms[0].translation.x;
		goal_pose_(1) = traj.multi_dof_joint_trajectory.points[idx+1].transforms[0].translation.y;
		goal_pose_(2) = traj.multi_dof_joint_trajectory.points[idx+1].transforms[0].rotation.z;
			/*prev_pose_(0) = 0;
			prev_pose_(1) = 0;
			prev_pose_(2) = 0;
			goal_pose_(0) = 1;
			goal_pose_(1) = 0;
			goal_pose_(2) = 0;
	*/
			//pd_trajectory_generator_->initializeOptimalControlProblem(traj.joint_trajectory.points[0].positions);

			pd_trajectory_generator_->solveOptimalControlProblem(current_state_,prev_pose_,next_pose_, goal_pose_, controlled_velocity_);
		}
		// solver optimal control problem


		// publishes error stamped for plot, trajectory
		//this->publishErrorPose(tf_traget_from_tracking_vector_);
		//this->publishTrajectory();

			// publish zero controlled velocity
			if (!tracking_)
			{
				actionSuccess();
			}
			//publishZeroJointVelocity();
			controlled_velocity_pub_.publish(controlled_velocity_);

	}

	void MPCC::moveGoalCB()
	{
		if(move_action_server_->isNewGoalAvailable())
		{
			boost::shared_ptr<const predictive_control::moveGoal> move_action_goal_ptr = move_action_server_->acceptNewGoal();
			tracking_ = false;
			//collision_detect_->createStaticFrame(move_action_goal_ptr->target_endeffector_pose, move_action_goal_ptr->target_frame_id);
			//target_frame_ = move_action_goal_ptr->target_frame_id;

			//erase previous trajectory
			for (auto it = traj_marker_array_.markers.begin(); it != traj_marker_array_.markers.end(); ++it)
			{
				it->action = visualization_msgs::Marker::DELETE;
				traj_pub_.publish(traj_marker_array_);
				//traj_marker_array_.markers.erase(it);
			}

			traj_marker_array_.markers.clear();
		}
	}

	void MPCC::moveitGoalCB()
	{
		ROS_INFO_STREAM("Got new MoveIt goal!!!");
		//Reset trajectory index
		idx = 1;
		pd_trajectory_generator_->s_=0;
		if(moveit_action_server_->isNewGoalAvailable())
		{
			boost::shared_ptr<const predictive_control::trajGoal> moveit_action_goal_ptr = moveit_action_server_->acceptNewGoal();
			traj = moveit_action_goal_ptr->trajectory;
			tracking_ = false;
			//start trajectory execution
		}
	}

	void MPCC::executeTrajectory(const moveit_msgs::RobotTrajectory & traj){

	}

	void MPCC::movePreemptCB()
	{
		move_action_result_.reach = true;
		move_action_server_->setPreempted(move_action_result_, "Action has been preempted");
		tracking_ = true;
	}

	void MPCC::actionSuccess()
	{
		move_action_server_->setSucceeded(move_action_result_, "Goal succeeded!");
		tracking_ = true;
	}

	void MPCC::actionAbort()
	{
		move_action_server_->setAborted(move_action_result_, "Action has been aborted");
		tracking_ = true;
	}

	/*
	int MPCC::moveCallBack(const predictive_control::moveGoalConstPtr& move_action_goal_ptr)
	{

		predictive_control::moveResult result;
		predictive_control::moveFeedback feedback;
		goal_pose_.resize(6);

		// asume that not any target frame id set than it should use interative marker node
		if (move_action_goal_ptr->target_frame_id.empty() || move_action_goal_ptr->target_endeffector_pose.header.frame_id.empty())
		{
			tracking_ = true;
			result.reach = false;
			move_action_server_->setAborted(result, " Not set desired goal and frame id, Now use to intractive marker to set desired goal");
			//target_frame_ = controller_config_->target_frame_;
		}

		else
		{
			ROS_WARN("==========***********============ MPCC::moveCallBack: recieving new goal request ==========***********============");
			tracking_ = false;

			collision_detect_->createStaticFrame(move_action_goal_ptr->target_endeffector_pose, move_action_goal_ptr->target_frame_id);

			target_frame_ = move_action_goal_ptr->target_frame_id;

			////
			// extract goal gripper position and orientation
			goal_pose_(0) = move_action_goal_ptr->target_endeffector_pose.pose.position.x;
			goal_pose_(1) = move_action_goal_ptr->target_endeffector_pose.pose.position.y;
			goal_pose_(2) = move_action_goal_ptr->target_endeffector_pose.pose.position.z;

			tf::Quaternion quat(move_action_goal_ptr->target_endeffector_pose.pose.orientation.x,
													move_action_goal_ptr->target_endeffector_pose.pose.orientation.y,
													move_action_goal_ptr->target_endeffector_pose.pose.orientation.z,
													move_action_goal_ptr->target_endeffector_pose.pose.orientation.w);

			tf::Matrix3x3 matrix(quat);
			double r(0.0), p(0.0), y(0.0);
			matrix.getRPY(r, p, y);

			goal_pose_(3) = r;
			goal_pose_(4) = p;
			goal_pose_(5) = y;
	////
			// set result for validation
			move_action_server_->setSucceeded(result, "successfully reach to desired pose");

			// publish feed to tracking information
			tf::Matrix3x3 return_matrix;
			tf::Quaternion current_quat;
			return_matrix.setRPY(current_pose_(3), current_pose_(4), current_pose_(5));
			return_matrix.getRotation(current_quat);

			feedback.current_endeffector_pose.position.x = current_pose_(0);
			feedback.current_endeffector_pose.position.y = current_pose_(1);
			feedback.current_endeffector_pose.position.z = current_pose_(2);
			feedback.current_endeffector_pose.orientation.w = current_quat.w();
			feedback.current_endeffector_pose.orientation.x = current_quat.x();
			feedback.current_endeffector_pose.orientation.y = current_quat.y();
			feedback.current_endeffector_pose.orientation.z = current_quat.z();
			move_action_server_->publishFeedback(feedback);

		}
			return 0;
		/////
		predictive_control::moveResult result;
		result.reach = true;
		move_action_server_.setSucceeded(result, "move to target pose successful ");
		return 0;////
	}
	*/
// read current position and velocity of robot joints
void MPCC::StateCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
    if (activate_debug_output_)
    {
        //ROS_INFO("MPCC::StateCallBack");
    }
    last_state_ = current_state_;
    current_state_(0) =    msg->position.x;
    current_state_(1) =    msg->position.y;
    current_state_(2) =    msg->orientation.z;

}

/*
void MPCC_node::run_node(const ros::TimerEvent& event)
{
	ros::Duration period = event.current_real - event.last_real;

	std::vector<double> current_position_vec_copy = current_position;

	// current pose of gripper using fk, jacobian matrix
	kinematic_solver_->compute_gripper_pose_and_jacobian(current_position_vec_copy, current_gripper_pose, Jacobian_Mat);

	std::vector<geometry_msgs::Vector3> link_length;
	kinematic_solver_->compute_and_get_each_joint_pose(current_position_vec_copy, tranformation_matrix_stamped, link_length);

	std::map<std::string, geometry_msgs::PoseStamped> self_collsion_matrix;
	kinematic_solver_->compute_and_get_each_joint_pose(current_position_vec_copy, self_collsion_matrix);

	std::cout<<"\033[20;1m" << "############"<< "links " << self_collsion_matrix.size() << "###########" << "\033[0m\n" << std::endl;


	int id=0u;
	for (auto it = self_collsion_matrix.begin(); it != self_collsion_matrix.end(); ++it, ++id)
	{
		pd_frame_tracker_->create_collision_ball(it->second, 0.15, id);
	}

	marker_pub.publish(pd_frame_tracker_->get_collision_ball_marker());

	//boost::thread mux_thread{pd_frame_tracker_->generate_self_collision_distance_matrix(self_collsion_matrix)};
	//pd_frame_tracker_->generate_self_collision_distance_matrix(self_collsion_matrix, collision_distance_matrix);
	//mux_thread.join();

	collision_distance_vector = pd_frame_tracker_->compute_self_collision_distance(self_collsion_matrix, 0.20, 0.01);

	// target poseStamped
	pd_frame_tracker_->get_transform("/arm_base_link", new_config.target_frame, target_gripper_pose);

	// optimal problem solver
	//pd_frame_tracker_->solver(J_Mat, current_gripper_pose, joint_velocity_data);
	pd_frame_tracker_->optimal_control_solver(Jacobian_Mat, current_gripper_pose, target_gripper_pose, joint_velocity_data); //, collision_distance_vector

	// error poseStamped, computation of euclidean distance error
	geometry_msgs::PoseStamped tip_Target_Frame_error_stamped;
	pd_frame_tracker_->get_transform(new_config.tip_link, new_config.target_frame, tip_Target_Frame_error_stamped);

	pd_frame_tracker_->compute_euclidean_distance(tip_Target_Frame_error_stamped.pose.position, cartesian_dist);
	pd_frame_tracker_->compute_rotation_distance(tip_Target_Frame_error_stamped.pose.orientation, rotation_dist);

	std::cout<<"\033[36;1m" << "***********************"<< "cartesian distance: " << cartesian_dist << "***********************" << std::endl
							<< "***********************"<< "rotation distance: " << rotation_dist << "***********************" << std::endl
			<< "\033[0m\n" << std::endl;


	if (cartesian_dist < 0.03 && rotation_dist < 0.05)
	{
			publish_zero_jointVelocity();
	}
	else
	{
		joint_velocity_pub.publish(joint_velocity_data);
	}

}
*/

void MPCC::publishZeroJointVelocity()
{
    if (activate_debug_output_)
    {
        ROS_INFO("Publishing ZERO joint velocity!!");
    }
    geometry_msgs::Twist pub_msg;

    controlled_velocity_ = pub_msg;

    controlled_velocity_pub_.publish(controlled_velocity_);
}


// publishes error vector
void MPCC::publishErrorPose(const Eigen::VectorXd& error)
{
    //ROS_ERROR_STREAM("Errror_vector: " << error);

    geometry_msgs::PoseStamped stamped;
    stamped.header.frame_id = ""; // TO BE IMPLEMENTED
    stamped.header.stamp = ros::Time(0).now();

    this->transformEigenToGeometryPose(error, stamped.pose);

    if (activate_debug_output_)
    {
        std::cout << "\033[94m" << "publishErrorPose:" << " qx:" << stamped.pose.orientation.x
                            << "qy:" << stamped.pose.orientation.y
                            << "qz:" << stamped.pose.orientation.z
                            << "qw:" << stamped.pose.orientation.w << "\033[0m" <<std::endl;
    }
    // publish
    cartesian_error_pub_.publish(stamped);

}

// publishes trajectory
void MPCC::publishTrajectory()
{
    geometry_msgs::Pose pose;
    visualization_msgs::Marker marker;


    marker.type = visualization_msgs::Marker::SPHERE;
    //marker.lifetime = ros::Duration(5.0);
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "preview";
    marker.id = traj_marker_array_.markers.size();

    //scale
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // color
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.color.r = 0.0;
    marker.color.b = 0.0;

    //header
    marker.header.frame_id = ""; //To be defined controller_config_->;
    //marker.header.stamp = ros::Time(0).now();

    // convert pose from eigen vector
    this->transformEigenToGeometryPose(current_state_, pose);

    // pose array
    marker.pose = pose;

    traj_marker_array_.markers.push_back(marker);

    //publishes
    traj_pub_.publish(traj_marker_array_);
}

// convert Eigen Vector to geomentry Pose
bool MPCC::transformEigenToGeometryPose(const Eigen::VectorXd &eigen_vector, geometry_msgs::Pose &pose)
{
    // traslation
    pose.position.x = eigen_vector(0);
    pose.position.y = eigen_vector(1);

    tf::Matrix3x3 quat_matrix;
    quat_matrix.setRPY(0, 0, eigen_vector(2));

    if (activate_debug_output_)
    {
        std::cout << "\033[32m" << "publishErrorPose:" << " x:" << eigen_vector(0)
                            << " y:" << eigen_vector(1)
                            << " yaw:" << eigen_vector(2)<< "\033[0m" <<std::endl;
    }

    tf::Quaternion quat_tf;
    quat_matrix.getRotation(quat_tf);

    // rotation
    pose.orientation.w = quat_tf.w();
    pose.orientation.x = quat_tf.x();
    pose.orientation.y = quat_tf.y();
    pose.orientation.z = quat_tf.z();

    return true;
}


bool MPCC::getTransform(const std::string& from, const std::string& to, Eigen::VectorXd& stamped_pose)
{
    bool transform = false;
    stamped_pose = Eigen::VectorXd(6);
    tf::StampedTransform stamped_tf;

    // make sure source and target frame exist
    if (tf_listener_.frameExists(to) && tf_listener_.frameExists(from))
    {
        try
        {
            // find transforamtion between souce and target frame
            tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.02));
            tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);

            // translation
            stamped_pose(0) = stamped_tf.getOrigin().x();
            stamped_pose(1) = stamped_tf.getOrigin().y();
            stamped_pose(2) = stamped_tf.getOrigin().z();

            // convert quternion to rpy
            tf::Quaternion quat(stamped_tf.getRotation().getX(),
                                                    stamped_tf.getRotation().getY(),
                                                    stamped_tf.getRotation().getZ(),
                                                    stamped_tf.getRotation().getW()
            );

            if (activate_debug_output_)
            {
                std::cout << "\033[94m" << "getTransform:" << " qx:" << stamped_tf.getRotation().getX()
                                    << "qy:" << stamped_tf.getRotation().getY()
                                    << "qz:" << stamped_tf.getRotation().getZ()
                                    << "qw:" << stamped_tf.getRotation().getW() << "\033[0m" <<std::endl;
            }

            tf::Matrix3x3 quat_matrix(quat);
            quat_matrix.getRPY(stamped_pose(3), stamped_pose(4), stamped_pose(5));

            if (activate_debug_output_)
            {
                std::cout << "\033[32m" << "getTransform:" << " roll:" << stamped_pose(3)
                                    << " pitch:" << stamped_pose(4)
                                    << " yaw:" << stamped_pose(5)
                                    << "\033[0m" <<std::endl;
            }

            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("MPCC::getTransform: %s", ex.what());
        }
    }

    else
    {
        ROS_WARN("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",
                         from.c_str(), to.c_str());
    }

    return transform;
}

/*
bool MPCC::getTransform(const std::string& from, const std::string& to, geometry_msgs::PoseStamped& stamped_pose)
{
    bool transform = false;
    tf::StampedTransform stamped_tf;

    // make sure source and target frame exist
    if (tf_listener_.frameExists(to) & tf_listener_.frameExists(from))
    {
        try
        {
            // find transforamtion between souce and target frame
            tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.2));
            tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);

            // rotation
            stamped_pose.pose.orientation.w =    stamped_tf.getRotation().getW();
            stamped_pose.pose.orientation.x =    stamped_tf.getRotation().getX();
            stamped_pose.pose.orientation.y =    stamped_tf.getRotation().getY();
            stamped_pose.pose.orientation.z =    stamped_tf.getRotation().getZ();

            // translation
            stamped_pose.pose.position.x = stamped_tf.getOrigin().x();
            stamped_pose.pose.position.y = stamped_tf.getOrigin().y();
            stamped_pose.pose.position.z = stamped_tf.getOrigin().z();

            // header frame_id should be parent frame
            stamped_pose.header.frame_id = stamped_tf.frame_id_;    //from or to
            stamped_pose.header.stamp = ros::Time(0);

            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("MPCC_node::getTransform: \n%s", ex.what());
        }
    }

    else
    {
        ROS_WARN("%s or %s frame doesn't exist, pass existing frame", from.c_str(), to.c_str());
    }

    return transform;
}
*/

// check position lower and upper limit violation
bool MPCC::checkVelocityLimitViolation(const std_msgs::Float64MultiArray &joint_velocity, const double &velocity_tolerance)
{
    int size = joint_velocity.data.size();
    // confirm size using conditional operator
    //size == 0? min_position_limit_.rows()* min_position_limit_.cols(): size;

    for (int i = 0u; i < size; ++i )
    {
        // Current position is below than minimum position limit + tolerance, check lower limit violation
        if ( (min_velocity_limit_(i) - velocity_tolerance) >= joint_velocity.data[i] )
        {
            ROS_WARN("lower velocity tolerance violate with current velocity %f required velocity %f",
                             joint_velocity.data[i], (min_velocity_limit_(i) - velocity_tolerance));
            return true;
        }

            // Current position is above than maximum position limit + tolerance, check upper limit violation
        else if ( (max_velocity_limit_(i) + velocity_tolerance) <= joint_velocity.data[i] )
        {
            ROS_WARN("upper velocity tolerance violate with current velocity %f required velocity %f",
                             joint_velocity.data[i], (max_velocity_limit_(i) + velocity_tolerance));
            return true;
        }

            // Current velocity is within range of minimum and maximum velocity limit
        else if ( activate_debug_output_ &&
                            (((min_velocity_limit_(i)+ velocity_tolerance) - joint_velocity.data[i]) <= 0.0) &&
                            ((joint_velocity.data[i] - (max_velocity_limit_(i) + velocity_tolerance) <= 0.0) )
                )
        {
            ROS_INFO("Current velocity is within range of minimum and maximum velocity limit");
        }
    }
    return false;
}

// enforcing joint locity to be in velocity limits
void MPCC::enforceVelocityInLimits(const std_msgs::Float64MultiArray& joint_velocity,
                                                                     std_msgs::Float64MultiArray& enforced_joint_velocity)
{
    // initialze enforced joint position vector, if limit violate than update otherwise remain same as joint position
    enforced_joint_velocity = joint_velocity;
    int size = joint_velocity.data.size();
    // confirm size using conditional operator
    //size == 0? min_velocity_limit_.rows()* min_position_limit_.cols(): size;

    for (int i = 0u; i < size; ++i )
    {
        // Current position is below than minimum velocity limit + tolerance, check lower limit violation
        if ( ( (min_velocity_limit_(i) ) >= joint_velocity.data[i] ) &&
                 ( std::abs(joint_velocity.data[i]-min_velocity_limit_(i)) < std::abs(max_velocity_limit_(i)-joint_velocity.data[i]) )
                )
        {
            ROS_WARN("lower velocity tolerance violate with current position %f required position %f",
                             joint_velocity.data[i], (min_velocity_limit_(i) ));

            // enforced lower joint limit
            enforced_joint_velocity.data[i] = min_velocity_limit_(i);
            ROS_INFO("new velocity %f", min_velocity_limit_(i));
        }

            // Current velocity is above than maximum position limit + tolerance, check upper limit violation
        else if ( ( (max_velocity_limit_(i) ) <= joint_velocity.data[i] ) &&
                            ( std::abs(joint_velocity.data[i]-min_velocity_limit_(i)) > std::abs(max_velocity_limit_(i)-joint_velocity.data[i]) )
                )
        {
            ROS_WARN("upper velocity tolerance violate with current position %f required position %f",
                             joint_velocity.data[i], (max_velocity_limit_(i) ));

            // enforced lower joint limit
            enforced_joint_velocity.data[i] = max_velocity_limit_(i);
            ROS_INFO("new velocity %f", max_velocity_limit_(i));
        }

            // Current position is within range of minimum and maximum position limit
        else if ( activate_debug_output_ &&
                            (((min_velocity_limit_(i)) - joint_velocity.data[i]) <= 0.0) &&
                            ((joint_velocity.data[i] - (max_velocity_limit_(i) ) <= 0.0) )
                )
        {
            ROS_INFO("Current velocity is within range of minimum and maximum velocity limit");
        }
    }

}
