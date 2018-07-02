
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

        obstacle_feed_sub_ = nh.subscribe(controller_config_->sub_ellipse_topic_, 1, &MPCC::ObstacleCallBack, this);

        //To be implemented
        traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory",1);
        tr_path_pub_ = nh.advertise<nav_msgs::Path>("horizon",1);
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

		//Initialize obstacles
		obstacle_feed::Obstacle obstacle_init;

        std_msgs::Header obst_header_init;

        obst_header_init.stamp = ros::Time::now();
        obst_header_init.frame_id = controller_config_->robot_base_link_;

        for (int it_obst = 0; it_obst < controller_config_->n_obstacles_; it_obst++)
        {
            obstacle_init.pose.orientation.z = 0;
            obstacle_init.pose.position.x = 1000;
            obstacle_init.pose.position.y = 1000;
            obstacle_init.minor_semiaxis = 0.01;
            obstacle_init.major_semiaxis = 0.01;

            obstacles_init_.header = obst_header_init;
            obstacles_init_.Obstacles.push_back(obstacle_init);
        }

        obstacles_ = obstacles_init_;

        std::cout << "initialized " << obstacles_.Obstacles.size() << " obstacles" << std::endl;

		moveit_msgs::RobotTrajectory j;
		traj = j;
		//initialize trajectory variable to plot prediction trajectory
		pred_traj_.poses.resize(controller_config_->discretization_intervals_+1);
		for(int i=0;i < controller_config_->discretization_intervals_; i++)
		{
			pred_traj_.poses[i].header.frame_id = "odom";
			pred_traj_.header.frame_id = "odom";
		}

		pred_traj_pub_ = nh.advertise<nav_msgs::Path>("mpc_horizon",1);

        ROS_WARN("PREDICTIVE CONTROL INTIALIZED!!");
        return true;
    }
    else
    {
        ROS_ERROR("MPCC: Failed to initialize as ROS Node is shoutdown");
        return false;
    }
}

void MPCC::transformPose(const std::string source_frame, const std::string target_frame, const geometry_msgs::Pose pose_in, geometry_msgs::Pose& pose_out)
{
	ROS_WARN_STREAM("MPCC::transformPose ... find transformation matrix between "
						<< target_frame.c_str() << " and " << source_frame.c_str());
	bool transform = false;
	geometry_msgs::PoseStamped stamped_in, stamped_out;
	stamped_in.header.frame_id = source_frame;
	stamped_in.pose = pose_in;
	do
	{
		try
		{
			if (tf_listener_.frameExists(target_frame))
			{
				// clear output
				pose_out = geometry_msgs::Pose();

				ros::Time now = ros::Time::now();
				tf_listener_.waitForTransform(target_frame, source_frame, now, ros::Duration(0.1));
				tf_listener_.transformPose(target_frame, stamped_in, stamped_out);
				pose_out = stamped_out.pose;
				transform = true;
			}

			else
			{
				ROS_WARN_STREAM("MPCC::transformPose" << target_frame.c_str() << " does not exist");
				transform = false;
			}
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("MPCC::transformPose: \n%s", ex.what());
			ros::Duration(0.1).sleep();
		}
	} while (!transform && ros::ok());
}

// update this function 1/colck_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{
	ROS_INFO_STREAM("MPCC::runNode");
	geometry_msgs::Pose pose_in,pose_out;
	if(traj.multi_dof_joint_trajectory.points.size()>4) {
		int idx = 0;
		//for (int idx = 0; idx < traj.multi_dof_joint_trajectory.points.size(); idx++) {

		//assigning next point as goal pose since initial point of trajectory is actual pose

		//Initialize and build splines for x and y
		std::vector<double> X, Y, S;

		X.push_back(traj.multi_dof_joint_trajectory.points[idx].transforms[0].translation.x);
		X.push_back(traj.multi_dof_joint_trajectory.points[idx + 1].transforms[0].translation.x);
		X.push_back(traj.multi_dof_joint_trajectory.points[idx + 2].transforms[0].translation.x);
		X.push_back(traj.multi_dof_joint_trajectory.points[idx + 3].transforms[0].translation.x);
		Y.push_back(traj.multi_dof_joint_trajectory.points[idx].transforms[0].translation.y);
		Y.push_back(traj.multi_dof_joint_trajectory.points[idx + 1].transforms[0].translation.y);
		Y.push_back(traj.multi_dof_joint_trajectory.points[idx + 2].transforms[0].translation.y);
		Y.push_back(traj.multi_dof_joint_trajectory.points[idx + 3].transforms[0].translation.y);
		S.push_back(0);
		S.push_back(2);

		pd_trajectory_generator_->ref_path_x.set_points(S, X);
		pd_trajectory_generator_->ref_path_y.set_points(S, Y);
		ROS_INFO_STREAM("ref_path_x.m_a " << pd_trajectory_generator_->ref_path_y.m_a);
		ROS_INFO_STREAM("ref_path_x.m_b " << pd_trajectory_generator_->ref_path_y.m_b);
		ROS_INFO_STREAM("ref_path_x.m_c " << pd_trajectory_generator_->ref_path_y.m_c);
		ROS_INFO_STREAM("ref_path_x.m_d " << pd_trajectory_generator_->ref_path_y.m_d);
		//}
		idx = traj.multi_dof_joint_trajectory.points.size();
		pose_in.position.x = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].translation.x;
		pose_in.position.y = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].translation.y;
		pose_in.position.z = 0;
		pose_in.orientation.x = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].rotation.x;
		pose_in.orientation.y = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].rotation.y;
		pose_in.orientation.z = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].rotation.z;
		pose_in.orientation.w = traj.multi_dof_joint_trajectory.points[idx - 1].transforms[0].rotation.w;
		transformPose("odom", "base_link", pose_in, pose_out);
		goal_pose_(0) = pose_out.position.x;
		goal_pose_(1) = pose_out.position.y;
		goal_pose_(2) = pose_out.position.z;

		states = pd_trajectory_generator_->solveOptimalControlProblem(current_state_, goal_pose_, obstacles_,
																	  controlled_velocity_);
		publishPredictedTrajectory();


		//publishPathFromTrajectory(traj);


		// publish zero controlled velocity
		if (!tracking_) {
			actionSuccess();
		}
		//publishZeroJointVelocity();
		controlled_velocity_pub_.publish(controlled_velocity_);
	}
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

void MPCC::ObstacleCallBack(const obstacle_feed::Obstacles& obstacles)
{
    geometry_msgs::PoseStamped stampedpose;
    stampedpose.header.frame_id = obstacles.header.frame_id;
    stampedpose.header.stamp = ros::Time::now();
    obstacle_feed::Obstacles obstacles_temp;

    obstacles_temp = obstacles_init_;
    obstacles_temp = obstacles;
    obstacles_ = obstacles_temp;

}

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

void MPCC::publishPredictedTrajectory(void)
{
	//pred_traj_.header.stamp = ros::Time::now();

	for(int i=0;i<states.getNumPoints();i++){
		state = states.getVector(i);
		//pred_traj_.poses[i].header.stamp = ros::Time::now();
		pred_traj_.poses[i].pose.position.x= state(0);
		pred_traj_.poses[i].pose.position.y= state(1);
	}
	pred_traj_pub_.publish(pred_traj_);
}

void MPCC::publishPathFromTrajectory(const moveit_msgs::RobotTrajectory& traj)
{

    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "odom";
    pose.header.stamp = ros::Time::now();

    for (int sample_it = 0 ; sample_it < traj.multi_dof_joint_trajectory.points.size() ; sample_it++){
        pose.pose.position.x = traj.multi_dof_joint_trajectory.points[sample_it].transforms[0].translation.x;
        pose.pose.position.y = traj.multi_dof_joint_trajectory.points[sample_it].transforms[0].translation.y;
        path.poses.push_back(pose);
    }

    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    tr_path_pub_.publish(path);
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


//bool MPCC::getTransform(const std::string& from, const std::string& to, Eigen::VectorXd& stamped_pose)
//{
//    bool transform = false;
//    stamped_pose = Eigen::VectorXd(6);
//    tf::StampedTransform stamped_tf;
//
//    // make sure source and target frame exist
//    if (tf_listener_.frameExists(to) && tf_listener_.frameExists(from))
//    {
//        try
//        {
//            // find transforamtion between souce and target frame
//            tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.02));
//            tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);
//
//            // translation
//            stamped_pose(0) = stamped_tf.getOrigin().x();
//            stamped_pose(1) = stamped_tf.getOrigin().y();
//            stamped_pose(2) = stamped_tf.getOrigin().z();
//
//            // convert quternion to rpy
//            tf::Quaternion quat(stamped_tf.getRotation().getX(),
//                                                    stamped_tf.getRotation().getY(),
//                                                    stamped_tf.getRotation().getZ(),
//                                                    stamped_tf.getRotation().getW()
//            );
//
//            if (activate_debug_output_)
//            {
//                std::cout << "\033[94m" << "getTransform:" << " qx:" << stamped_tf.getRotation().getX()
//                                    << "qy:" << stamped_tf.getRotation().getY()
//                                    << "qz:" << stamped_tf.getRotation().getZ()
//                                    << "qw:" << stamped_tf.getRotation().getW() << "\033[0m" <<std::endl;
//            }
//
//            tf::Matrix3x3 quat_matrix(quat);
//            quat_matrix.getRPY(stamped_pose(3), stamped_pose(4), stamped_pose(5));
//
//            if (activate_debug_output_)
//            {
//                std::cout << "\033[32m" << "getTransform:" << " roll:" << stamped_pose(3)
//                                    << " pitch:" << stamped_pose(4)
//                                    << " yaw:" << stamped_pose(5)
//                                    << "\033[0m" <<std::endl;
//            }
//
//            transform = true;
//        }
//        catch (tf::TransformException& ex)
//        {
//            ROS_ERROR("MPCC::getTransform: %s", ex.what());
//        }
//    }
//
//    else
//    {
//        ROS_WARN("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",
//                         from.c_str(), to.c_str());
//    }
//
//    return transform;
//}


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
            stamped_pose.pose.orientation.w = stamped_tf.getRotation().getW();
            stamped_pose.pose.orientation.x = stamped_tf.getRotation().getX();
            stamped_pose.pose.orientation.y = stamped_tf.getRotation().getY();
            stamped_pose.pose.orientation.z = stamped_tf.getRotation().getZ();

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
