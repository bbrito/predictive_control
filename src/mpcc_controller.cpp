
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/mpcc_controller.h>

MPCC::MPCC()
{
    ;
}

MPCC::~MPCC()
{
    clearDataMember();
}

void MPCC::spinNode()
{
    ROS_INFO(" Predictive control node is running, now it's 'Spinning Node'");
    ros::spin();
}

// disallocated memory
void MPCC::clearDataMember()
{
    last_position_ = Eigen::VectorXd(3);
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

        bool kinematic_success = true;

        pd_trajectory_generator_.reset(new pd_frame_tracker());
        bool pd_traj_success = pd_trajectory_generator_->initialize();

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

// update this function 1/clock_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{

	if(((int)traj.multi_dof_joint_trajectory.points.size()) > 1){

		idx = (int)traj.multi_dof_joint_trajectory.points.size() -2;
        goal_pose_(0) = traj.multi_dof_joint_trajectory.points[idx+1].transforms[0].translation.x;
        goal_pose_(1) = traj.multi_dof_joint_trajectory.points[idx+1].transforms[0].translation.y;
        goal_pose_(2) = traj.multi_dof_joint_trajectory.points[idx+1].transforms[0].rotation.z;

        states = pd_trajectory_generator_->solveOptimalControlProblem(current_state_,prev_pose_,next_pose_, goal_pose_, obstacles_, controlled_velocity_);

        publishPredictedTrajectory();
    }

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

        //erase previous trajectory
        for (auto it = traj_marker_array_.markers.begin(); it != traj_marker_array_.markers.end(); ++it)
        {
            it->action = visualization_msgs::Marker::DELETE;
            traj_pub_.publish(traj_marker_array_);
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

    obstacles_ = obstacles;

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

void MPCC::publishPredictedTrajectory(void)
{

	for(int i=0;i<states.getNumPoints();i++){
		state = states.getVector(i);
		//pred_traj_.poses[i].header.stamp = ros::Time::now();
		pred_traj_.poses[i].pose.position.x= state(0);
		pred_traj_.poses[i].pose.position.y= state(1);
	}

	pred_traj_pub_.publish(pred_traj_);
}