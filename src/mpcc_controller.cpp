
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/mpcc_controller.h>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

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

        if (controller_config_success == false)
         {
            ROS_ERROR("MPCC: FAILED TO INITILIZED!!");
            std::cout << "States: \n"
                                << " pd_config: " << std::boolalpha << controller_config_success << "\n"
                                //<< " kinematic solver: " << std::boolalpha << kinematic_success << "\n"
                                //<< " collision avoidance: " << std::boolalpha << collision_avoidance_success << "\n"
                                //<< " collision detect: " << std::boolalpha << collision_success << "\n"
                                //<< " static collision avoidance: " << std::boolalpha << static_collision_success << "\n"
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
		pred_traj_.poses.resize(ACADO_N);
		for(int i=0;i < ACADO_N; i++)
		{
			pred_traj_.poses[i].header.frame_id = "odom";
			pred_traj_.header.frame_id = "odom";
		}

		pred_traj_pub_ = nh.advertise<nav_msgs::Path>("mpc_horizon",1);

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

		//Bruno can you fix this?
        // initialize state and control weight factors
        lsq_state_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_state_weight_factors_);
        lsq_control_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_control_weight_factors_);

        lsq_state_terminal_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_state_terminal_weight_factors_);
        lsq_control_terminal_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_control_terminal_weight_factors_);

        /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
        reconfigure_server_.reset(new dynamic_reconfigure::Server<predictive_control::PredictiveControllerConfig>(reconfig_mutex_, nh));
        reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));

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
//    ROS_INFO("RUNNODE");

    int N_iter;
    acado_timer t;
    acado_tic( &t );

    int traj_n = traj.multi_dof_joint_trajectory.points.size();

    if (traj_n > 0) {
        goal_pose_(0) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.x;
        goal_pose_(1) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.y;
        goal_pose_(2) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].rotation.z;

        acado_initializeSolver( );

        for (N_iter = 0; N_iter < ACADO_N; N_iter++)
        {
         //
            acadoVariables.x[ (ACADO_NX * N_iter) + 0 ] = current_state_(0);
            acadoVariables.x[ (ACADO_NX * N_iter) + 1 ] = current_state_(1);
            acadoVariables.x[ (ACADO_NX * N_iter) + 2 ] = current_state_(2);

            acadoVariables.u[ (ACADO_NU * N_iter) + 0 ] = controlled_velocity_.linear.x;
            acadoVariables.u[ (ACADO_NU * N_iter) + 1 ] = controlled_velocity_.angular.z;

            // Initialize Online Data variables
            acadoVariables.od[ (ACADO_NOD * N_iter) + 0 ] = goal_pose_(0);
            acadoVariables.od[ (ACADO_NOD * N_iter) + 1 ] = goal_pose_(1);
            acadoVariables.od[ (ACADO_NOD * N_iter) + 2 ] = goal_pose_(2);

            acadoVariables.od[ (ACADO_NOD * N_iter) + 3 ] = 1;
            acadoVariables.od[ (ACADO_NOD * N_iter) + 4 ] = 1;
            acadoVariables.od[ (ACADO_NOD * N_iter) + 5 ] = 1;
            acadoVariables.od[ (ACADO_NOD * N_iter) + 6 ] = 1;
            acadoVariables.od[ (ACADO_NOD * N_iter) + 7 ] = 1;
        }

        acadoVariables.x0[ 0 ] = current_state_(0);
        acadoVariables.x0[ 1 ] = current_state_(1);
        acadoVariables.x0[ 2 ] = current_state_(2);

        acadoVariables.x0[ 4 ] = controlled_velocity_.linear.x;
        acadoVariables.x0[ 5 ] = controlled_velocity_.angular.z;

        acado_preparationStep();

        acado_feedbackStep();

        controlled_velocity_.linear.x = acadoVariables.u[0];
        controlled_velocity_.angular.z = acadoVariables.u[1];

        real_t te = acado_toc( &t );

        ROS_INFO_STREAM("Solve time " << te*1e6 );

        acado_printDifferentialVariables();

        publishPredictedTrajectory();

    // publish zero controlled velocity
        if (!tracking_)
        {
            actionSuccess();
        }

        //publishZeroJointVelocity();
        controlled_velocity_pub_.publish(controlled_velocity_);
    }
}

void MPCC::moveGoalCB()
{
//    ROS_INFO("MOVEGOALCB");
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
    if(moveit_action_server_->isNewGoalAvailable())
    {
        boost::shared_ptr<const predictive_control::trajGoal> moveit_action_goal_ptr = moveit_action_server_->acceptNewGoal();
        traj = moveit_action_goal_ptr->trajectory;
        tracking_ = false;
        //start trajectory execution
    }
}


void MPCC::reconfigureCallback(predictive_control::PredictiveControllerConfig& config, uint32_t level){

    lsq_state_weight_factors_(0) = config.Kx;
    lsq_state_weight_factors_(1) = config.Ky;
    lsq_state_weight_factors_(2) = config.Ktheta;
    lsq_control_weight_factors_(0) = config.Kv;
    lsq_control_weight_factors_(1) = config.Kw;

    lsq_state_terminal_weight_factors_(0) = config.Px;
    lsq_state_terminal_weight_factors_(1) = config.Py;
    lsq_state_terminal_weight_factors_(2) = config.Ptheta;
    lsq_control_terminal_weight_factors_(0) = config.Pv;
    lsq_control_terminal_weight_factors_(1) = config.Pw;
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
//        ROS_INFO("MPCC::StateCallBack");
    }
    last_state_ = current_state_;
    current_state_(0) =    msg->position.x;
    current_state_(1) =    msg->position.y;
    current_state_(2) =    msg->orientation.z;
}

void MPCC::ObstacleCallBack(const obstacle_feed::Obstacles& obstacles)
{
//    ROS_INFO("OBSTACLECB");

    obstacles_ = obstacles;

}

void MPCC::publishZeroJointVelocity()
{
    if (activate_debug_output_)
    {
//        ROS_INFO("Publishing ZERO joint velocity!!");
    }
    geometry_msgs::Twist pub_msg;

    controlled_velocity_ = pub_msg;

    controlled_velocity_pub_.publish(controlled_velocity_);
}

void MPCC::publishPredictedTrajectory(void)
{
    for (int i = 0; i < ACADO_N; i++)
    {
        pred_traj_.poses[i].pose.position.x = acadoVariables.x[i * ACADO_NX + 0];
        pred_traj_.poses[i].pose.position.y = acadoVariables.x[i * ACADO_NX + 1];
    }

	pred_traj_pub_.publish(pred_traj_);
}