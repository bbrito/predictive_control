
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/mpcc_controller.h>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

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

        //Publishers
        traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory",1);
		pred_cmd_pub_ = nh.advertise<visualization_msgs::MarkerArray>("predicted_cmd",1);
        tr_path_pub_ = nh.advertise<nav_msgs::Path>("horizon",1);
        controlled_velocity_pub_ = nh.advertise<geometry_msgs::Twist>(controller_config_->output_cmd,1);
		joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
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
		pred_cmd_.poses.resize(ACADO_N);
		for(int i=0;i < ACADO_N; i++)
		{
			pred_traj_.poses[i].header.frame_id = "odom";
			pred_traj_.header.frame_id = "odom";
		}

		pred_traj_pub_ = nh.advertise<nav_msgs::Path>("mpc_horizon",1);

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

        // initialize state and control weight factors
        cost_state_weight_factors_ = transformStdVectorToEigenVector(controller_config_->lsq_state_weight_factors_);
        cost_control_weight_factors_ = transformStdVectorToEigenVector(controller_config_->lsq_control_weight_factors_);

        cost_state_terminal_weight_factors_ = transformStdVectorToEigenVector(controller_config_->lsq_state_terminal_weight_factors_);

        ros::NodeHandle nh_predictive("predictive_controller");

        /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
        reconfigure_server_.reset(new dynamic_reconfigure::Server<predictive_control::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
        reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));

        computeEgoDiscs();

		//Controller options
		enable_output_ = true;
		n_iterations_ = 1;
		simulation_mode_ = true;

        ROS_WARN("PREDICTIVE CONTROL INTIALIZED!!");
        return true;
    }
    else
    {
        ROS_ERROR("MPCC: Failed to initialize as ROS Node is shoutdown");
        return false;
    }
}

void MPCC::computeEgoDiscs()
{
    // Collect parameters for disc representation
    int n_discs = controller_config_->n_discs_;
    double length = controller_config_->ego_l_;
    double width = controller_config_->ego_w_;

    // Initialize positions of discs
    x_discs_.resize(n_discs);

    // Loop over discs and assign positions
    for ( int discs_it = 0; discs_it < n_discs; discs_it++){
        x_discs_[discs_it] = -length/2 + (discs_it + 1)*(length/(n_discs + 1));
    }

    // Compute radius of the discs
    r_discs_ = sqrt(pow(x_discs_[n_discs - 1] - length/2,2) + pow(width/2,2));
    r_discs_ = 0.001;
    ROS_WARN_STREAM("Generated " << n_discs <<  " ego-vehicle discs with radius " << r_discs_ );
}

void MPCC::broadcastTF(){

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "odom";
	transformStamped.child_frame_id = controller_config_->robot_base_link_;
	transformStamped.transform.translation.x = pred_traj_.poses[1].pose.position.x;
	transformStamped.transform.translation.y = pred_traj_.poses[1].pose.position.y;
	transformStamped.transform.translation.z = 0.0;

	tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
	state_pub_.sendTransform(transformStamped);

	sensor_msgs::JointState empty;
	empty.position.resize(4);
	empty.name ={"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
	empty.header.stamp = ros::Time::now();
	joint_state_pub_.publish(empty);
}

// update this function 1/clock_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{
//    ROS_INFO("RUNNODE");

    int N_iter;
    acado_timer t;
    acado_tic( &t );

    obstacle_feed::Obstacles obstacles = obstacles_;

    int traj_n = traj.multi_dof_joint_trajectory.points.size();
	broadcastTF();
    if (traj_n > 0) {
        goal_pose_(0) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.x;
        goal_pose_(1) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.y;
        goal_pose_(2) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].rotation.z;

        acado_initializeSolver();

        for (N_iter = 0; N_iter < ACADO_N; N_iter++) {
            //
            acadoVariables.x[(ACADO_NX * N_iter) + 0] = current_state_(0);
            acadoVariables.x[(ACADO_NX * N_iter) + 1] = current_state_(1);
            acadoVariables.x[(ACADO_NX * N_iter) + 2] = current_state_(2);
            //acadoVariables.x[(ACADO_NX * N_iter) + 3] = 0;

            acadoVariables.u[(ACADO_NU * N_iter) + 0] = controlled_velocity_.linear.x;
            acadoVariables.u[(ACADO_NU * N_iter) + 1] = controlled_velocity_.angular.z;

            // Initialize Online Data variables
            acadoVariables.od[(ACADO_NOD * N_iter) + 0] = goal_pose_(0);                        // Goal x
            acadoVariables.od[(ACADO_NOD * N_iter) + 1] = goal_pose_(1);                        // Goal y
            acadoVariables.od[(ACADO_NOD * N_iter) + 2] = goal_pose_(2);                        // Goal theta

            acadoVariables.od[(ACADO_NOD * N_iter) + 3] = cost_state_weight_factors_(0);       // weight factor on x
            acadoVariables.od[(ACADO_NOD * N_iter) + 4] = cost_state_weight_factors_(1);       // weight factor on y
            acadoVariables.od[(ACADO_NOD * N_iter) + 5] = cost_state_weight_factors_(2);       // weight factor on theta
            acadoVariables.od[(ACADO_NOD * N_iter) + 6] = cost_control_weight_factors_(0);     // weight factor on v
            acadoVariables.od[(ACADO_NOD * N_iter) + 7] = cost_control_weight_factors_(1);     // weight factor on w

            acadoVariables.od[(ACADO_NOD * N_iter) + 8] = cost_state_terminal_weight_factors_(
                0);  // terminal weight factor on x
            acadoVariables.od[(ACADO_NOD * N_iter) + 9] = cost_state_terminal_weight_factors_(
                1);  // terminal weight factor on y
            acadoVariables.od[(ACADO_NOD * N_iter) + 10] = cost_state_terminal_weight_factors_(
                2);  // terminal weight factor on theta
            acadoVariables.od[(ACADO_NOD * N_iter) + 11] = 0;
            acadoVariables.od[(ACADO_NOD * N_iter) + 12] = r_discs_;                                // radius of car discs
            acadoVariables.od[(ACADO_NOD * N_iter) + 13] = 0;                             // position of the car discs

            acadoVariables.od[(ACADO_NOD * N_iter) + 14] = obstacles.Obstacles[0].pose.position.x;      // x position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 15] = obstacles.Obstacles[0].pose.position.y;      // y position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 16] = obstacles.Obstacles[0].pose.orientation.z;   // heading of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 17] = obstacles.Obstacles[0].major_semiaxis;       // major semiaxis of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 18] = obstacles.Obstacles[0].minor_semiaxis;       // minor semiaxis of obstacle 1
        }

        acadoVariables.x0[0] = current_state_(0);
        acadoVariables.x0[1] = current_state_(1);
        acadoVariables.x0[2] = current_state_(2);
        //acadoVariables.x0[ 3 ] = 0.0001;

        acado_preparationStep();

        acado_feedbackStep();

        controlled_velocity_.linear.x = acadoVariables.u[0];
        controlled_velocity_.angular.z = acadoVariables.u[1];

        printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());

		int j=1;
        while (acado_getKKT()> 1e-2 && j<n_iterations_){

			acado_preparationStep();

            acado_feedbackStep();

            controlled_velocity_.linear.x = acadoVariables.u[0];
            controlled_velocity_.angular.z = acadoVariables.u[1];

            printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());
			j++;    //        acado_printDifferentialVariables();
        }
        publishPredictedTrajectory();

		real_t te = acado_toc(&t);

		ROS_INFO_STREAM("Solve time " << te * 1e6 << " us");

    // publish zero controlled velocity
        if (!tracking_)
        {
            actionSuccess();
        }

        if(!enable_output_)
			publishZeroJointVelocity();
		else
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

    ROS_INFO("reconfigure callback!");
    cost_state_weight_factors_(0) = config.Kx;
    cost_state_weight_factors_(1) = config.Ky;
    cost_state_weight_factors_(2) = config.Ktheta;
    cost_control_weight_factors_(0) = config.Kv;
    cost_control_weight_factors_(1) = config.Kw;

    cost_state_terminal_weight_factors_(0) = config.Px;
    cost_state_terminal_weight_factors_(1) = config.Py;
    cost_state_terminal_weight_factors_(2) = config.Ptheta;

	enable_output_ = config.enable_output;
	n_iterations_ = config.n_iterations;
	simulation_mode_ = config.simulation_mode;
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
	broadcastTF();
    controlled_velocity_ = pub_msg;

    controlled_velocity_pub_.publish(controlled_velocity_);
}

void MPCC::publishPredictedTrajectory(void)
{
    for (int i = 0; i < ACADO_N; i++)
    {
        pred_traj_.poses[i].pose.position.x = acadoVariables.x[i * ACADO_NX + 0]; //x
        pred_traj_.poses[i].pose.position.y = acadoVariables.x[i * ACADO_NX + 1]; //y
		pred_traj_.poses[i].pose.orientation.z = acadoVariables.x[i * ACADO_NX + 2]; //theta
    }

	pred_traj_pub_.publish(pred_traj_);
}

void MPCC::publishPredictedOutput(void)
{
	for (int i = 0; i < ACADO_N; i++)
	{
		pred_cmd_.poses[i].pose.position.x = acadoVariables.u[i * ACADO_NX + 0]; //x
		pred_cmd_.poses[i].pose.position.y = acadoVariables.u[i * ACADO_NX + 1]; //y
	}

	pred_cmd_pub_.publish(pred_cmd_);
}