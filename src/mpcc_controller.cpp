
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <predictive_control/mpcc_controller.h>
#include <nav_msgs/Odometry.h>

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
        current_state_ = Eigen::Vector4d(0,0,0,0);
        last_state_ = Eigen::Vector4d(0,0,0,0);
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
		pred_cmd_pub_ = nh.advertise<nav_msgs::Path>("predicted_cmd",1);
		cost_pub_ = nh.advertise<std_msgs::Float64>("cost",1);
		brake_pub_ = nh.advertise<std_msgs::Float64>("break",1);
		contour_error_pub_ = nh.advertise<std_msgs::Float64MultiArray>("contour_error",1);
		controlled_velocity_pub_ = nh.advertise<prius_msgs::Control>(controller_config_->output_cmd,1);
		joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
		robot_collision_space_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/robot_collision_space", 100);
		pred_traj_pub_ = nh.advertise<nav_msgs::Path>("predicted_trajectory",1);
		spline_traj_pub_ = nh.advertise<nav_msgs::Path>("spline_traj",1);
		feedback_pub_ = nh.advertise<predictive_control::control_feedback>("controller_feedback",1);
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
		goal_reached_ = false;
        controlled_velocity_.steer = 0;
        controlled_velocity_.throttle = 0;
        controlled_velocity_.brake = 0;
		last_poly_ = false;

		moveit_msgs::RobotTrajectory j;
		traj = j;

		//initialize trajectory variable to plot prediction trajectory
		spline_traj_.poses.resize(100);
		spline_traj2_.poses.resize(100);
		pred_traj_.poses.resize(ACADO_N);
		pred_cmd_.poses.resize(ACADO_N);
		pred_traj_.header.frame_id = controller_config_->tracking_frame_;
		for(int i=0;i < ACADO_N; i++)
		{
			pred_traj_.poses[i].header.frame_id = controller_config_->tracking_frame_;
		}

		pred_traj_pub_ = nh.advertise<nav_msgs::Path>("mpc_horizon",1);

        //service clients
        reset_simulation_client_ = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
        reset_ekf_client_ = nh.serviceClient<robot_localization::SetPose>("/set_pose");

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

        // initialize state and control weight factors
        cost_contour_weight_factors_ = transformStdVectorToEigenVector(controller_config_->contour_weight_factors_);
        cost_control_weight_factors_ = transformStdVectorToEigenVector(controller_config_->control_weight_factors_);
        slack_weight_ = controller_config_->slack_weight_;
        repulsive_weight_ = controller_config_->repulsive_weight_;
        //velocity_weight_ = controller_config_->velocity_weight_;
        reference_velocity_ = controller_config_->reference_velocity_;

        ros::NodeHandle nh_predictive("predictive_controller");

        /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
        reconfigure_server_.reset(new dynamic_reconfigure::Server<predictive_control::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
        reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));

	    // Initialize obstacles
        obstacle_feed::Obstacles obstacles;
		obstacles.Obstacles.resize(controller_config_->n_obstacles_);
        obstacles_.Obstacles.resize(controller_config_->n_obstacles_);
		for (int obst_it = 0; obst_it < controller_config_->n_obstacles_; obst_it++)
        {
            obstacles.Obstacles[obst_it].pose.position.x = 1000;
            obstacles.Obstacles[obst_it].pose.position.y = 1000;
            obstacles.Obstacles[obst_it].pose.orientation.z = 0;
            obstacles.Obstacles[obst_it].major_semiaxis = 0.001;
            obstacles.Obstacles[obst_it].minor_semiaxis = 0.001;
		}
		obstacles_ = obstacles;

		computeEgoDiscs();

		//Controller options
		enable_output_ = false;
		n_iterations_ = 100;
		simulation_mode_ = true;

		//Plot variables
		ellips1.type = visualization_msgs::Marker::CYLINDER;
		ellips1.id = 60;
		ellips1.color.b = 1.0;
		ellips1.color.a = 0.5;
		ellips1.header.frame_id = controller_config_->tracking_frame_;
		ellips1.ns = "trajectory";
		ellips1.action = visualization_msgs::Marker::ADD;
		ellips1.lifetime = ros::Duration(0.1);
		ellips1.scale.x = r_discs_*2.0;
		ellips1.scale.y = r_discs_*2.0;
		ellips1.scale.z = 0.05;

		// Initialize pregenerated mpc solver
		acado_initializeSolver( );

		// MPCC reference path variables
		X_road.resize(controller_config_->ref_x_.size());
		Y_road.resize(controller_config_->ref_y_.size());
		Theta_road.resize(controller_config_->ref_theta_.size());

		// Check if all reference vectors are of the same length
		if (!( (controller_config_->ref_x_.size() == controller_config_->ref_y_.size()) && ( controller_config_->ref_x_.size() == controller_config_->ref_theta_.size() ) && (controller_config_->ref_y_.size() == controller_config_->ref_theta_.size()) ))
        {
            ROS_ERROR("Reference path inputs should be of equal length");
        }

		traj_i =0;
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
    ROS_WARN_STREAM("Generated " << n_discs <<  " ego-vehicle discs with radius " << r_discs_ );
}

void MPCC::broadcastPathPose(){

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = controller_config_->tracking_frame_;
	transformStamped.child_frame_id = "path";

	transformStamped.transform.translation.x = ref_path_x(acadoVariables.x[4]);
	transformStamped.transform.translation.y = ref_path_y(acadoVariables.x[4]);
	transformStamped.transform.translation.z = 0.0;
	tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
	transformStamped.transform.rotation.x = 0;
	transformStamped.transform.rotation.y = 0;
	transformStamped.transform.rotation.z = 0;
	transformStamped.transform.rotation.w = 1;

	path_pose_pub_.sendTransform(transformStamped);
}

void MPCC::broadcastTF(){

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = controller_config_->target_frame_;
	transformStamped.child_frame_id = controller_config_->robot_base_link_;
	if(!enable_output_){
		transformStamped.transform.translation.x = current_state_(0);
		transformStamped.transform.translation.y = current_state_(1);
		transformStamped.transform.translation.z = 0.0;
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();
	}

	else{
		transformStamped.transform.translation.x = pred_traj_.poses[1].pose.position.x;
		transformStamped.transform.translation.y = pred_traj_.poses[1].pose.position.y;
		transformStamped.transform.translation.z = 0.0;

		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();
	}

	state_pub_.sendTransform(transformStamped);

	sensor_msgs::JointState empty;
	empty.position.resize(7);
	empty.name ={"rear_right_wheel_joint", "rear_left_wheel_joint", "front_right_wheel_joint", "front_left_wheel_joint","front_right_steer_joint","front_left_steer_joint","steering_joint"};
	empty.header.stamp = ros::Time::now();
	joint_state_pub_.publish(empty);
}

// update this function 1/clock_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{
    int N_iter;
    acado_timer t;
    acado_tic( &t );
    acado_initializeSolver( );

    int traj_n = traj.multi_dof_joint_trajectory.points.size();
	if(!simulation_mode_)
		broadcastTF();
    if (traj_n > 0) {

        if (simulation_mode_) {
            acadoVariables.x[0] = current_state_(0);
            acadoVariables.x[1] = current_state_(1);
            acadoVariables.x[2] = current_state_(2);
            acadoVariables.x[3] = current_state_(3);             //it should be obtained by the wheel speed
            acadoVariables.x0[0] = current_state_(0);
            acadoVariables.x0[1] = current_state_(1);
            acadoVariables.x0[2] = current_state_(2);
            acadoVariables.x0[3] = current_state_(3);             //it should be obtained by the wheel speed
        } else {
            if (enable_output_) {
                acadoVariables.x[0] = acadoVariables.x[0 + ACADO_NX];
                acadoVariables.x[1] = acadoVariables.x[1 + ACADO_NX];
                acadoVariables.x[2] = acadoVariables.x[2 + ACADO_NX];
                acadoVariables.x[3] = acadoVariables.x[3 + ACADO_NX];             //it should be obtained by the wheel speed
                acadoVariables.x0[0] = acadoVariables.x[0 + ACADO_NX];
                acadoVariables.x0[1] = acadoVariables.x[1 + ACADO_NX];
                acadoVariables.x0[2] = acadoVariables.x[2 + ACADO_NX];
                acadoVariables.x0[3] = acadoVariables.x[3 + ACADO_NX];             //it should be obtained by the wheel speed
            } else {
                acadoVariables.x[0] = acadoVariables.x[0];
                acadoVariables.x[1] = acadoVariables.x[1];
                acadoVariables.x[2] = acadoVariables.x[2];
                acadoVariables.x[3] = acadoVariables.x[3];             //it should be obtained by the wheel speed
                acadoVariables.x0[0] = acadoVariables.x[0];
                acadoVariables.x0[1] = acadoVariables.x[1];
                acadoVariables.x0[2] = acadoVariables.x[2];
                acadoVariables.x0[3] = acadoVariables.x[3];             //it should be obtained by the wheel speed
            }
        }
        ROS_WARN_STREAM("ss.size():" << ss.size() << " traj_i: " << traj_i);
        if (acadoVariables.x[4] > ss[traj_i + 1]) {

            if (traj_i + 2 == ss.size()) {
                goal_reached_ = true;
                ROS_ERROR_STREAM("GOAL REACHED");
            } else {
                traj_i++;
                ROS_ERROR_STREAM("SWITCH SPLINE " << acadoVariables.x[4]);
            }
        }

        if(idx ==1) {
            double smin;
            smin = spline_closest_point(ss[traj_i], 1000, acadoVariables.x[ACADO_NX+4], window_size_, n_search_points_);
            acadoVariables.x[4] = smin;
            acadoVariables.x0[4] = smin;
            ROS_ERROR_STREAM("smin: " << smin);
            ROS_ERROR_STREAM("smin: " << ss[traj_i]);
            ROS_ERROR_STREAM("smin: " << ss[traj_i+1]);
        }
        else
            acadoVariables.x[4] = acadoVariables.x[4];

        acadoVariables.u[0] = controlled_velocity_.throttle;
        acadoVariables.u[1] = controlled_velocity_.steer;
        //acadoVariables.u[2] = 0.0000001;           //slack variable

        for (N_iter = 0; N_iter < ACADO_N; N_iter++) {


            acadoVariables.od[(ACADO_NOD * N_iter) + 0] = ref_path_x.m_a[traj_i];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 1] = ref_path_x.m_b[traj_i];
            acadoVariables.od[(ACADO_NOD * N_iter) + 2] = ref_path_x.m_c[traj_i];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 3] = ref_path_x.m_d[traj_i];
            acadoVariables.od[(ACADO_NOD * N_iter) + 4] = ref_path_y.m_a[traj_i];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 5] = ref_path_y.m_b[traj_i];
            acadoVariables.od[(ACADO_NOD * N_iter) + 6] = ref_path_y.m_c[traj_i];        // spline coefficients
            acadoVariables.od[(ACADO_NOD * N_iter) + 7] = ref_path_y.m_d[traj_i];

            acadoVariables.od[(ACADO_NOD * N_iter) + 16] = ss[traj_i];       // s1
            acadoVariables.od[(ACADO_NOD * N_iter) + 17] = ss[traj_i + 1];       //s2
            acadoVariables.od[(ACADO_NOD * N_iter) + 18] = ss[traj_i + 1] + 0.02;       // d
            acadoVariables.od[(ACADO_NOD * N_iter) + 19] = cost_contour_weight_factors_(0);     // weight contour error
            acadoVariables.od[(ACADO_NOD * N_iter) + 20] = cost_contour_weight_factors_(1);     // weight lag error
            acadoVariables.od[(ACADO_NOD * N_iter) + 21] = cost_control_weight_factors_(0);    // weight acceleration
            acadoVariables.od[(ACADO_NOD * N_iter) + 22] = cost_control_weight_factors_(1);   // weight delta
            acadoVariables.od[(ACADO_NOD * N_iter) + 25] = slack_weight_;                     //slack weight
            acadoVariables.od[(ACADO_NOD * N_iter) + 26] = repulsive_weight_;                     //repulsive weight
            acadoVariables.od[(ACADO_NOD * N_iter) + 40] = velocity_weight_;                     //repulsive weight
            
            acadoVariables.od[(ACADO_NOD * N_iter) + 27] = r_discs_; //radius of the disks
            acadoVariables.od[(ACADO_NOD * N_iter) + 28] = x_discs_[0];                        // position of the car discs
            acadoVariables.od[(ACADO_NOD * N_iter) + 41] = x_discs_[1];                        // position of the car discs
            acadoVariables.od[(ACADO_NOD * N_iter) + 42] = x_discs_[2];                        // position of the car discs
            
            acadoVariables.od[(ACADO_NOD * N_iter) + 29] = obstacles_.Obstacles[0].pose.position.x;      // x position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 30] = obstacles_.Obstacles[0].pose.position.y;      // y position of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 31] = obstacles_.Obstacles[0].pose.orientation.z;   // heading of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 32] = obstacles_.Obstacles[0].major_semiaxis;       // major semiaxis of obstacle 1
            acadoVariables.od[(ACADO_NOD * N_iter) + 33] = obstacles_.Obstacles[0].minor_semiaxis;       // minor semiaxis of obstacle 1
           

            acadoVariables.od[(ACADO_NOD * N_iter) + 34] = obstacles_.Obstacles[1].pose.position.x;      // x position of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 35] = obstacles_.Obstacles[1].pose.position.y;      // y position of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 36] = obstacles_.Obstacles[1].pose.orientation.z;   // heading of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 37] = obstacles_.Obstacles[1].major_semiaxis;       // major semiaxis of obstacle 2
            acadoVariables.od[(ACADO_NOD * N_iter) + 38] = obstacles_.Obstacles[1].minor_semiaxis;       // minor semiaxis of obstacle 2
   
            


            if (goal_reached_) {
                acadoVariables.od[(ACADO_NOD * N_iter) + 23] = 0;
                acadoVariables.od[(ACADO_NOD * N_iter) + 24] = 0;        
                acadoVariables.od[(ACADO_NOD * N_iter) + 8] =  0;
                acadoVariables.od[(ACADO_NOD * N_iter) + 9] =  0;
                acadoVariables.od[(ACADO_NOD * N_iter) + 10] = 0;        // spline coefficients
                acadoVariables.od[(ACADO_NOD * N_iter) + 11] = ref_path_x(ss[traj_i]);
                acadoVariables.od[(ACADO_NOD * N_iter) + 12] = 0;        // spline coefficients
                acadoVariables.od[(ACADO_NOD * N_iter) + 13] = 0;
                acadoVariables.od[(ACADO_NOD * N_iter) + 14] = 0;        // spline coefficients
                acadoVariables.od[(ACADO_NOD * N_iter) + 15] = ref_path_y(ss[traj_i]);
            } else {
                acadoVariables.od[(ACADO_NOD * N_iter) + 23] = reference_velocity_;       
                acadoVariables.od[(ACADO_NOD * N_iter) + 24] = reference_velocity_;     
                acadoVariables.od[(ACADO_NOD * N_iter) + 8] = ref_path_x.m_a[traj_i + 1];        // spline coefficients
                acadoVariables.od[(ACADO_NOD * N_iter) + 9] = ref_path_x.m_b[traj_i + 1];
                acadoVariables.od[(ACADO_NOD * N_iter) + 10] = ref_path_x.m_c[traj_i + 1];        // spline coefficients
                acadoVariables.od[(ACADO_NOD * N_iter) + 11] = ref_path_x.m_d[traj_i + 1];
                acadoVariables.od[(ACADO_NOD * N_iter) + 12] = ref_path_y.m_a[traj_i + 1];        // spline coefficients
                acadoVariables.od[(ACADO_NOD * N_iter) + 13] = ref_path_y.m_b[traj_i + 1];
                acadoVariables.od[(ACADO_NOD * N_iter) + 14] = ref_path_y.m_c[traj_i + 1];        // spline coefficients
                acadoVariables.od[(ACADO_NOD * N_iter) + 15] = ref_path_y.m_d[traj_i + 1];
            }

            //acadoVariables.od[(ACADO_NOD * N_iter) + 23] = ss[traj_i + 1] + 0.02;
        }

        acado_preparationStep();

        acado_feedbackStep();

        printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());

        int j = 1;
        while (acado_getKKT() > 1e-3 && j < n_iterations_) {

            acado_preparationStep();

            acado_feedbackStep();

            printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());

            j++;    //        acado_printDifferentialVariables();
        }
        te_ = acado_toc(&t);
        if (acadoVariables.u[0] < 0) {
            controlled_velocity_.brake = -1.0 * acadoVariables.u[0];// / (-4.0); // maximum brake
            controlled_velocity_.throttle = 0.0;
        } else {
            controlled_velocity_.throttle = acadoVariables.u[0];// / 1.5; // maximum acceleration 1.5m/s
            controlled_velocity_.brake = 0.0;
        }

        controlled_velocity_.steer = acadoVariables.u[1] / 0.52; // maximum steer

        publishPredictedTrajectory();
        publishPredictedCollisionSpace();
        publishPredictedOutput();
        broadcastPathPose();
        brake_.data = controlled_velocity_.brake;
        cost_.data = acado_getObjective();
        publishCost();

    }
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

double MPCC::spline_closest_point(double s_min, double s_max, double s_guess, double window, int n_tries){

	double lower = std::max(s_min, s_guess-window);
	double upper = std::min(s_max, s_guess + window);
	double s_i=lower,spline_pos_x_i,spline_pos_y_i;
	double dist_i,min_dist,smin=0.0;

	spline_pos_x_i = ref_path_x(s_i);
	spline_pos_y_i = ref_path_y(s_i);

	min_dist = std::sqrt((spline_pos_x_i-current_state_(0))*(spline_pos_x_i-current_state_(0))+(spline_pos_y_i-current_state_(1))*(spline_pos_y_i-current_state_(1)));

	for(int i=0;i<n_tries;i++){
		s_i = lower+(upper-lower)/n_tries*i;
		spline_pos_x_i = ref_path_x(s_i);
		spline_pos_y_i = ref_path_y(s_i);
		dist_i = std::sqrt((spline_pos_x_i-current_state_(0))*(spline_pos_x_i-current_state_(0))+(spline_pos_y_i-current_state_(1))*(spline_pos_y_i-current_state_(1)));

		if(dist_i<min_dist){
			min_dist = dist_i;
			smin = s_i;
		}

	}
    if(smin < lower){
        smin=lower;
    }
    if(smin > upper){
        smin=upper;
    }

	return smin;

}

void MPCC::moveGoalCB()
{
//    ROS_INFO("MOVEGOALCB");
    if(move_action_server_->isNewGoalAvailable())
    {
        boost::shared_ptr<const predictive_control::moveGoal> move_action_goal_ptr = move_action_server_->acceptNewGoal();
        tracking_ = false;
    }
}

void MPCC::Ref_path(std::vector<double> x,std::vector<double> y, std::vector<double> theta) {

    double k, dk, L;
    std::vector<double> X(10), Y(10);
    std::vector<double> X_all, Y_all, S_all;
    total_length_= 0;
    n_clothoid = controller_config_->n_points_clothoid_;
    n_pts = controller_config_->n_points_spline_;
    S_all.push_back(0);

    for (int i = 0; i < x.size()-1; i++){
        Clothoid::buildClothoid(x[i], y[i], theta[i], x[i+1], y[i+1], theta[i+1], k, dk, L);

        Clothoid::pointsOnClothoid(x[i], y[i], theta[i], k, dk, L, n_clothoid, X, Y);
        if (i==0){
            X_all.insert(X_all.end(), X.begin(), X.end());
            Y_all.insert(Y_all.end(), Y.begin(), Y.end());
        }
        else{
            X.erase(X.begin()+0);
            Y.erase(Y.begin()+0);
            X_all.insert(X_all.end(), X.begin(), X.end());
            Y_all.insert(Y_all.end(), Y.begin(), Y.end());
        }
        total_length_ += L;
        for (int j=1; j< n_clothoid; j++){
            S_all.push_back(S_all[j-1+i*(n_clothoid-1)]+L/(n_clothoid-1));
            //ROS_INFO_STREAM("S_all: " << S_all[j]);
        }
        //ROS_INFO_STREAM("X_all: " << X_all[i]);
        //ROS_INFO_STREAM("Y_all: " << Y_all[i]);
    }

    ref_path_x.set_points(S_all, X_all);
    ref_path_y.set_points(S_all, Y_all);

    dist_spline_pts_ = total_length_ / (n_pts - 1);
    //ROS_INFO_STREAM("dist_spline_pts_: " << dist_spline_pts_);
    ss.resize(n_pts);
    xx.resize(n_pts);
    yy.resize(n_pts);

    for (int i=0; i<n_pts; i++){
        ss[i] = dist_spline_pts_ *i;
        xx[i] = ref_path_x(ss[i]);
        yy[i] = ref_path_y(ss[i]);
        //ROS_INFO_STREAM("ss: " << ss[i]);
        //ROS_INFO_STREAM("xx: " << xx[i]);
        //ROS_INFO_STREAM("yy: " << yy[i]);
    }

    ref_path_x.set_points(ss,xx);
    ref_path_y.set_points(ss,yy);
}

void MPCC::ConstructRefPath(){

    for (int ref_point_it = 0; ref_point_it < controller_config_->ref_x_.size(); ref_point_it++)
    {
        X_road[ref_point_it] = controller_config_->ref_x_.at(ref_point_it);
        Y_road[ref_point_it] = controller_config_->ref_y_.at(ref_point_it);
        Theta_road[ref_point_it] = controller_config_->ref_theta_.at(ref_point_it);
    }

    Ref_path(X_road, Y_road, Theta_road);
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

        int traj_n = traj.multi_dof_joint_trajectory.points.size();
        goal_pose_(0) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.x;
        goal_pose_(1) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].translation.y;
        goal_pose_(2) = traj.multi_dof_joint_trajectory.points[traj_n - 1].transforms[0].rotation.z;

        acado_initializeSolver( );

        int N_iter;
	for (N_iter = 0; N_iter < ACADO_N; N_iter++) {

            // Initialize Constant Online Data variables
            acadoVariables.od[(ACADO_NOD * N_iter) + 27] = r_discs_;                                // radius of car discs
            acadoVariables.od[(ACADO_NOD * N_iter) + 28] = 0; //x_discs_[1];                        // position of the car discs
        }

        traj_i = 0;
		goal_reached_ = false;
		last_poly_ = false;
        ConstructRefPath();

		publishSplineTrajectory();
    }
}


void MPCC::reconfigureCallback(predictive_control::PredictiveControllerConfig& config, uint32_t level){

	ROS_INFO("reconfigure callback!");
	cost_contour_weight_factors_(0) = config.Wcontour;
	cost_contour_weight_factors_(1) = config.Wlag;
	cost_control_weight_factors_(0) = config.Ka;
	cost_control_weight_factors_(1) = config.Kdelta;
	velocity_weight_ = config.Kv;

	slack_weight_= config.Ws;
	repulsive_weight_ = config.WR;

	reference_velocity_ = config.vRef;
   	slack_weight_= config.Ws;
	repulsive_weight_ = config.WR;

	enable_output_ = config.enable_output;
	n_iterations_ = config.n_iterations;
	simulation_mode_ = config.simulation_mode;

	//Search window parameters
	window_size_ = config.window_size;
	n_search_points_ = config.n_search_points;

	reset_world_ = config.reset_world;
	if(reset_world_) {
	    reset_simulation_client_.call(reset_msg_);
	    reset_ekf_client_.call(reset_pose_msg_);
	    acadoVariables.x[0] = 0;
	    acadoVariables.x[1] = 0;
		acadoVariables.x[2] = 0;
        acadoVariables.x[3] = 0;             //it should be obtained by the wheel speed
        acadoVariables.x0[0] = 0;
        acadoVariables.x0[1] = 0;
	    acadoVariables.x0[2] = 0;
        acadoVariables.x0[3] = 0;
        acadoVariables.x[ACADO_NX+4] = 0;
        traj_i=0;
        acado_initializeSolver();
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
void MPCC::StateCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{

    if (activate_debug_output_)
    {
//  ROS_INFO("MPCC::StateCallBack");
    }
    //Intermidiate variables
    double ysqr, t3, t4;

    last_state_ = current_state_;
    current_state_(0) =    msg->pose.pose.position.x;
    current_state_(1) =    msg->pose.pose.position.y;
    //ROS_INFO_STREAM("current_state_(0): " << current_state_(0));
    // Convert quaternion to angle

    ysqr = msg->pose.pose.orientation.y * msg->pose.pose.orientation.y;
    t3 = +2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z
                 + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    t4 = +1.0 - 2.0 * (ysqr + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

    current_state_(2) = std::atan2(t3, t4);

    current_state_(3) =    sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));

}

void MPCC::ObstacleCallBack(const obstacle_feed::Obstacles& obstacles)
{

    obstacle_feed::Obstacles total_obstacles;
    total_obstacles.Obstacles.resize(controller_config_->n_obstacles_);

    total_obstacles.Obstacles = obstacles.Obstacles;

//    ROS_INFO_STREAM("-- Received # obstacles: " << obstacles.Obstacles.size());
//    ROS_INFO_STREAM("-- Expected # obstacles: " << controller_config_->n_obstacles_);

    if (obstacles.Obstacles.size() < controller_config_->n_obstacles_)
    {
        for (int obst_it = obstacles.Obstacles.size(); obst_it < controller_config_->n_obstacles_; obst_it++)
        {
            total_obstacles.Obstacles[obst_it].pose.position.x = 1000;
            total_obstacles.Obstacles[obst_it].pose.position.y = 1000;
            total_obstacles.Obstacles[obst_it].pose.orientation.z = 0;
            total_obstacles.Obstacles[obst_it].major_semiaxis = 0.001;
            total_obstacles.Obstacles[obst_it].minor_semiaxis = 0.001;
        }
    }

    obstacles_.Obstacles.resize(controller_config_->n_obstacles_);

    for (int total_obst_it = 0; total_obst_it < controller_config_->n_obstacles_; total_obst_it++)
    {
        obstacles_.Obstacles[total_obst_it] = total_obstacles.Obstacles[total_obst_it];
    }

//    ROS_INFO_STREAM("-- total_Obst1: [" << total_obstacles.Obstacles[0].pose.position.x << ",  " << total_obstacles.Obstacles[0].pose.position.y << "], Obst2 [" << total_obstacles.Obstacles[1].pose.position.x << ",  " << total_obstacles.Obstacles[1].pose.position.y << "]");
//    ROS_INFO_STREAM("-- Obst1_: [" << obstacles_.Obstacles[0].pose.position.x << ",  " << obstacles_.Obstacles[0].pose.position.y << "], Obst2 [" << obstacles_.Obstacles[1].pose.position.x << ",  " << obstacles_.Obstacles[1].pose.position.y << "]");
}

void MPCC::publishZeroJointVelocity()
{
    if (activate_debug_output_)
    {
//        ROS_INFO("Publishing ZERO joint velocity!!");
    }
	prius_msgs::Control pub_msg;
	if(!simulation_mode_)
		broadcastTF();
    controlled_velocity_ = pub_msg;
    controlled_velocity_.brake = 10.0;
    controlled_velocity_pub_.publish(controlled_velocity_);
}

void MPCC::publishSplineTrajectory(void)
{
	spline_traj_.header.stamp = ros::Time::now();
	spline_traj_.header.frame_id = controller_config_->tracking_frame_;
	for (int i = 0; i < 100; i++) // 100 points
	{
		spline_traj_.poses[i].pose.position.x = ref_path_x(i*(n_pts-1)*dist_spline_pts_/100.0); //x
		spline_traj_.poses[i].pose.position.y = ref_path_y(i*(n_pts-1)*dist_spline_pts_/100.0); //y

	}

	spline_traj_pub_.publish(spline_traj_);
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
		pred_cmd_.poses[i].pose.position.x = acadoVariables.u[i + 0]; //x
		pred_cmd_.poses[i].pose.position.y = acadoVariables.u[i + 1]; //y
	}

	pred_cmd_pub_.publish(pred_cmd_);
}

void MPCC::publishPredictedCollisionSpace(void)
{
	visualization_msgs::MarkerArray collision_space;


	for (int i = 0; i < ACADO_N; i++)
	{
		ellips1.id = 60+i;
		ellips1.pose.position.x = acadoVariables.x[i * ACADO_NX + 0];
		ellips1.pose.position.y = acadoVariables.x[i * ACADO_NX + 1];
		ellips1.pose.orientation.x = 0;
		ellips1.pose.orientation.y = 0;
		ellips1.pose.orientation.z = 0;
		ellips1.pose.orientation.w = 1;
		collision_space.markers.push_back(ellips1);
	}

	robot_collision_space_pub_.publish(collision_space);
}

void MPCC::publishCost(void){

	cost_pub_.publish(cost_);
	brake_pub_.publish(brake_);
}
