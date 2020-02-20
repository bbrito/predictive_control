
//This file containts read parameter from server, callback, call class objects, control all class, objects of all class

#include <lmpcc/mpcc_controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <FORCESNLPsolver.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/contrib/contrib.hpp>
//#include <opencv2/highgui/highgui.hpp>


#define FORCES_N 20 // model.N, horizon length
#define FORCES_NU 3 //number of control variables
#define FORCES_NX 6 // differentiable variables
#define FORCES_TOTAL_V 9 //Total control and differentiable
#define FORCES_NPAR 340 //Total number of parameters for one horizon step


FORCESNLPsolver_params forces_params; // 50 X 9   [acc   delta  sv     x      y       psi   v    s dummy X 50]
FORCESNLPsolver_output forces_output;
FORCESNLPsolver_info forces_info;

//This is different from the exammple given in high level interface
// as this requires calling a function from C which has been compiled as a library

extern "C" {
extern void FORCESNLPsolver_casadi2forces( FORCESNLPsolver_float *x,        /* primal vars                                         */
                                                 FORCESNLPsolver_float *y,        /* eq. constraint multiplers                           */
                                                 FORCESNLPsolver_float *l,        /* ineq. constraint multipliers                        */
                                                 FORCESNLPsolver_float *p,        /* parameters                                          */
                                                 FORCESNLPsolver_float *f,        /* objective function (scalar)                         */
                                                 FORCESNLPsolver_float *nabla_f,  /* gradient of objective function                      */
                                                 FORCESNLPsolver_float *c,        /* dynamics                                            */
                                                 FORCESNLPsolver_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                                 FORCESNLPsolver_float *h,        /* inequality constraints                              */
                                                 FORCESNLPsolver_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                                 FORCESNLPsolver_float *hess,     /* Hessian (column major)                              */
                                                 solver_int32_default stage,     /* stage number (0 indexed)                            */
                                                 solver_int32_default iteration /* iteration number of solver                          */);
FORCESNLPsolver_extfunc extfunc_eval = &FORCESNLPsolver_casadi2forces;
}


MPCC::~MPCC()
{
}

void MPCC::spinNode()
{
    ROS_INFO(" Predictive control node is running, now it's 'Spinning Node'");
    ros::spin();
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

        if (controller_config_success == false)
        {
            ROS_ERROR("MPCC: FAILED TO INITILIZED!!");
            std::cout << "States: \n"
                      << " pd_config: " << std::boolalpha << controller_config_success << "\n"
                      << " pd config init success: " << std::boolalpha << controller_config_->initialize_success_
                      << std::endl;
            return false;
        }

        //ROS_INFO("initialize data member of class");
        clock_frequency_ = controller_config_->clock_frequency_;

        //DEBUG
        activate_debug_output_ = controller_config_->activate_debug_output_;
        plotting_result_ = controller_config_->plotting_result_;

        // DEBUG
        if (controller_config_->activate_controller_node_output_)
        {
            ROS_WARN("===== DEBUG INFO ACTIVATED =====");
        }

        // resize position and velocity velocity vectors
        current_state_ = Eigen::Vector4d(0,0,0,0);
        last_state_ = Eigen::Vector4d(0,0,0,0);

        waypoints_size_ = 0; // min 2 waypoints

        robot_state_sub_ = nh.subscribe(controller_config_->robot_state_topic_, 1, &MPCC::StateCallBack, this);
        obstacle_feed_sub_ = nh.subscribe(controller_config_->sub_ellipse_topic_, 1, &MPCC::ObstacleCallBack, this);

        //obstacles_state_sub_ = nh.subscribe(controller_config_->obs_state_topic_, 1, &MPCC::ObstacleStateCallback, this);
        ped_stop_sub_ = nh.subscribe("/MultiPedestrianDBN_node/probabilities",1, &MPCC::pedStopCallBack, this);
        waypoints_sub_ = nh.subscribe(controller_config_->waypoint_topic_,1, &MPCC::getWayPointsCallBack, this);

        //Publishers
        traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("pd_trajectory",1);
        pred_cmd_pub_ = nh.advertise<nav_msgs::Path>("predicted_cmd",1);
        cost_pub_ = nh.advertise<std_msgs::Float64>("cost",1);
        brake_pub_ = nh.advertise<std_msgs::Float64>("break",1);
        contour_error_pub_ = nh.advertise<std_msgs::Float64MultiArray>("contour_error",1);

        controlled_velocity_pub_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>(controller_config_->cmd_,1);
        joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
        robot_collision_space_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/robot_collision_space", 100);
        pred_traj_pub_ = nh.advertise<nav_msgs::Path>("predicted_trajectory",1);
        spline_traj_pub_ = nh.advertise<nav_msgs::Path>("spline_traj",1);
        feedback_pub_ = nh.advertise<lmpcc::control_feedback>("controller_feedback",1);
        //Road publisher
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("road", 10);
        ros::Duration(1).sleep();
        timer_ = nh.createTimer(ros::Duration(1/clock_frequency_), &MPCC::runNode, this);
       

        //Initialize trajectory variables
        next_point_dist = 0;
        goal_dist = 0;
        prev_point_dist = 0;

        goal_reached_ = false;
        controlled_velocity_.steer = 0;
        controlled_velocity_.throttle = 0;
        controlled_velocity_.brake = 0;

        //ROS_INFO("initialize trajectory variable to plot prediction trajectory");
        spline_traj_.poses.resize(100);
        spline_traj2_.poses.resize(100);
        pred_traj_.poses.resize(FORCES_N);
        pred_cmd_.poses.resize(FORCES_N);
        pred_traj_.header.frame_id = controller_config_->target_frame_;
        for(int i=0;i < FORCES_N; i++)
        {
            pred_traj_.poses[i].header.frame_id = controller_config_->target_frame_;
        }

        pred_traj_pub_ = nh.advertise<nav_msgs::Path>("mpc_horizon",1);

        //service clients
        //reset_simulation_client_ = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
        //reset_ekf_client_ = nh.serviceClient<robot_localization::SetPose>("/set_pose");
        update_trigger = nh.serviceClient<lmpcc_msgs::IntTrigger>("update_trigger_int");
        obstacle_trigger.request.value = (int) clock_frequency_;

        ROS_INFO("initialize state and control weight factors");
        cost_contour_weight_factors_ = transformStdVectorToEigenVector(controller_config_->contour_weight_factors_);
        cost_control_weight_factors_ = transformStdVectorToEigenVector(controller_config_->control_weight_factors_);
        slack_weight_ = controller_config_->slack_weight_;
        repulsive_weight_ = controller_config_->repulsive_weight_;
        reference_velocity_ = controller_config_->reference_velocity_;
        reduced_reference_velocity_ = reference_velocity_;
        ini_vel_x_ = controller_config_->ini_vel_x_;
        ros::NodeHandle nh_predictive("predictive_controller");

        ROS_INFO("Setting up dynamic_reconfigure server for the TwistControlerConfig parameters");
        reconfigure_server_.reset(new dynamic_reconfigure::Server<lmpcc::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
        reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));
        
        // Initialize obstacles
        int N = FORCES_N; // hack.. needs to be beter computed
        obstacles_.lmpcc_obstacles.resize(controller_config_->n_obstacles_);
        for (int obst_it = 0; obst_it < controller_config_->n_obstacles_; obst_it++)
        {
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses.resize(N);
            obstacles_.lmpcc_obstacles[obst_it].major_semiaxis.resize(N);
            obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis.resize(N);
            for(int t = 0;t<N;t++){
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.x = 10000;
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.y = 10000;
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.orientation.z = 0;
                obstacles_.lmpcc_obstacles[obst_it].major_semiaxis[t] = 0.001;
                obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis[t] = 0.001;
            }
        }
        computeEgoDiscs();

        //Controller options
        enable_output_ = false;
        plan_ = false;
        replan_ = false;
        left_offset_= 0;
        right_offset_= 0;
        debug_ = false;
        n_iterations_ = 100;
        simulation_mode_ = true;
        bb_hack_ =0;
        stop_likelihood_=1;
        //Plot variables
        ellips1.type = visualization_msgs::Marker::CYLINDER;
        ellips1.id = 60;
        ellips1.color.b = 1.0;
        ellips1.color.a = 0.6;
        ellips1.header.frame_id = controller_config_->target_frame_;
        ellips1.ns = "trajectory";
        ellips1.action = visualization_msgs::Marker::ADD;
        ellips1.lifetime = ros::Duration(0.1);
        ellips1.scale.x = r_discs_*2.0;
        ellips1.scale.y = r_discs_*2.0;
        ellips1.scale.z = 0.05;

        // Check if all reference vectors are of the same length
        if (!( (controller_config_->ref_x_.size() == controller_config_->ref_y_.size()) && ( controller_config_->ref_x_.size() == controller_config_->ref_theta_.size() ) && (controller_config_->ref_y_.size() == controller_config_->ref_theta_.size()) ))
        {
            ROS_ERROR("Reference path inputs should be of equal length");
            return false;
        }

        // MPCC path variables
        X_road.resize(controller_config_->ref_x_.size());
        Y_road.resize(controller_config_->ref_y_.size());
        Theta_road.resize(controller_config_->ref_theta_.size());

        ROS_WARN_STREAM("Getting waypoints from config file..."<<controller_config_->ref_x_.size());
        waypoints_size_ = controller_config_->ref_x_.size();
        for (int ref_point_it = 0; ref_point_it<waypoints_size_; ref_point_it++)
        {
            X_road[ref_point_it] = controller_config_->ref_x_[ref_point_it];
            Y_road[ref_point_it] = controller_config_->ref_y_[ref_point_it];
            Theta_road[ref_point_it] = controller_config_->ref_theta_[ref_point_it];
        }
        Ref_path(X_road, Y_road, Theta_road);
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

void MPCC::pedStopCallBack(const ros_intent_slds::FloatArrayStamped& msg){
    stop_likelihood_ = msg.data[4];
    stop_likelihood_ = 1.0;
    ROS_INFO_STREAM("stop_likelihood_: " << stop_likelihood_);
};

void MPCC::computeEgoDiscs()
{
    // Collect parameters for disc representation
    int n_discs = controller_config_->n_discs_;
    double length = controller_config_->ego_l_;
    double width = controller_config_->ego_w_;
    double com_to_back = 2.4; // distance center of mass to back of the car

    // Initialize positions of discs
    x_discs_.resize(n_discs);


    // Compute radius of the discs
    r_discs_ = width/2;
    // Loop over discs and assign positions, with respect to center of mass
    for ( int discs_it = 0; discs_it < n_discs; discs_it++){

        if (n_discs == 1) { // if only 1 disc, position in center;
            x_discs_[discs_it] = -com_to_back+length/2;
        }
        else if(discs_it == 0){ // position first disc so it touches the back of the car
            x_discs_[discs_it] = -com_to_back+r_discs_;
        }
        else if(discs_it == n_discs-1){
            x_discs_[discs_it] = -com_to_back+length-r_discs_;
        }
        else {
            x_discs_[discs_it] = -com_to_back + r_discs_ + discs_it*(length-2*r_discs_)/(n_discs-1) ;
        }

        // x_discs_[discs_it] = -(discs_it )*(length/(n_discs)); //old distribution (was still starting at front frame)

    }


    ROS_WARN_STREAM("Generated " << n_discs <<  " ego-vehicle discs with radius " << r_discs_ );
    ROS_INFO_STREAM(x_discs_);
}

void MPCC::broadcastPathPose(){

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = controller_config_->target_frame_;
    transformStamped.child_frame_id = "path";

    transformStamped.transform.translation.x = ref_path_x(forces_params.x0[7]);
    transformStamped.transform.translation.y = ref_path_y(forces_params.x0[7]);
    transformStamped.transform.translation.z = 0.0;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pred_traj_.poses[1].pose.orientation.z);
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    path_pose_pub_.sendTransform(transformStamped);
}

void MPCC::broadcastTF(){
    // Only used for perfect state simulation
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

void  MPCC::reset_solver(){

    for (int i = 0; i < *(&forces_params.all_parameters + 1) - forces_params.all_parameters; i++){
        forces_params.all_parameters[i] = 0.0;
    }

    for (int i = 0; i < *(&forces_params.xinit + 1) - forces_params.xinit; i++){
        forces_params.xinit[i] = 0.0;
    }
    int k = 0;
    for (int i = 0; i < FORCES_N*FORCES_TOTAL_V; i++){
        forces_params.x0[k] = 0.0;
        k++;
    }
}



// update this function 1/clock_frequency
void MPCC::runNode(const ros::TimerEvent &event)
{
    ROS_INFO("Start of runNode");
    int N_iter;
    int exit_code = 0;
    float obstacle_distance1, obstacle_distance2;
    if(event.profile.last_duration.nsec > 1/clock_frequency_*1e9)
        ROS_ERROR_STREAM("Real-time constraint not satisfied... Cycle time: " << event.profile.last_duration);

    if(!simulation_mode_)
        broadcastTF();
    if (plan_ && (waypoints_size_ > 0)) {

        forces_params.xinit[0] = current_state_(0);
        forces_params.xinit[1] = current_state_(1);
        forces_params.xinit[2] = current_state_(2);
        forces_params.xinit[3] = current_state_(3);
        forces_params.xinit[4] = 0.0;
        forces_params.xinit[5] = 0.0;

        //ROS_WARN_STREAM("ss.size():" << ss.size() << " traj_i: " << traj_i);
        if (forces_params.xinit[4] > ss[traj_i + 1]) {

            if (traj_i + 1 == ss.size()) {
                goal_reached_ = true;
                ROS_ERROR_STREAM("GOAL REACHED: "<< ss.size());
            } else {
                traj_i++;
                //ROS_ERROR_STREAM("SWITCH SPLINE " << acadoVariables.x[4]);
            }
        }

        if(plan_) {
            double smin = forces_params.x0[FORCES_TOTAL_V + 4];

            traj_i = spline_closest_point(traj_i, ss, smin, window_size_,
                                        n_search_points_);
            forces_params.xinit[4] = smin;
            //ROS_ERROR_STREAM("smin: " << smin);
            //ROS_ERROR_STREAM("smin: " << ss[traj_i]);
            //ROS_ERROR_STREAM("smin: " << ss[traj_i+1]);
        }

        for (N_iter = 0; N_iter < FORCES_N; N_iter++) {
            int k = N_iter*FORCES_NPAR;

            forces_params.all_parameters[k + 0] = ref_path_x.m_a[traj_i];        // spline coefficients
            forces_params.all_parameters[k + 1] = ref_path_x.m_b[traj_i];
            forces_params.all_parameters[k + 2] = ref_path_x.m_c[traj_i];        // spline coefficients
            forces_params.all_parameters[k + 3] = ref_path_x.m_d[traj_i];
            forces_params.all_parameters[k + 4] = ref_path_y.m_a[traj_i];        // spline coefficients
            forces_params.all_parameters[k + 5] = ref_path_y.m_b[traj_i];
            forces_params.all_parameters[k + 6] = ref_path_y.m_c[traj_i];        // spline coefficients
            forces_params.all_parameters[k + 7] = ref_path_y.m_d[traj_i];

            forces_params.all_parameters[k + 16] = ss[traj_i];       // s1
            forces_params.all_parameters[k + 17] = ss[traj_i + 1];       //s2
            forces_params.all_parameters[k + 18] = ss[traj_i + 1] + 0.02;       // d
            obstacle_distance1 = sqrt(pow((current_state_(0)-obstacles_.lmpcc_obstacles[0].trajectory.poses[N_iter].pose.position.x),2)+pow((current_state_(1)-obstacles_.lmpcc_obstacles[0].trajectory.poses[N_iter].pose.position.y),2));
            obstacle_distance2 = sqrt(pow((current_state_(0)-obstacles_.lmpcc_obstacles[1].trajectory.poses[N_iter].pose.position.x),2)+pow((current_state_(1)-obstacles_.lmpcc_obstacles[0].trajectory.poses[N_iter].pose.position.y),2));

            forces_params.all_parameters[k + 19] = cost_contour_weight_factors_(0);
            forces_params.all_parameters[k + 20] = cost_contour_weight_factors_(1);     // weight lag error
            forces_params.all_parameters[k + 21] = cost_control_weight_factors_(0);    // weight acceleration
            forces_params.all_parameters[k + 22] = cost_control_weight_factors_(1);   // weight delta
            forces_params.all_parameters[k + 24] = slack_weight_;                     //slack weight
            forces_params.all_parameters[k + 25] = repulsive_weight_;                     //repulsive weight
            forces_params.all_parameters[k + 38] = velocity_weight_;                     //repulsive weight

            forces_params.all_parameters[k + 26] = r_discs_; //radius of the disks
            forces_params.all_parameters[k + 27] = x_discs_[0];                        // position of the car discs
            forces_params.all_parameters[k + 39] = x_discs_[1];                        // position of the car discs
            forces_params.all_parameters[k + 40] = x_discs_[2];                        // position of the car discs
            //forces_params.all_parameters[k + 42] = x_discs_[3];                        // position of the car discs
            //forces_params.all_parameters[k + 43] = x_discs_[4];

            //forces_params.all_parameters[k + 28] = obstacles_.lmpcc_obstacles[0].trajectory.poses[N_iter].pose.position.x;      // x position of obstacle 1
            //forces_params.all_parameters[k + 19] = obstacles_.lmpcc_obstacles[0].trajectory.poses[N_iter].pose.position.y;      // y position of obstacle 1
            //ToDo check convertion from quaternion to RPY angle
            /*forces_params.all_parameters[k + 30] = obstacles_.lmpcc_obstacles[0].trajectory.poses[N_iter].pose.orientation.z;   // heading of obstacle 1
            forces_params.all_parameters[k + 31] = obstacles_.lmpcc_obstacles[0].major_semiaxis[N_iter];       // major semiaxis of obstacle 1
            forces_params.all_parameters[k + 32] = obstacles_.lmpcc_obstacles[0].minor_semiaxis[N_iter];       // minor semiaxis of obstacle 1


            forces_params.all_parameters[k + 33] = obstacles_.lmpcc_obstacles[1].trajectory.poses[N_iter].pose.position.x;      // x position of obstacle 2
            forces_params.all_parameters[k + 34] = obstacles_.lmpcc_obstacles[1].trajectory.poses[N_iter].pose.position.y;      // y position of obstacle 2
            forces_params.all_parameters[k + 35] = obstacles_.lmpcc_obstacles[1].trajectory.poses[N_iter].pose.orientation.z;   // heading of obstacle 2
            forces_params.all_parameters[k + 36] = obstacles_.lmpcc_obstacles[1].major_semiaxis[N_iter];       // major semiaxis of obstacle 2
            forces_params.all_parameters[k + 37] = obstacles_.lmpcc_obstacles[1].minor_semiaxis[N_iter];       // minor semiaxis of obstacle 2
            forces_params.all_parameters[k + 41] = right_offset_;
            forces_params.all_parameters[k + 42] = left_offset_;*/

            if (goal_reached_) {
                reduced_reference_velocity_ = current_state_(3)-4*0.25*(N_iter+1);
                if(reduced_reference_velocity_ < 0)
                    reduced_reference_velocity_=0;

                forces_params.all_parameters[k + 23] = reduced_reference_velocity_;
            } else {

                reduced_reference_velocity_ = current_state_(3) + 1.5 * 0.25 * (N_iter+1);
                if (reduced_reference_velocity_ > reference_velocity_)
                    reduced_reference_velocity_ = reference_velocity_;
                forces_params.all_parameters[k + 23] = reduced_reference_velocity_;

                forces_params.all_parameters[k + 8] = ref_path_x.m_a[traj_i + 1];        // spline coefficients
                forces_params.all_parameters[k + 9] = ref_path_x.m_b[traj_i + 1];
                forces_params.all_parameters[k + 10] = ref_path_x.m_c[traj_i + 1];        // spline coefficients
                forces_params.all_parameters[k + 11] = ref_path_x.m_d[traj_i + 1];
                forces_params.all_parameters[k + 12] = ref_path_y.m_a[traj_i + 1];        // spline coefficients
                forces_params.all_parameters[k + 13] = ref_path_y.m_b[traj_i + 1];
                forces_params.all_parameters[k + 14] = ref_path_y.m_c[traj_i + 1];        // spline coefficients
                forces_params.all_parameters[k + 15] = ref_path_y.m_d[traj_i + 1];
            }

            //forces_params.all_parameters[k + 23] = ss[traj_i + 1] + 0.02;
        }

        /*ROS_INFO_STREAM("x0[0] " << forces_params.x0[0]);
        ROS_INFO_STREAM("x0[1] " << forces_params.x0[1]);
        ROS_INFO_STREAM("x0[2] " << forces_params.x0[2]);
        ROS_INFO_STREAM("x0[3] " << forces_params.x0[3]);
        ROS_INFO_STREAM("x0[4] " << forces_params.x0[4]);
        ROS_INFO_STREAM("x0[5] " << forces_params.x0[5]);
        ROS_INFO_STREAM("x0[6] " << forces_params.x0[6]);
        ROS_INFO_STREAM("x0[7] " << forces_params.x0[7]);
        ROS_INFO_STREAM("x0[8] " << forces_params.x0[8]);
        ROS_INFO_STREAM("xinit[0] " << forces_params.xinit[0]);
        ROS_INFO_STREAM("xinit[1] " << forces_params.xinit[1]);
        ROS_INFO_STREAM("xinit[2] " << forces_params.xinit[2]);
        ROS_INFO_STREAM("xinit[3] " << forces_params.xinit[3]);
        ROS_INFO_STREAM("xinit[4] " << forces_params.xinit[4]);
        ROS_INFO_STREAM("xinit[5] " << forces_params.xinit[5]);*/


        exit_code = FORCESNLPsolver_solve(&forces_params, &forces_output, &forces_info, stdout, extfunc_eval);
        ROS_INFO_STREAM("exit_code before iter " << exit_code);
        ROS_INFO_STREAM("primal objective " << forces_info.pobj);
        ROS_INFO_STREAM("number of iterations for optimality " << forces_info.it2opt);

        /*int j = 0;
        while (exit_code !=1 && j < (n_iterations_-1)) {

            exit_code = FORCESNLPsolver_solve(&forces_params, &forces_output, &forces_info, NULL, extfunc_eval);
            ROS_INFO_STREAM("exit_code" << exit_code);

            //printf("\tReal-Time Iteration:  KKT Tolerance = %.3e\n\n", acado_getKKT());
            //ROS_INFO_STREAM(acado_getObjective());
            j++;    //acado_printDifferentialVariables();

        }*/

        controlled_velocity_.throttle = forces_output.x01[0]/10.0;// / 1.5; // maximum acceleration 1.5m/s
        ROS_INFO_STREAM("accel_output " << controlled_velocity_.throttle);
        if (controlled_velocity_.throttle < 0.0)
            controlled_velocity_.brake = abs(controlled_velocity_.throttle);
        else
            controlled_velocity_.brake = 0.0;
        //controlled_velocity_.steer = -acadoVariables.u[1]*2.0 ;// / 0.52; // maximum steer
        controlled_velocity_.steer = forces_output.x01[1] * 2.0;
        ROS_INFO_STREAM("steer_output " << controlled_velocity_.steer);

        if(debug_){
            //publishPredictedTrajectory();
            //publishPredictedOutput();
            //broadcastPathPose();
            //publishFeedback(j,te_);
        }
   
        publishPredictedCollisionSpace();
        //broadcastPathPose();
        brake_.data = controlled_velocity_.brake;
        //cost_.data = acado_getObjective();
    }

    if(exit_code!=1) {
        publishZeroJointVelocity();
    }
    else {
        controlled_velocity_pub_.publish(controlled_velocity_);
    }
    /*if (acado_getKKT() != acado_getKKT())
    {
        //reset_solver();
        ROS_INFO_STREAM("is nan: resetting");
    }*/

    for(int i = 0; i < FORCES_TOTAL_V; i++){
        forces_params.x0[i] = forces_output.x01[i]; 
        forces_params.x0[i+FORCES_TOTAL_V] = forces_output.x02[i];
        forces_params.x0[i+2*FORCES_TOTAL_V] = forces_output.x03[i];
        forces_params.x0[i+3*FORCES_TOTAL_V] = forces_output.x04[i];
        forces_params.x0[i+4*FORCES_TOTAL_V] = forces_output.x05[i];
        forces_params.x0[i+5*FORCES_TOTAL_V] = forces_output.x06[i];
        forces_params.x0[i+6*FORCES_TOTAL_V] = forces_output.x07[i];
        forces_params.x0[i+7*FORCES_TOTAL_V] = forces_output.x08[i];
        forces_params.x0[i+8*FORCES_TOTAL_V] = forces_output.x09[i];
        forces_params.x0[i+9*FORCES_TOTAL_V] = forces_output.x10[i];
        forces_params.x0[i+10*FORCES_TOTAL_V] = forces_output.x11[i];
        forces_params.x0[i+11*FORCES_TOTAL_V] = forces_output.x12[i];
        forces_params.x0[i+12*FORCES_TOTAL_V] = forces_output.x13[i];
        forces_params.x0[i+13*FORCES_TOTAL_V] = forces_output.x14[i];
        forces_params.x0[i+14*FORCES_TOTAL_V] = forces_output.x15[i];
        forces_params.x0[i+15*FORCES_TOTAL_V] = forces_output.x16[i];
        forces_params.x0[i+16*FORCES_TOTAL_V] = forces_output.x17[i];
        forces_params.x0[i+17*FORCES_TOTAL_V] = forces_output.x18[i];
        forces_params.x0[i+18*FORCES_TOTAL_V] = forces_output.x19[i];
        forces_params.x0[i+19*FORCES_TOTAL_V] = forces_output.x20[i];
    }
}

int MPCC::spline_closest_point(int cur_traj_i, std::vector<double> ss_vec, double &s_guess, double window, int n_tries){
    double s_min = ss_vec[cur_traj_i] - MAX_STEP_BACK_TOLERANCE;
    double s_max = ss_vec[cur_traj_i+1] + MAX_STEP_BACK_TOLERANCE;
    double lower = std::max(s_min, s_guess-window);
    double upper = std::min(s_max, s_guess + window);
    double s_i=upper,spline_pos_x_i,spline_pos_y_i;
    double dist_i,min_dist;

    //First, try the furthest point in our search window. This is the reference that must be beat.
    spline_pos_x_i = ref_path_x(s_i);
    spline_pos_y_i = ref_path_y(s_i);
    double s_best = s_i;
    min_dist = std::sqrt((spline_pos_x_i-current_state_(0))*(spline_pos_x_i-current_state_(0))+(spline_pos_y_i-current_state_(1))*(spline_pos_y_i-current_state_(1)));

    //Compute the step size.
    //Divide by minus one. If you want to go from 1 to 3 (distance two) with three steps, step size must be (3-1)/2=1 to go 1,2,3.
    double step_size = (upper-lower)/(n_tries-1);
    for(s_i=lower; s_i < upper; s_i+=step_size) {
        spline_pos_x_i = ref_path_x(s_i);
        spline_pos_y_i = ref_path_y(s_i);
        dist_i = std::sqrt((spline_pos_x_i-current_state_(0))*(spline_pos_x_i-current_state_(0))+(spline_pos_y_i-current_state_(1))*(spline_pos_y_i-current_state_(1)));

        if(dist_i<min_dist){
            min_dist = dist_i;
            s_best = s_i;
        }
    }
    double previous_guess = s_guess;
    s_guess = s_best;

    int next_traj = cur_traj_i;
    if(s_best == lower && lower != previous_guess){
        //If we hit the low point of the window, and that low point was the end of this spline segment, try one segment higher!
        if(lower == s_min && cur_traj_i > 0) {
            next_traj--;
        }
        return spline_closest_point(next_traj, ss_vec, s_guess, window, n_tries);
    }

    if(s_best == upper && upper != previous_guess){
        //If we hit the high point of the window, and that high point was the end of this spline segment, try one segment higher!
        if(upper == s_max && cur_traj_i < ss_vec.size()-2) {
            next_traj++;
        }
        return spline_closest_point(next_traj, ss_vec, s_guess, window, n_tries);
    }
    return cur_traj_i;
}


void MPCC::Ref_path(std::vector<double> x,std::vector<double> y, std::vector<double> theta) {

    double k, dk, L;
    n_clothoid = controller_config_->n_points_clothoid_;
    n_pts = controller_config_->n_points_spline_;
    std::vector<double> X(n_clothoid), Y(n_clothoid);
    std::vector<double> X_all, Y_all, S_all;
    total_length_= 0;

    S_all.push_back(0);
    ROS_INFO("Generating path...");
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

    dist_spline_pts_ = total_length_ / (n_pts );
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
        path_length_ = ss[i];
    }

    ref_path_x.set_points(ss,xx);
    ref_path_y.set_points(ss,yy);

    ROS_INFO("Path generated");
}

void MPCC::ConstructRefPath(){
    geometry_msgs::Pose pose;
    double ysqr, t3, t4;
    tf2::Quaternion myQuaternion;
    for (int ref_point_it = 0; ref_point_it < controller_config_->ref_x_.size(); ref_point_it++)
    {
        pose.position.x = controller_config_->ref_x_.at(ref_point_it);
        pose.position.y = controller_config_->ref_y_.at(ref_point_it);
        // Convert RPY from path to quaternion
        myQuaternion.setRPY( 0, 0, controller_config_->ref_theta_.at(ref_point_it) );
        pose.orientation.x = myQuaternion.x();
        pose.orientation.y = myQuaternion.y();
        pose.orientation.z = myQuaternion.z();
        pose.orientation.w = myQuaternion.w();
        // Convert from global_frame to planning frame
        transformPose(controller_config_->global_path_frame_,controller_config_->target_frame_,pose);
        X_road[ref_point_it] = pose.position.x;
        Y_road[ref_point_it] = pose.position.y;
        // Convert from quaternion to RPY
        ysqr = pose.orientation.y * pose.orientation.y;
        t3 = +2.0 * (pose.orientation.w * pose.orientation.z
                             + pose.orientation.x *pose.orientation.y);
        t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z);
        Theta_road[ref_point_it] = std::atan2(t3, t4);
    }

    Ref_path(X_road, Y_road, Theta_road);
}

void MPCC::getWayPointsCallBack(nav_msgs::Path waypoints){
	//
	last_waypoints_size_ = waypoints_size_;
	if (waypoints.poses.size()==0){
		

		ROS_WARN("Waypoint message is empty");
		X_road[0] = 0.0;
		Y_road[0] = 0.0;
		Theta_road[0] = 0.0; //to do conversion quaternion
			
		X_road[1] = 300.0;
		Y_road[1] = 0.0;
		Theta_road[1] = 0.0; //to do conversion quaternion
			
        waypoints_size_ = 2.0;
           
	}
    else
	{
	    ROS_INFO("Getting waypoints from SOMEWHERE...");
        waypoints_size_ = waypoints.poses.size();
 		/*for (int ref_point_it = 0; ref_point_it<waypoints_size_; ref_point_it++)
        {
		    X_road[ref_point_it] = waypoints.poses.at(ref_point_it).pose.position.x;
		    Y_road[ref_point_it] = waypoints.poses.at(ref_point_it).pose.position.y;
		    Theta_road[ref_point_it] = quaternionToangle(waypoints.poses.at(ref_point_it).pose.orientation); //to do conversion quaternion
        }*/
	}
	  
    //ConstructRefPath();
    Ref_path(X_road, Y_road, Theta_road);
    //ROS_INFO("ConstructRefPath");
    publishSplineTrajectory();
    plotRoad();
}

double MPCC::quaternionToangle(geometry_msgs::Quaternion q){

  double ysqr, t3, t4;
  
  // Convert from quaternion to RPY
  ysqr = q.y * q.y;
  t3 = +2.0 * (q.w *q.z + q.x *q.y);
  t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
  return std::atan2(t3, t4);
}

void MPCC::reconfigureCallback(lmpcc::PredictiveControllerConfig& config, uint32_t level){

    ROS_INFO("reconfigure callback!");
    cost_contour_weight_factors_(0) = config.Wcontour;
    cost_contour_weight_factors_(1) = config.Wlag;
    cost_control_weight_factors_(0) = config.Ka;
    cost_control_weight_factors_(1) = config.Kdelta;
    velocity_weight_ = config.Kv;
    ini_vel_x_ = config.ini_v0;
    bb_hack_ = config.bb_hack;
    slack_weight_= config.Ws;
    repulsive_weight_ = config.WR;

    reference_velocity_ = config.vRef;
    slack_weight_= config.Ws;
    repulsive_weight_ = config.WR;

    n_iterations_ = config.n_iterations;
    simulation_mode_ = config.simulation_mode;
    // reset world
    reset_world_ = config.reset_world;
    if(reset_world_) {
        reset_simulation_client_.call(reset_msg_);
        reset_ekf_client_.call(reset_pose_msg_);
        reset_solver();
        traj_i = 0;
        goal_reached_ = false;
    }

    //Search window parameters
    window_size_ = config.window_size;
    n_search_points_ = config.n_search_points;

    if(right_offset_!=config.right_offset) {
        right_offset_ = config.right_offset;
        replan_ = true;
    }
    if(left_offset_!=config.left_offset) {
        left_offset_ = config.left_offset;
        replan_ = true;
    }

    if(replan_ && waypoints_size_>0){

        Ref_path(X_road, Y_road, Theta_road);
        //ROS_INFO("ConstructRefPath");
        publishSplineTrajectory();
        plotRoad();
        replan_ = false;
    }

    debug_ = config.debug;
    if (waypoints_size_ >1) {
        ROS_WARN("Planning...");
        plan_ = config.plan;
        enable_output_ = config.enable_output;
    } else {
        ROS_WARN("No waypoints were provided...");
        config.plan = false;
        config.enable_output = false;
        plan_ = false;        
        enable_output_ = false;
    }

    if(plan_){
        ROS_INFO("Before reset_solver");
		reset_solver();
        ROS_INFO("After reset_solver");
		//acado_initializeSolver( );
		//ROS_INFO("acado_initializeSolver");
		//ConstructRefPath();
		//getWayPointsCallBack( );
		//ROS_INFO("ConstructRefPath");
        plotRoad();
        publishSplineTrajectory();
        //ROS_INFO("reconfigure callback!");
        traj_i = 0;
        goal_reached_ = false;
        timer_.start();
	}
    else{
        timer_.stop();
    }
}

// read current position and velocity of robot joints
void MPCC::StateCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
   double ysqr, t3, t4;
   if (activate_debug_output_)
   {
//  ROS_INFO("MPCC::StateCallBack");
   }
   //ROS_INFO("MPCC::StateCallBack");
   controller_config_->target_frame_ = msg->header.frame_id;
   last_state_ = current_state_;

   ysqr = msg->pose.pose.orientation.y * msg->pose.pose.orientation.y;
   t3 = +2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z
                             + msg->pose.pose.orientation.x *msg->pose.pose.orientation.y);
   t4 = +1.0 - 2.0 * (ysqr + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

   current_state_(2) = std::atan2(t3, t4);

   current_state_(0) =    msg->pose.pose.position.x ; // for shifting the current coordinates to the center of mass
   current_state_(1) =    msg->pose.pose.position.y ;

   current_state_(3) = std::sqrt(std::pow(msg->twist.twist.linear.x,2)+std::pow(msg->twist.twist.linear.y,2));
}
void MPCC::ObstacleStateCallback(const cv_msgs::PredictedMoGTracks& objects)
{
    /*
    double ysqr, t3, t4;
    //ROS_INFO_STREAM("N tracks: "<<objects.tracks.size());
    if(objects.tracks.size()) {
        //ROS_INFO_STREAM("N track: " << objects.tracks[0].track.size());
        //ROS_INFO_STREAM("N: " << objects.tracks[0].track[0].pose.size());
    }
    //reset all objects
    for (int obst_it = 0; obst_it < controller_config_->n_obstacles_; obst_it++)
    {
        for(int t = 0;t<obstacles_.lmpcc_obstacles[obst_it].trajectory.poses.size();t++){
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.x = 10000;
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.position.y = 10000;
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[t].pose.orientation.z = 0;
            obstacles_.lmpcc_obstacles[obst_it].major_semiaxis[t] = 0.001;
            obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis[t] = 0.001;
        }
    }
    for(int i=0;i<objects.tracks.size();i++){
        cv_msgs::PredictedMoGTrack track = objects.tracks[i];
        for(int j=0;j<track.track.size();j++){
            cv_msgs::PredictedMoG mog = track.track[j];
            int mogs_to_consider = 1; //mog.pose.size(); //FIXME only use the first item in the MOG, to allow for multi-object tracking.
            for (int k = 0; k < mogs_to_consider; k++) {
                int current_obstacle = i*mogs_to_consider + k;
                if (j == 0) {
                  std::cout << "current obstacle: " << current_obstacle << std::endl;
                  std::cout << "current i: " << i << std::endl;
                }
                obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose=mog.pose[k].pose;
                //ToDo
                transformPose(objects.header.frame_id,controller_config_->target_frame_,obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose);

                // Convert quaternion to RPY
                ysqr = obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.y * obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.y;
                t3 = +2.0 * (obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.w * obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.z
                             + obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.x * obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.y);
                t4 = +1.0 - 2.0 * (ysqr + obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.z * obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.z);

                obstacles_.lmpcc_obstacles[current_obstacle].trajectory.poses[j].pose.orientation.z = std::atan2(t3, t4);

                obstacles_.lmpcc_obstacles[current_obstacle].major_semiaxis[j] = 0.5;
                obstacles_.lmpcc_obstacles[current_obstacle].minor_semiaxis[j] = 0.5;

            }
        }
    }
  for(int i=0;i<objects.tracks.size();i++){
    cv_msgs::PredictedMoGTrack track = objects.tracks[i];
    for(int j=track.track.size();j<FORCES_N;j++){
      cv_msgs::PredictedMoG mog = track.track[track.track.size()-1];
      for (int k = 0; k < mog.pose.size(); k++) {
        obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose=mog.pose[k].pose;
        //ToDo
        transformPose(objects.header.frame_id,controller_config_->target_frame_,obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose);

        // Convert quaternion to RPY
        ysqr = obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.y * obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.y;
        t3 = +2.0 * (obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.w * obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.z
                     + obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.x * obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.y);
        t4 = +1.0 - 2.0 * (ysqr + obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.z * obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.z);

        obstacles_.lmpcc_obstacles[k].trajectory.poses[j].pose.orientation.z = std::atan2(t3, t4);

        obstacles_.lmpcc_obstacles[k].major_semiaxis[j] = 0.5;
        obstacles_.lmpcc_obstacles[k].minor_semiaxis[j] = 0.5;

      }
    }
  }*/
}
void MPCC::ObstacleCallBack(const lmpcc_msgs::lmpcc_obstacle_array& received_obstacles)
{
    //ROS_INFO("LMPCC::ObstacleCallBack");
    lmpcc_msgs::lmpcc_obstacle_array total_obstacles;
    total_obstacles.lmpcc_obstacles.resize(controller_config_->n_obstacles_);

    total_obstacles.lmpcc_obstacles = received_obstacles.lmpcc_obstacles;

    //ROS_INFO_STREAM("-- Received # obstacles: " << received_obstacles.lmpcc_obstacles.size());
    //ROS_INFO_STREAM("-- Expected # obstacles: " << controller_config_->n_obstacles_);

    if (received_obstacles.lmpcc_obstacles.size() < controller_config_->n_obstacles_)
    {
        for (int obst_it = received_obstacles.lmpcc_obstacles.size(); obst_it < controller_config_->n_obstacles_; obst_it++)
        {
            total_obstacles.lmpcc_obstacles[obst_it].pose.position.x = current_state_(0) - 100;
            total_obstacles.lmpcc_obstacles[obst_it].pose.position.y = 0;
            total_obstacles.lmpcc_obstacles[obst_it].pose.orientation.z = 0;
            total_obstacles.lmpcc_obstacles[obst_it].major_semiaxis.resize(FORCES_N);
            total_obstacles.lmpcc_obstacles[obst_it].minor_semiaxis.resize(FORCES_N);
            total_obstacles.lmpcc_obstacles[obst_it].trajectory.poses.resize(FORCES_N);
            for (int traj_it = 0; traj_it < FORCES_N; traj_it++)
            {
                total_obstacles.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.x = current_state_(0) - 100;
                total_obstacles.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.y = 0;
                total_obstacles.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.orientation.z = 0;
                total_obstacles.lmpcc_obstacles[obst_it].major_semiaxis[traj_it] = 0.001;
                total_obstacles.lmpcc_obstacles[obst_it].minor_semiaxis[traj_it] = 0.001;
            }
        }
    }

    //obstacles_.lmpcc_obstacles.resize(controller_config_->n_obstacles_);

    for (int total_obst_it = 0; total_obst_it < controller_config_->n_obstacles_; total_obst_it++)
    {
        obstacles_.lmpcc_obstacles[total_obst_it] = total_obstacles.lmpcc_obstacles[total_obst_it];
    }
}

void MPCC::publishZeroJointVelocity()
{
    if (activate_debug_output_)
    {
       ROS_INFO("Publishing ZERO joint velocity!!");
    }
    carla_msgs::CarlaEgoVehicleControl pub_msg;
    if(!simulation_mode_)
        broadcastTF();
    controlled_velocity_ = pub_msg;
    controlled_velocity_.throttle = 0;
    controlled_velocity_.brake = 1;
    controlled_velocity_.steer = 0.0;
    controlled_velocity_pub_.publish(controlled_velocity_);
}

void MPCC::plotRoad(void)
{
    visualization_msgs::Marker line_strip;
    visualization_msgs::MarkerArray line_list;
    line_strip.header.frame_id = controller_config_->target_frame_;
    line_strip.id = 1;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.5;
    line_strip.scale.y = 0.5;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Pose pose;

    pose.position.x = (controller_config_->road_width_left_+left_offset_+line_strip.scale.x/2.0)*-sin(current_state_(2));
    pose.position.y = (controller_config_->road_width_left_+left_offset_+line_strip.scale.x/2.0)*cos(current_state_(2));

    geometry_msgs::Point p;
    p.x = spline_traj_.poses[0].pose.position.x + pose.position.x;
    p.y = spline_traj_.poses[0].pose.position.y + pose.position.y;
    p.z = 0.2;  //z a little bit above ground to draw it above the pointcloud.

    line_strip.points.push_back(p);

    p.x = spline_traj_.poses[spline_traj_.poses.size()-1].pose.position.x+ pose.position.x;
    p.y = spline_traj_.poses[spline_traj_.poses.size()-1].pose.position.y+ pose.position.y;
    p.z = 0.2;  //z a little bit above ground to draw it above the pointcloud.

    line_strip.points.push_back(p);

    line_list.markers.push_back(line_strip);

    line_strip.points.pop_back();
    line_strip.points.pop_back();

    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.id = 2;
    pose.position.x = (-controller_config_->road_width_right_-right_offset_-line_strip.scale.x/2.0)*-sin(current_state_(2));
    pose.position.y = (-controller_config_->road_width_right_-right_offset_-line_strip.scale.x/2.0)*cos(current_state_(2));

    p.x = spline_traj_.poses[0].pose.position.x+pose.position.x;
    p.y = spline_traj_.poses[0].pose.position.y+pose.position.y;
    p.z = 0.2;  //z a little bit above ground to draw it above the pointcloud.

    line_strip.points.push_back(p);

    p.x = spline_traj_.poses[spline_traj_.poses.size()-1].pose.position.x+pose.position.x;
    p.y = spline_traj_.poses[spline_traj_.poses.size()-1].pose.position.y+pose.position.y;
    p.z = 0.2;  //z a little bit above ground to draw it above the pointcloud.

    line_strip.points.push_back(p);

    line_list.markers.push_back(line_strip);

    marker_pub_.publish(line_list);

}

void MPCC::publishSplineTrajectory(void)
{
    spline_traj_.header.stamp = ros::Time::now();
    spline_traj_.header.frame_id = controller_config_->target_frame_;
    for (int i = 0; i < spline_traj_.poses.size(); i++) // 100 points
    {
        spline_traj_.poses[i].pose.position.x = ref_path_x(i*(n_pts)*dist_spline_pts_/spline_traj_.poses.size()); //x
        spline_traj_.poses[i].pose.position.y = ref_path_y(i*(n_pts)*dist_spline_pts_/spline_traj_.poses.size()); //y
        spline_traj_.poses[i].pose.position.z = 0.2; //z a little bit above ground to draw it above the pointcloud.
    }

    spline_traj_pub_.publish(spline_traj_);
}

void MPCC::publishPredictedTrajectory(void)
{
    for (int i = 0; i < FORCES_N; i++)
    {
        pred_traj_.poses[i].pose.position.x = forces_params.x0[i * FORCES_TOTAL_V + 3]; //x
        pred_traj_.poses[i].pose.position.y = forces_params.x0[i * FORCES_TOTAL_V + 4]; //y
        pred_traj_.poses[i].pose.orientation.z = forces_params.x0[i * FORCES_TOTAL_V + 5]; //theta
    }

    pred_traj_pub_.publish(pred_traj_);
}

void MPCC::publishPredictedOutput(void)
{
    for (int i = 0; i < FORCES_N; i++)
    {
        pred_cmd_.poses[i].pose.position.x = forces_params.x0[i*FORCES_TOTAL_V + 0]; //acc
       
		
        pred_cmd_.poses[i].pose.position.y = forces_params.x0[i*FORCES_TOTAL_V + 1]; //delta
        pred_cmd_.poses[i].pose.position.z = forces_params.x0[i*FORCES_TOTAL_V+ 2];  //slack
    }

    pred_cmd_pub_.publish(pred_cmd_);
}
/*
cv::Mat make_colormap(int length){
    cv::Mat img_in(length,1, CV_8UC1);
    for(int k = 0; k < length; k++){
        img_in.row(k).setTo((k*256)/(length-1));
    }
    cv::Mat img_color;
    cv::applyColorMap(img_in, img_color, cv::COLORMAP_PARULA);
    return img_color;
}
*/
void MPCC::publishPredictedCollisionSpace(void)
{
    visualization_msgs::MarkerArray collision_space;
    //cv::Mat colormap = make_colormap(FORCES_N);
    for (int k = 0; k< controller_config_->n_discs_; k++){

        for (int i = 0; i < FORCES_N; i++)
        {
            ellips1.id = 60+i+k*FORCES_N;

            //-1.577
            ellips1.pose.position.x = forces_params.x0[i * FORCES_TOTAL_V + 3]+(x_discs_[k])*cos(forces_params.x0[i * FORCES_TOTAL_V + 5]);
            ellips1.pose.position.y = forces_params.x0[i * FORCES_TOTAL_V + 4]+(x_discs_[k])*sin(forces_params.x0[i * FORCES_TOTAL_V + 5]);
            ellips1.pose.position.z = 0.2;  //z a little bit above ground to draw it above the pointcloud.
            ellips1.pose.orientation.x = 0;
            ellips1.pose.orientation.y = 0;
            ellips1.pose.orientation.z = 0;
            ellips1.pose.orientation.w = 1;
            ellips1.color.b = 1.0;//colormap.at<unsigned char>(i,0)/255.0;
            ellips1.color.g = 0.0;//colormap.at<unsigned char>(i,1)/255.0;
            ellips1.color.r = 0.0;//colormap.at<unsigned char>(i,2)/255.0;
            collision_space.markers.push_back(ellips1);
        }
    }
    robot_collision_space_pub_.publish(collision_space);
}

/*void MPCC::publishCost(void){

    cost_pub_.publish(cost_);
    brake_pub_.publish(brake_);
}*/

/*void MPCC::publishFeedback(int& it, double& time)
{

    lmpcc::control_feedback feedback_msg;

    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.header.frame_id = controller_config_->target_frame_;

    feedback_msg.cost = cost_.data;
    feedback_msg.iterations = it;
    feedback_msg.computation_time = time;
    feedback_msg.kkt = acado_getKKT();

    feedback_msg.wC = cost_contour_weight_factors_(0);       // weight factor on contour error
    feedback_msg.wL = cost_contour_weight_factors_(1);       // weight factor on lag error
    feedback_msg.wV = cost_control_weight_factors_(0);       // weight factor on theta
    feedback_msg.wW = cost_control_weight_factors_(1);

    // Compute contour errors
    feedback_msg.contour_errors.data.resize(2);

    feedback_msg.contour_errors.data[0] = contour_error_;
    feedback_msg.contour_errors.data[1] = lag_error_;

    feedback_msg.reference_path = spline_traj2_;
    feedback_msg.prediction_horizon = pred_traj_;
    feedback_msg.prediction_horizon.poses[0].pose.position.z = acadoVariables.x[3];

    //Search window parameters
    feedback_msg.window = window_size_;
    feedback_msg.search_points = n_search_points_;

    // obstacles
    feedback_msg.obstx_0 = obstacles_.lmpcc_obstacles[0].trajectory.poses[0].pose.position.x;
    feedback_msg.obsty_0 = obstacles_.lmpcc_obstacles[0].trajectory.poses[0].pose.position.y;
    feedback_msg.obstx_1 = obstacles_.lmpcc_obstacles[1].trajectory.poses[0].pose.position.x;
    feedback_msg.obsty_1 = obstacles_.lmpcc_obstacles[1].trajectory.poses[0].pose.position.y;
    feedback_msg.obstacle_distance1 = sqrt(pow((current_state_(0)-feedback_msg.obstx_0),2)+pow((current_state_(1)-feedback_msg.obsty_0),2));
    feedback_msg.obstacle_distance2 = sqrt(pow((current_state_(0)-feedback_msg.obstx_1),2)+pow((current_state_(1)-feedback_msg.obsty_1),2));

    // control input
    feedback_msg.computed_control.linear.x = controlled_velocity_.steer;
    feedback_msg.computed_control.linear.y = controlled_velocity_.throttle;
    feedback_msg.computed_control.linear.z = controlled_velocity_.brake;

    // state information
    feedback_msg.computed_control.angular.x =  current_state_(0);
    feedback_msg.computed_control.angular.y =  current_state_(1);
    feedback_msg.computed_control.angular.z =  current_state_(2);

    feedback_pub_.publish(feedback_msg);
}*/

// Utils

bool MPCC::transformPose(const std::string& from, const std::string& to, geometry_msgs::Pose& pose)
{
    bool transform = false;
    tf::StampedTransform stamped_tf;
    //ROS_INFO_STREAM("Transforming from :" << from << " to: " << to);
    geometry_msgs::PoseStamped stampedPose_in, stampedPose_out;

    stampedPose_in.pose = pose;
    if(std::sqrt(std::pow(pose.orientation.x,2)+std::pow(pose.orientation.y,2)+std::pow(pose.orientation.z,2)+std::pow(pose.orientation.w,2))<1){
        stampedPose_in.pose.orientation.x = 0;
        stampedPose_in.pose.orientation.y = 0;
        stampedPose_in.pose.orientation.z = 0;
        stampedPose_in.pose.orientation.w = 1;
    }
//    stampedPose_in.header.stamp = ros::Time::now();
    stampedPose_in.header.frame_id = from;

    // make sure source and target frame exist
    if (tf_listener_.frameExists(to) && tf_listener_.frameExists(from))
    {
        try
        {
            // find transforamtion between souce and target frame
            tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.02));
            tf_listener_.transformPose(to, stampedPose_in, stampedPose_out);

            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("MPCC::getTransform: %s", ex.what());
        }
    }

    else
    {
        ROS_WARN("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",from.c_str(), to.c_str());
    }
    pose = stampedPose_out.pose;
    stampedPose_in.pose = stampedPose_out.pose;
    stampedPose_in.header.frame_id = to;

    return transform;
}
