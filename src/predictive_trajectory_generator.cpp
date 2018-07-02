
//This file containts cost function intsert in to generated trajectory.

#include <predictive_control/predictive_trajectory_generator.h>

pd_frame_tracker::~pd_frame_tracker()
{
  clearDataMember();
}

// diallocated memory
void pd_frame_tracker::clearDataMember()
{
  // resize matrix and vectors
  state_initialize_.resize(state_dim_);
  state_initialize_.setAll(0.0);
  control_initialize_.resize(control_dim_);
  control_initialize_.setAll(0.0);

  control_min_constraint_.resize(control_dim_);
  control_min_constraint_.setAll(0.0);
  control_max_constraint_.resize(control_dim_);
  control_max_constraint_.setAll(0.0);

  lsq_state_weight_factors_.resize(state_dim_);
  lsq_control_weight_factors_.resize(control_dim_);
}

// initialize data member of pd_frame_tracker class
bool pd_frame_tracker::initialize()
{
  /*boost::shared_ptr<RealTimeAlgorithm> ocp;
  ocp.reset(new RealTimeAlgorithm());
  setAlgorithmOptions<RealTimeAlgorithm>(ocp);
*/

  // make sure predictice_configuration class initialized
  if (!predictive_configuration::initialize_success_)
  {
    predictive_configuration::initialize();
  }

  // intialize data members
  state_initialize_.resize(state_dim_);
  state_initialize_.setAll(1E-5);
  control_initialize_.resize(control_dim_);
  control_initialize_.setAll(1E-5);

  // initialize acado configuration parameters
  max_num_iteration_ = predictive_configuration::max_num_iteration_;
  kkt_tolerance_ = predictive_configuration::kkt_tolerance_;
  integrator_tolerance_ = predictive_configuration::integrator_tolerance_;

  // initialize horizons, sampling time
  start_time_ = predictive_configuration::start_time_horizon_;
  end_time_ = predictive_configuration::end_time_horizon_;
  discretization_intervals_ = predictive_configuration::discretization_intervals_;
  sampling_time_ = predictive_configuration::sampling_time_;

	// optimaization type
	// use_lagrange_term_ = predictive_configuration::use_lagrange_term_;
	use_LSQ_term_ = predictive_configuration::use_LSQ_term_;
	use_mayer_term_ = predictive_configuration::use_mayer_term_;

	// initialize state and control weight factors
    lsq_state_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_state_weight_factors_);
    lsq_control_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_control_weight_factors_);

  //state_vector_size_ = predictive_configuration::lsq_state_weight_factors_.size();
  //control_vector_size_ = predictive_configuration::lsq_control_weight_factors_.size();
  state_vector_size_ = lsq_state_weight_factors_.rows()* lsq_state_weight_factors_.cols();
  control_vector_size_ = lsq_control_weight_factors_.rows()*lsq_control_weight_factors_.cols();
  horizon_steps_ = (int)(end_time_/sampling_time_);

	// intialize parameters
	//param_.reset(new VariablesGrid(state_dim_,horizon_steps_));
	//param_->setAll(0.0);

	control_min_constraint_ = transformStdVectorToEigenVector(predictive_configuration::vel_min_limit_);
	control_max_constraint_ = transformStdVectorToEigenVector(predictive_configuration::vel_max_limit_);

  // slef collision cost constant term
  self_collision_cost_constant_term_ = discretization_intervals_/ (end_time_-start_time_);

	//move to a kinematic function
	// Differential Kinematic
	iniKinematics(x_,v_);
	s_ = 0;

  // Initialize parameters concerning obstacles
  n_obstacles_ = predictive_configuration::n_obstacles_;

  computeEgoDiscs();

  ROS_WARN("PD_FRAME_TRACKER INITIALIZED!!");
  return true;
}

void pd_frame_tracker::iniKinematics(const DifferentialState& x, const Control& v){

	ROS_WARN("pd_frame_tracker::iniKinematics");

	f.reset();
	f << dot(x(0)) == v(1)*x(1)-v(0)*cos(x(2));
	f << dot(x(1)) == -v(1)*x(0)-v(0)*sin(x(2));
	f << dot(x(2)) == -v(1);
	//f << dot(x(3)) == v(0);
}

void pd_frame_tracker::computeEgoDiscs()
{
    // Collect parameters for disc representation
    int n_discs = predictive_configuration::n_discs_;
    double length = predictive_configuration::ego_l_;
    double width = predictive_configuration::ego_w_;

    // Initialize positions of discs
    x_discs_.resize(n_discs);

    // Loop over discs and assign positions
    for ( int discs_it = 0; discs_it < n_discs; discs_it++){
        x_discs_[discs_it] = -length/2 + (discs_it + 1)*(length/(n_discs + 1));
    }

    // Compute radius of the discs
    r_discs_ = sqrt(pow(x_discs_[n_discs - 1] - length/2,2) + pow(width/2,2));

    ROS_WARN_STREAM("Generated " << n_discs <<  " ego-vehicle discs with radius " << r_discs_);
}

// calculate quternion product
void pd_frame_tracker::calculateQuaternionProduct(const geometry_msgs::Quaternion& quat_1, const geometry_msgs::Quaternion& quat_2, geometry_msgs::Quaternion& quat_resultant)
{
  Eigen::Vector3d quat_1_lcl(quat_1.x, quat_1.y, quat_1.z);
  Eigen::Vector3d quat_2_lcl(quat_2.x, quat_2.y, quat_2.z);

  // cross product
  Eigen::Vector3d cross_prod = quat_1_lcl.cross(quat_2_lcl);

  // multiplication of quternion
  // Modelling and Control of Robot Manipulators by L. Sciavicco and B. Siciliano
  quat_resultant.w = ( (quat_1.w * quat_2.w) - (quat_1_lcl.dot(quat_2_lcl)) );
  quat_resultant.x = ( (quat_1.w * quat_2.x) + (quat_2.w * quat_1.x) + cross_prod(0) );
  quat_resultant.y = ( (quat_1.w * quat_2.y) + (quat_2.w * quat_1.y) + cross_prod(1) );
  quat_resultant.z = ( (quat_1.w * quat_2.z) + (quat_2.w * quat_1.z) + cross_prod(2) );
}

// calculate inverse of quternion
void pd_frame_tracker::calculateQuaternionInverse(const geometry_msgs::Quaternion& quat, geometry_msgs::Quaternion& quat_inv)
{
  // Modelling and Control of Robot Manipulators by L. Sciavicco and B. Siciliano
  quat_inv = geometry_msgs::Quaternion();
  quat_inv.w = quat.w;
  quat_inv.x = -quat.x;
  quat_inv.y = -quat.y;
  quat_inv.z = -quat.z;
}

// get transformation matrix between source and target frame
bool pd_frame_tracker::getTransform(const std::string& from, const std::string& to,
                                    Eigen::VectorXd& stamped_pose,
                                    geometry_msgs::Quaternion &quat_msg)
{
  bool transform = false;
  stamped_pose = Eigen::VectorXd(6);
  tf::StampedTransform stamped_tf;

  // make sure source and target frame exist
  if (tf_listener_.frameExists(to) & tf_listener_.frameExists(from))
  {
    try
    {
      // find transforamtion between souce and target frame
      tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.2));
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

      // filled quaternion stamped pose
      quat_msg.w = quat.w();
      quat_msg.x = quat.x();
      quat_msg.y = quat.y();
      quat_msg.z = quat.z();

      tf::Matrix3x3 quat_matrix(quat);
      quat_matrix.getRPY(stamped_pose(3), stamped_pose(4), stamped_pose(5));

      transform = true;
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("pd_frame_tracker::getTransform: %s", ex.what());
    }
  }

  else
  {
    ROS_WARN("pd_frame_tracker::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",
             from.c_str(), to.c_str());
  }

  return transform;
}

// Generate collision cost used to avoid self collision
void pd_frame_tracker::generateCollisionCostFunction(OCP& OCP_problem,
                                                     const DifferentialState& x,
                                                     const Control& v,
                                                     const Eigen::MatrixXd& Jacobian_Matrix,
                                                     const double& total_distance,
                                                     const double& delta_t
                                                    )
{

}

void pd_frame_tracker::setCollisionConstraints(OCP& OCP_problem, const DifferentialState& x, const obstacle_feed::Obstacles& obstacles, const double& delta_t){

    // Iterate over all obstacles
    for (int obst_it = 0; obst_it < n_obstacles_; obst_it++) {
        // Iterate over all ego-vehicle discs
        for (int discs_it = 0; discs_it < predictive_configuration::n_discs_; discs_it++) {

            // Expression for position of obstacle
            double x_obst = obstacles.Obstacles[obst_it].pose.position.x;
            double y_obst = obstacles.Obstacles[obst_it].pose.position.y;

            // Size and heading of allipse
            double a = obstacles.Obstacles[obst_it].major_semiaxis + r_discs_;
            double b = obstacles.Obstacles[obst_it].minor_semiaxis + r_discs_;
            double phi = obstacles.Obstacles[obst_it].pose.orientation.z;       // Orientation is an Euler angle

            // Distance from individual ego-vehicle discs to obstacle
            Expression deltaPos(2, 1);
//            deltaPos(0) = x(0) - cos(x(2))*x_discs_[discs_it] - x_obst;
//            deltaPos(1) = x(1) - sin(x(2))*x_discs_[discs_it] - y_obst;

            deltaPos(0) = x(0) - x_obst;
            deltaPos(1) = x(1) - y_obst;

//            deltaPos(0) = deltaPos(0).getEuclideanNorm();
//            deltaPos(1) = deltaPos(1).getEuclideanNorm();
//            deltaPos(0) = x_obst - x(0);
//            deltaPos(1) = y_obst - x(1);

            // Rotation matrix corresponding to the obstacle heading
            Expression R_obst(2, 2);
            R_obst(0, 0) = cos(phi-x(2));
            R_obst(0, 1) = -sin(x(2)-x(2));
            R_obst(1, 0) = sin(phi-x(2));
            R_obst(1, 1) = cos(phi-x(2));

            // Matrix with total clearance from obstacles
            Expression ab_mat(2, 2);
            ab_mat(0, 0) = 1 / (a * a);
            ab_mat(1, 1) = 1 / (b * b);
            ab_mat(0, 1) = 0;
            ab_mat(1, 0) = 0;

            // Total contraint on obstacle
            Expression c_k;
//            c_k = deltaPos.transpose() * R_obst.transpose() * ab_mat * R_obst * deltaPos;
            c_k = deltaPos.transpose()  * ab_mat *  deltaPos;

            // Add constraint to OCP problem
            OCP_problem.subjectTo(c_k >= 1);
        }
    }
}

void pd_frame_tracker::path_function_spline_direct(OCP& OCP_problem,
												   const DifferentialState &s,
												   const Control &v,
												   const Eigen::Vector3d& goal_pose){

	DVector a_x(4),a_y(4);

	a_x(3) = ref_path_x.m_a[0];
	a_x(2) = ref_path_x.m_b[0];
	a_x(1) = ref_path_x.m_c[0];
	a_x(0) = ref_path_x.m_d[0];
	a_y(3) = ref_path_y.m_a[0];
	a_y(2) = ref_path_y.m_b[0];
	a_y(1) = ref_path_y.m_c[0];
	a_y(0) = ref_path_y.m_d[0];

	Expression x_path = (a_x(3)*s(3)*s(3)*s(3) + a_x(2)*s(3)*s(3) + a_x(1)*s(3) + a_x(0)) ;
	Expression y_path = (a_y(3)*s(3)*s(3)*s(3) + a_y(2)*s(3)*s(3) + a_y(1)*s(3) + a_y(0)) ;
	Expression dx_path = (3*a_x(3)*s(3)*s(3) + 2*a_x(2)*s(3) + a_x(1)) ;
	Expression dy_path = (3*a_y(3)*s(3)*s(3) + 2*a_y(2)*s(3) + a_y(1)) ;


	Expression abs_grad = sqrt(dx_path.getPowInt(2) + dy_path.getPowInt(2));
	Expression dx_path_norm = dx_path/abs_grad;
	Expression dy_path_norm =  dy_path/abs_grad;
	// Compute the errors
	//Expression theta_path = dy_path/dx_path;
	//theta_path = theta_path.getAtan();
	//Expression dx_path_norm = theta_path.getCos();
	//Expression dy_path_norm =  theta_path.getSin();

	Expression error_contour   = dy_path_norm * (s(0) - x_path) - dx_path_norm * (s(1) - y_path);

	Expression error_lag       = dx_path_norm * (s(0) - x_path) + dy_path_norm * (s(1) - y_path);

	OCP_problem.minimizeMayerTerm(error_contour.getPowInt(2) + error_lag.getPowInt(2) + 0.01*v.transpose() * v -0.1*s(3));

}

// Generate cost function of optimal control problem
void pd_frame_tracker::generateCostFunction(OCP& OCP_problem,
											const DifferentialState &x,
                                            const Control &v,
                                            const Eigen::Vector3d& goal_pose)
{
  if (use_mayer_term_)
  {
    if (activate_debug_output_)
    {
      ROS_INFO("pd_frame_tracker::generateCostFunction: use_mayer_term_");
    }
	  Expression sqp = lsq_state_weight_factors_(0) * x(0) * x(0)
						+lsq_state_weight_factors_(1) * x(1)* x(1)
						+lsq_state_weight_factors_(2) *x(2)* x(2)
	  					+lsq_control_weight_factors_(0) * (v.transpose() * v);

	  OCP_problem.minimizeMayerTerm( sqp );

  }

}

void pd_frame_tracker::initializeOptimalControlProblem(std::vector<double> parameters){

	ROS_INFO("pd_frame_tracker::initializeOptimalControlProblem");

}


VariablesGrid pd_frame_tracker::solveOptimalControlProblem(const Eigen::VectorXd &last_position,
												  const Eigen::Vector3d &goal_pose,
												  const obstacle_feed::Obstacles &obstacles,
												  geometry_msgs::Twist& controlled_velocity)
{

	// control initialize with previous command
	control_initialize_(0) = controlled_velocity.linear.x;
	control_initialize_(1) = controlled_velocity.angular.z;

	// state initialize
	ROS_INFO_STREAM("GOAL: " << goal_pose);
	state_initialize_(0) = goal_pose(0)-last_position(0);
	state_initialize_(1) = goal_pose(1)-last_position(1);
	state_initialize_(2) = goal_pose(2)-last_position(2);
	//state_initialize_(3) = s_;

	obstacles_ = obstacles;

	// OCP variables
	// Optimal control problem
	OCP OCP_problem_(start_time_, end_time_, discretization_intervals_);

	//Equal constraints
	OCP_problem_.subjectTo(f);

	// generate cost function
 	generateCostFunction(OCP_problem_, x_, v_, goal_pose);
  	//path_function_spline_direct(OCP_problem_, x_, v_, goal_pose);

  //setCollisionConstraints(OCP_problem_, x_, obstacles_, discretization_intervals_);

  // Optimal Control Algorithm
  RealTimeAlgorithm OCP_solver(OCP_problem_, 0.025); // 0.025 sampling time

  OCP_solver.initializeControls(control_initialize_);
  OCP_solver.initializeDifferentialStates(state_initialize_);

  setAlgorithmOptions(OCP_solver);

	OCP_solver.solve(0.0,state_initialize_);

	OCP_solver.getDifferentialStates(pred_states);


  // get control at first step and update controlled velocity vector

	OCP_solver.getU(u);

	if (u.size()>0){
		controlled_velocity.linear.x = u(0);
		controlled_velocity.angular.z = u(1);

		// Simulate path varible
		// this should be later replaced by a process
		ROS_INFO_STREAM("Path distance: " << s_);
		s_ += u(0) * sampling_time_;
		s_ = 0;
	}

	return pred_states;
}

// setup acado algorithm options, need to set solver when calling this function
void pd_frame_tracker::setAlgorithmOptions(RealTimeAlgorithm& OCP_solver)
{

  OCP_solver.set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
  OCP_solver.set(PRINT_SCP_METHOD_PROFILE, false);        // default false
  OCP_solver.set(PRINT_COPYRIGHT, false);                 // default true


  OCP_solver.set(MAX_NUM_ITERATIONS, max_num_iteration_);
  OCP_solver.set(LEVENBERG_MARQUARDT, integrator_tolerance_);
  OCP_solver.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
  OCP_solver.set( DISCRETIZATION_TYPE, COLLOCATION);
  OCP_solver.set(KKT_TOLERANCE, kkt_tolerance_);
  OCP_solver.set(HOTSTART_QP, true);                   // default true
  OCP_solver.set(SPARSE_QP_SOLUTION, CONDENSING);      // CONDENSING, FULL CONDENSING, SPARSE SOLVER


  OCP_solver.set(INFEASIBLE_QP_HANDLING, defaultInfeasibleQPhandling);
  OCP_solver.set(INFEASIBLE_QP_RELAXATION, defaultInfeasibleQPrelaxation);


}
