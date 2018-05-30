
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
	param_.reset(new VariablesGrid(state_dim_,horizon_steps_));
	param_->setAll(0.0);

	control_min_constraint_ = transformStdVectorToEigenVector(predictive_configuration::vel_min_limit_);
	control_max_constraint_ = transformStdVectorToEigenVector(predictive_configuration::vel_max_limit_);

  // slef collision cost constant term
  self_collision_cost_constant_term_ = discretization_intervals_/ (end_time_-start_time_);

	//move to a kinematic function
	// Differential Kinematic
	iniKinematics(x_,v_);

  ROS_WARN("PD_FRAME_TRACKER INITIALIZED!!");
  return true;
}

void pd_frame_tracker::iniKinematics(const DifferentialState& x, const Control& v){

	ROS_WARN("pd_frame_tracker::iniKinematics");

	f.reset();
	f << dot(x(0)) == v(0)*cos(x(2));
	f << dot(x(1)) == v(0)*sin(x(2));
	f << dot(x(2)) == v(1);
	f << dot(x(3)) == v(0);
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
                                                     const Control& v,
                                                     const Eigen::MatrixXd& Jacobian_Matrix,
                                                     const double& total_distance,
                                                     const double& delta_t)
{
  //TO BE IMPLEMENTED
}

void pd_frame_tracker::path_function_spline_direct(DifferentialState& s){

	if (activate_debug_output_) {
		ROS_WARN("pd_frame_tracker::path_function_spline_direct");
		for(int i=0;i<10;i++)
			ROS_INFO_STREAM("p(" << i << "): " << *(p+i));
	}

	int i = 1;
	int z = 500;
	int N_SPLINE_POINTS = 30;
	auto t = s(3) - (*(p+3))*(i -1 + (*(p+4)));
	ROS_INFO_STREAM("t: " << t);
	auto fac = 1/exp(500*(t - *(p+3))); // implements the if_else condition in a continous way

	double dist_spline_pts = *(p+3);
	double break_index = *(p+4);

	volatile auto a_x_3 = *(p+z + (i-1)*8 + 1);
	volatile auto a_x_2 = *(p+z  + (i-1)*8 + 2);
	volatile auto a_x_1 = *(p+z  + (i-1)*8 + 3);
	volatile auto a_x_0 = *(p+z  + (i-1)*8 + 4);
	volatile auto a_y_3 = *(p+z  + (i-1)*8 + 5);
	volatile auto a_y_2 = *(p+z  + (i-1)*8 + 6);
	volatile auto a_y_1 = *(p+z  + (i-1)*8 + 7);
	volatile auto a_y_0 = *(p+z  + (i-1)*8 + 8);

	//param_->setAll(*(p+3));

	/*auto x_path = x_path + (a_x_3*t^3 + a_x_2*t^2 + a_x_1*t + a_x_0) * fac;
	auto y_path = y_path + (a_y_3*t^3 + a_y_2*t^2 + a_y_1*t + a_y_0) * fac;
	auto dx_path = dx_path + (3*a_x_3*t^2 + 2*a_x_2*t + a_x_1) * fac;
	auto dy_path = dy_path + (3*a_y_3*t^2 + 2*a_y_2*t + a_y_1) * fac;

	for(i = 2;N_SPLINE_POINTS-2;i++) {
		t = s - dist_spline_pts * (i - 1 + break_index);
		//fac = if_else(0 <= t, 1, 0);
		//fac = if_else(t < dist_spline_pts, fac, 0);

		a_x_3 = *(p+z+ (i - 1) * 8 + 1);
		a_x_2 = *(p+z+ (i - 1) * 8 + 2);
		a_x_1 = *(p+z+ (i - 1) * 8 + 3);
		a_x_0 = *(p+z+ (i - 1) * 8 + 4);
		a_y_3 = *(p+z+ (i - 1) * 8 + 5);
		a_y_2 = *(p+z+ (i - 1) * 8 + 6);
		a_y_1 = *(p+z+ (i - 1) * 8 + 7);
		a_y_0 = *(p+z+ (i - 1) * 8 + 8);

		x_path = x_path + (a_x_3 * t ^ 3 + a_x_2 * t ^ 2 + a_x_1 * t + a_x_0) * fac;
		y_path = y_path + (a_y_3 * t ^ 3 + a_y_2 * t ^ 2 + a_y_1 * t + a_y_0) * fac;
		dx_path = dx_path + (3 * a_x_3 * t ^ 2 + 2 * a_x_2 * t + a_x_1) * fac;
		dy_path = dy_path + (3 * a_y_3 * t ^ 2 + 2 * a_y_2 * t + a_y_1) * fac;
	}

	i = N_SPLINE_POINTS-1;
	t = s - dist_spline_pts*(i-1 + break_index);
	fac = if_else( 0 <= t, 1, 0);

	a_x_3 = *(p+z+ (i-1)*8 + 1);
	a_x_2 = *(p+z+ (i-1)*8 + 2);
	a_x_1 = *(p+z+ (i-1)*8 + 3);
	a_x_0 = *(p+z+ (i-1)*8 + 4);
	a_y_3 = *(p+z+ (i-1)*8 + 5);
	a_y_2 = *(p+z+ (i-1)*8 + 6);
	a_y_1 = *(p+z+ (i-1)*8 + 7);
	a_y_0 = *(p+z+ (i-1)*8 + 8);

	x_path = x_path + (a_x_3*t^3 + a_x_2*t^2 + a_x_1*t + a_x_0) * fac;
	y_path = y_path + (a_y_3*t^3 + a_y_2*t^2 + a_y_1*t + a_y_0) * fac;
	dx_path = dx_path + (3*a_x_3*t^2 + 2*a_x_2*t + a_x_1) * fac;
	dy_path = dy_path + (3*a_y_3*t^2 + 2*a_y_2*t + a_y_1) * fac;
	 */
}

// Generate cost function of optimal control problem
void pd_frame_tracker::generateCostFunction(OCP& OCP_problem,
											const DifferentialState &x,
                                            const Control &v,
                                            const Eigen::VectorXd& goal_pose)
{
  if (use_mayer_term_)
  {
    if (activate_debug_output_)
    {
      ROS_INFO("pd_frame_tracker::generateCostFunction: use_mayer_term_");
    }

	  OCP_problem.minimizeMayerTerm( lsq_state_weight_factors_(0) * ( (x(0) - goal_pose(0)) * (x(0) - goal_pose(0)) )
                                             +lsq_state_weight_factors_(1) * ( (x(1) - goal_pose(1)) * (x(1) - goal_pose(1)))
                                             +lsq_state_weight_factors_(2) * ( (x(2) - goal_pose(2)) * (x(2) - goal_pose(2)))
                                     + lsq_control_weight_factors_(0) * (v.transpose() * v)
                                  );

    //OCP_problem.minimizeMayerTerm( 10.0* ( (x-goal_pose) * (x-goal_pose) ) + 1.00* (v.transpose() * v) );

  }

}

void pd_frame_tracker::initializeOptimalControlProblem(std::vector<double> parameters){

	ROS_INFO("pd_frame_tracker::initializeOptimalControlProblem");
	int z = 500; //just because it is used in matlab like this... it should be changed
	for(int i = 0; i < parameters.size();i++){
		/*
		param_->operator() (i,0) = parameters[i]; //a_x_3
		param_->operator() (i,1) = parameters[i]; //a_x_2
		param_->operator() (i,2) = parameters[i]; //a_x_1
		param_->operator() (i,3) = parameters[i]; //a_x_0
		param_->operator() (i,4) = parameters[i]; //a_y_3
		param_->operator() (i,5) = parameters[i]; //a_y_2
		param_->operator() (i,6) = parameters[i]; //a_y_1
		param_->operator() (i,7) = parameters[i]; //a_y_0
		 */
	}
}


void pd_frame_tracker::solveOptimalControlProblem(const Eigen::VectorXd &last_position,
												  const Eigen::VectorXd &goal_pose,
												  geometry_msgs::Twist& controlled_velocity)
{

	// control initialize with previous command
	control_initialize_(0) = controlled_velocity.linear.x;
	control_initialize_(1) = controlled_velocity.angular.z;

	// state initialize
	state_initialize_(0) = last_position(0);
	state_initialize_(1) = last_position(1);
	state_initialize_(2) = last_position(2);

	// OCP variables

	//pd_frame_tracker::path_function_spline_direct(x);
	// Optimal control problem
	OCP OCP_problem_(start_time_, end_time_, discretization_intervals_);

	//Equal consttraints
	OCP_problem_.subjectTo(f);

  // generate cost function
  generateCostFunction(OCP_problem_, x_, v_, goal_pose);

  // generate collision cost function
  //TO BE IMPLEMENTED
  /* if (self_collision_vector > (0.15 + 0.10))
  {
    generateCollisionCostFunction(OCP_problem, v, Jacobian_Matrix_, self_collision_vector, 0.0);
  }

  //--------------------------------------------------- static collision cost ------------------
  DVector normal_vector(3);
  normal_vector.setAll(1.0);

  DVector create_expression_vec = -(normal_vector.transpose() * Jacobian_Matrix_);

   Expression expression(create_expression_vec);
   // http://doc.aldebaran.com/2-1/naoqi/motion/reflexes-collision-avoidance.html
   expression = expression.transpose() * v + static_collision_vector.sum() * (self_collision_cost_constant_term_);
   //  d / t , t = 1.0 / (L/n)

  Function h;
  h << expression;

  DMatrix Q(1,1);
  Q(0,0) = 0.0001;

  DVector ref(1);
  ref.setAll(0.0);
*/
  /*if (self_collision_vector.sum() >= 1000)
  {
    Q(0,0) = 0.001;
  }*/

    // create objective function
    //OCP_problem.minimizeLSQ(Q, h, ref);

    // set constraints related to collision cost
    //OCP_problem.subjectTo(0.0 <= expression <= 5.0);
    //OCP_problem.subjectTo(expression <= 5.0);


  //-----------------------------------------------------------------------------------------------------

  //OCP_problem.subjectTo(vel_min_limit_ <= v <= vel_max_limit_);
  // OCP_problem.subjectTo(AT_START, v == );
  //OCP_problem.subjectTo(AT_END, v == control_initialize_); //0.0

  // Optimal Control Algorithm
  RealTimeAlgorithm OCP_solver(OCP_problem_, 0.025); // 0.025 sampling time

  OCP_solver.initializeControls(control_initialize_);
  OCP_solver.initializeDifferentialStates(state_initialize_);

  setAlgorithmOptions(OCP_solver);

  // setup controller
  Controller controller(OCP_solver);
  controller.init(0.0, state_initialize_);
  controller.step(0.0, state_initialize_);

  // get control at first step and update controlled velocity vector
  DVector u;
  controller.getU(u);
  if (activate_debug_output_) {
    ROS_WARN("================");
    u.print();
    ROS_WARN("================");
  }
  controlled_velocity.linear.x = u(0);
  controlled_velocity.angular.z = u(1);

	// Clear state, control variable
	clearAllStaticCounters();

}

// setup acado algorithm options, need to set solver when calling this function
void pd_frame_tracker::setAlgorithmOptions(RealTimeAlgorithm& OCP_solver)
{
 /* // intergrator
  OCP_solver.set(INTEGRATOR_TYPE, LEVENBERG_MARQUARDT);                                      // default INT_RK45: (INT_RK12, INT_RK23, INT_RK45, INT_RK78, INT_BDF)
  OCP_solver.set(INTEGRATOR_TOLERANCE, 1e-5);     // default 1e-8

  // SQP method
  OCP_solver.set(HESSIAN_APPROXIMATION, EXACT_HESSIAN);                                      // default BLOCK_BFGS_UPDATE: (CONSTANT_HESSIAN, FULL_BFGS_UPDATE, BLOCK_BFGS_UPDATE, GAUSS_NEWTON, EXACT_HESSIAN)
  OCP_solver.set(MAX_NUM_ITERATIONS, 10);          // default 1000
  //OCP_solver.set(KKT_TOLERANCE, predictive_configuration::kkt_tolerance_);                   // default 1e-6
//  OCP_solver.set(HOTSTART_QP, true);                   // default true
//  OCP_solver.set(SPARSE_QP_SOLUTION, CONDENSING);      // CONDENSING, FULL CONDENSING, SPARSE SOLVER

  // Discretization Type
  OCP_solver.set(DISCRETIZATION_TYPE, COLLOCATION); // default MULTIPLE_SHOOTING: (SINGLE_SHOOTING, MULTIPLE_SHOOTING)
  //OCP_solver.set(TERMINATE_AT_CONVERGENCE, true);         // default true
*/
  // output
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

  // Discretization Type
  //OCP_solver.set(TERMINATE_AT_CONVERGENCE, true);         // default true
}


/*
// setup acado algorithm options, need to set solver when calling this function
template<typename T>
void pd_frame_tracker::setAlgorithmOptions(boost::shared_ptr<T> OCP_solver)
{
  // intergrator
  OCP_solver->set(INTEGRATOR_TYPE, LEVENBERG_MARQUARDT);                                      // default INT_RK45: (INT_RK12, INT_RK23, INT_RK45, INT_RK78, INT_BDF)
  OCP_solver->set(INTEGRATOR_TOLERANCE, predictive_configuration::integrator_tolerance_);     // default 1e-8

  // SQP method
  OCP_solver->set(HESSIAN_APPROXIMATION, EXACT_HESSIAN);                                      // default BLOCK_BFGS_UPDATE: (CONSTANT_HESSIAN, FULL_BFGS_UPDATE, BLOCK_BFGS_UPDATE, GAUSS_NEWTON, EXACT_HESSIAN)
  OCP_solver->set(MAX_NUM_ITERATIONS, predictive_configuration::max_num_iteration_);          // default 1000
  OCP_solver->set(KKT_TOLERANCE, predictive_configuration::kkt_tolerance_);                   // default 1e-6
//  OCP_solver->set(HOTSTART_QP, true);                   // default true
//  OCP_solver->set(SPARSE_QP_SOLUTION, CONDENSING);      // CONDENSING, FULL CONDENSING, SPARSE SOLVER

  // Discretization Type
  OCP_solver->set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // default MULTIPLE_SHOOTING: (SINGLE_SHOOTING, MULTIPLE_SHOOTING)
  OCP_solver->set(TERMINATE_AT_CONVERGENCE, true);         // default true

  // output
  OCP_solver->set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
  OCP_solver->set(PRINT_SCP_METHOD_PROFILE, false);        // default false
  OCP_solver->set(PRINT_COPYRIGHT, false);                 // default true

}
*/

/*
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
MatrixVariablesGrid:
    shiftBackwards (Matrix lastValue=emptyMatrix)
    Shifts all grid points backwards by one grid point, deleting the first one and doubling the value at last grid point.
    shiftTimes (double timeShift)
    Shifts times at all grid points by a given offset.
Optimization Algorithm:
    double  getObjectiveValue () const
    returnValue     getSensitivitiesX
// add NLP solver options
    addOption( MAX_NUM_ITERATIONS          , defaultMaxNumIterations        );
    addOption( KKT_TOLERANCE               , defaultKKTtolerance            );
    addOption( KKT_TOLERANCE_SAFEGUARD     , defaultKKTtoleranceSafeguard   );
    addOption( LEVENBERG_MARQUARDT         , defaultLevenbergMarguardt      );
    addOption( HESSIAN_PROJECTION_FACTOR   , defaultHessianProjectionFactor );
    addOption( PRINTLEVEL                  , defaultPrintlevel              );
    addOption( PRINT_COPYRIGHT             , defaultPrintCopyright          );
    addOption( HESSIAN_APPROXIMATION       , defaultHessianApproximation    );
    addOption( DYNAMIC_HESSIAN_APPROXIMATION, defaultDynamicHessianApproximation );
    addOption( DYNAMIC_SENSITIVITY         , defaultDynamicSensitivity      );
    addOption( OBJECTIVE_SENSITIVITY       , defaultObjectiveSensitivity    );
    addOption( CONSTRAINT_SENSITIVITY      , defaultConstraintSensitivity   );
    addOption( DISCRETIZATION_TYPE         , defaultDiscretizationType      );
    addOption( LINESEARCH_TOLERANCE        , defaultLinesearchTolerance     );
    addOption( MIN_LINESEARCH_PARAMETER    , defaultMinLinesearchParameter  );
    addOption( MAX_NUM_QP_ITERATIONS       , defaultMaxNumQPiterations      );
    addOption( HOTSTART_QP                 , defaultHotstartQP              );
    addOption( INFEASIBLE_QP_RELAXATION    , defaultInfeasibleQPrelaxation  );
    addOption( INFEASIBLE_QP_HANDLING      , defaultInfeasibleQPhandling    );
    addOption( USE_REALTIME_ITERATIONS     , defaultUseRealtimeIterations   );
    addOption( TERMINATE_AT_CONVERGENCE    , defaultTerminateAtConvergence  );
    addOption( SPARSE_QP_SOLUTION          , defaultSparseQPsolution        );
    addOption( GLOBALIZATION_STRATEGY      , defaultGlobalizationStrategy   );
    addOption( PRINT_SCP_METHOD_PROFILE    , defaultprintSCPmethodProfile   );
    // add integration options
    addOption( FREEZE_INTEGRATOR           , defaultFreezeIntegrator        );
    addOption( INTEGRATOR_TYPE             , defaultIntegratorType          );
    addOption( FEASIBILITY_CHECK           , defaultFeasibilityCheck        );
    addOption( PLOT_RESOLUTION             , defaultPlotResoltion           );

    // add integrator options
    addOption( MAX_NUM_INTEGRATOR_STEPS    , defaultMaxNumSteps             );
    addOption( INTEGRATOR_TOLERANCE        , defaultIntegratorTolerance     );
    addOption( ABSOLUTE_TOLERANCE          , defaultAbsoluteTolerance       );
    addOption( INITIAL_INTEGRATOR_STEPSIZE , defaultInitialStepsize         );
    addOption( MIN_INTEGRATOR_STEPSIZE     , defaultMinStepsize             );
    addOption( MAX_INTEGRATOR_STEPSIZE     , defaultMaxStepsize             );
    addOption( STEPSIZE_TUNING             , defaultStepsizeTuning          );
    addOption( CORRECTOR_TOLERANCE         , defaultCorrectorTolerance      );
    addOption( INTEGRATOR_PRINTLEVEL       , defaultIntegratorPrintlevel    );
    addOption( LINEAR_ALGEBRA_SOLVER       , defaultLinearAlgebraSolver     );
    addOption( ALGEBRAIC_RELAXATION        , defaultAlgebraicRelaxation     );
    addOption( RELAXATION_PARAMETER        , defaultRelaxationParameter     );
    addOption( PRINT_INTEGRATOR_PROFILE    , defaultprintIntegratorProfile  );
*/
