
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
  lsq_state_weight_factors_.resize(state_dim_);
  lsq_control_weight_factors_.resize(control_dim_);

  lsq_state_terminal_weight_factors_.resize(state_dim_);
  lsq_control_terminal_weight_factors_.resize(control_dim_);
}

// initialize data member of pd_frame_tracker class
bool pd_frame_tracker::initialize()
{
  // make sure predictive_configuration class initialized
  if (!predictive_configuration::initialize_success_)
  {
    predictive_configuration::initialize();
  }
    ros::NodeHandle nh_predictive("predictive_controller");

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

  lsq_state_terminal_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_state_terminal_weight_factors_);
  lsq_control_terminal_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_control_terminal_weight_factors_);

  //state_vector_size_ = predictive_configuration::lsq_state_weight_factors_.size();
  //control_vector_size_ = predictive_configuration::lsq_control_weight_factors_.size();
  state_vector_size_ = lsq_state_weight_factors_.rows()* lsq_state_weight_factors_.cols();
  control_vector_size_ = lsq_control_weight_factors_.rows()*lsq_control_weight_factors_.cols();

  // Initialize parameters concerning obstacles
  n_obstacles_ = predictive_configuration::n_obstacles_;

    /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
    reconfigure_server_.reset(new dynamic_reconfigure::Server<predictive_control::PredictiveControllerConfig>(reconfig_mutex_, nh_predictive));
    reconfigure_server_->setCallback(boost::bind(&MPCC::reconfigureCallback,   this, _1, _2));

  // Compute ego discs for collision constraints
  computeEgoDiscs();

  ROS_WARN("PD_FRAME_TRACKER INITIALIZED!!");
  return true;
}

void pd_frame_tracker::reconfigureCallback(predictive_control::PredictiveControllerConfig& config, uint32_t level){

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

    discretization_intervals_ = config.N;
    end_time_ = config.end_time;

}

void pd_frame_tracker::iniKinematics(const DifferentialState& x, const Control& v){

	ROS_WARN("pd_frame_tracker::iniKinematics");

	/*f.reset();
	f << dot(x_) == v_*cos(theta_);
	f << dot(y_) == v_*sin(theta_);
	f << dot(theta_) == w_;*/
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

inline double get_time_second() {
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

bool pd_frame_tracker::solveOptimalControlProblem(const Eigen::VectorXd &last_position,
												  const Eigen::Vector3d &prev_pose,
												  const Eigen::Vector3d &next_pose,
												  const Eigen::Vector3d &goal_pose,
												  const obstacle_feed::Obstacles &obstacles,
												  geometry_msgs::Twist& controlled_velocity)
{
    double start_t = get_time_second();

	DifferentialState x_, y_, theta_;       // position
	Control v_,w_;                 // velocities

	DifferentialEquation f;
	f << dot(x_) == v_*cos(theta_);
	f << dot(y_) == v_*sin(theta_);
	f << dot(theta_) == w_;

	obstacles_ = obstacles;

	// OCP variables
	// Optimal control problem
	OCP OCP_problem_(start_time_, end_time_,discretization_intervals_);

	//Equal constraints
	OCP_problem_.subjectTo(f);

	OCP_problem_.subjectTo(AT_START, x_ ==  last_position(0));
	OCP_problem_.subjectTo(AT_START, y_ ==  last_position(1));
	OCP_problem_.subjectTo(AT_START, theta_ ==  last_position(2));

	//Inequality constraints
	OCP_problem_.subjectTo( control_min_constraint_(0)<= v_ <=  control_max_constraint_(0));
	OCP_problem_.subjectTo( control_min_constraint_(1)<= w_ <=  control_max_constraint_(1));

	// generate cost function
	//generateCostFunction(OCP_problem_, goal_pose);
	Expression sqp = lsq_state_weight_factors_(0)*(x_ - goal_pose(0)) * (x_ - goal_pose(0)) +
                     lsq_state_weight_factors_(1)*(y_ - goal_pose(1)) * (y_ - goal_pose(1))+
                     lsq_state_weight_factors_(2)*(theta_ - goal_pose(2)) * (theta_ - goal_pose(2))+
                     lsq_control_weight_factors_(0)*v_*v_+lsq_control_weight_factors_(1)*w_*w_;

    Expression sqp_terminal =   lsq_state_terminal_weight_factors_(0)*(x_ - goal_pose(0)) * (x_ - goal_pose(0)) +
                                lsq_state_terminal_weight_factors_(1)*(y_ - goal_pose(1)) * (y_ - goal_pose(1))+
                                lsq_state_terminal_weight_factors_(2)*(theta_ - goal_pose(2)) * (theta_ - goal_pose(2))+
                                lsq_control_terminal_weight_factors_(0)*v_*v_+lsq_control_terminal_weight_factors_(1)*w_*w_;

    OCP_problem_.minimizeLagrangeTerm( sqp );
	OCP_problem_.minimizeMayerTerm( sqp_terminal );

	//Collision constraints
	//setCollisionConstraints(OCP_problem_, x_, obstacles_, discretization_intervals_);

	// Optimal Control Algorithm
	OptimizationAlgorithm OCP_solver(OCP_problem_); // 0.025 sampling time

	//setAlgorithmOptions(OCP_solver);
	OCP_solver.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
	OCP_solver.set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
	OCP_solver.set(PRINT_SCP_METHOD_PROFILE, false);        // default false
	OCP_solver.set(PRINT_COPYRIGHT, false);                 // default true
	OCP_solver.set( DISCRETIZATION_TYPE, COLLOCATION);
	OCP_solver.set(KKT_TOLERANCE, kkt_tolerance_);
	OCP_solver.set( ABSOLUTE_TOLERANCE, 1e-4);
	OCP_solver.set( INTEGRATOR_TOLERANCE, 1e-4);
	OCP_solver.set( MAX_NUM_ITERATIONS, 1000);
	OCP_solver.set( MAX_NUM_INTEGRATOR_STEPS, 10000);
	OCP_solver.set(HOTSTART_QP, true);                   // default true

	//Intialize variable grids
	VariablesGrid s2(state_dim_,start_time_,end_time_,discretization_intervals_);
	VariablesGrid c2(control_dim_,start_time_,end_time_,discretization_intervals_);
	OCP_solver.initializeDifferentialStates(s2);
	OCP_solver.initializeControls          (c2);

	//Solve problem
	OCP_solver.solve();

	//Get prediction horizon
	OCP_solver.getDifferentialStates(pred_states);

	//Get control input
	VariablesGrid c3;
	OCP_solver.getControls(c3);

	u = c3.getFirstVector();
	//ROS_INFO_STREAM("cONTROL: " << u);
	if (u.size()>0){
		controlled_velocity.linear.x = u(0);
		controlled_velocity.angular.z = u(1);

		// Simulate path varible
		// this should be later replaced by a process

	}
	//x_.clearStaticCounters();
	//v_.clearStaticCounters();
	clearAllStaticCounters();
    double end_t = get_time_second();
    double search_time = (end_t - start_t);
    std::cout << "[RunStep] Time spent in " << "::Solving(): " << search_time << std::endl;
	return pred_states;
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
  OCP_solver.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
  OCP_solver.set( DISCRETIZATION_TYPE, COLLOCATION);
  OCP_solver.set(KKT_TOLERANCE, kkt_tolerance_);
  OCP_solver.set(HOTSTART_QP, true);                   // default true
  OCP_solver.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);      // CONDENSING, FULL CONDENSING, SPARSE SOLVER
	OCP_solver.set( MAX_NUM_INTEGRATOR_STEPS, 10000);

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
