
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
  // make sure predictive_configuration class initialized
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

  control_min_constraint_ = transformStdVectorToEigenVector(predictive_configuration::vel_min_limit_);
  control_max_constraint_ = transformStdVectorToEigenVector(predictive_configuration::vel_max_limit_);

  // self collision cost constant term
  self_collision_cost_constant_term_ = discretization_intervals_/ (end_time_-start_time_);

  //move to a kinematic function
  // Differential Kinematic
  iniKinematics(x_,v_);
  s_ = 0;

  // Initialize parameters concerning obstacles
  n_obstacles_ = predictive_configuration::n_obstacles_;

  // Compute ego discs for collision constraints
  computeEgoDiscs();

  ROS_WARN("PD_FRAME_TRACKER INITIALIZED!!");
  return true;
}

void pd_frame_tracker::iniKinematics(const DifferentialState& x, const Control& v){

	ROS_WARN("pd_frame_tracker::iniKinematics");

	f.reset();
	f << dot(x(0)) == v(0)*cos(x(2));
	f << dot(x(1)) == v(0)*sin(x(2));
	f << dot(x(2)) == v(1);
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

void pd_frame_tracker::setCollisionConstraints(OCP& OCP_problem, const DifferentialState& x, const obstacle_feed::Obstacles& obstacles, const double& delta_t) {


    // Iterate over all given obstacles upto defined bound
    for (int obst_it = 0; obst_it < obstacles.Obstacles.size(); obst_it++) {
        // Iterate over all ego-vehicle discs
        for (int discs_it = 0; discs_it < predictive_configuration::n_discs_ && obst_it < n_obstacles_; discs_it++) {

            // Expression for position of obstacle
            double x_obst = obstacles.Obstacles[obst_it].pose.position.x;
            double y_obst = obstacles.Obstacles[obst_it].pose.position.y;

            // Size and heading of allipse
            double a = obstacles.Obstacles[obst_it].major_semiaxis + r_discs_;
            double b = obstacles.Obstacles[obst_it].minor_semiaxis + r_discs_;
            double phi = obstacles.Obstacles[obst_it].pose.orientation.z;       // Orientation is an Euler angle

            // Distance from individual ego-vehicle discs to obstacle
            Expression deltaPos(2, 1);
            deltaPos(0) = x(0) - cos(x(2))*x_discs_[discs_it] - x_obst;
            deltaPos(1) = x(1) - sin(x(2))*x_discs_[discs_it] - y_obst;

//            deltaPos(0) = x(0) - x_obst;
//            deltaPos(1) = x(1) - y_obst;

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
            c_k = deltaPos.transpose() * R_obst.transpose() * ab_mat * R_obst * deltaPos;
//            c_k = deltaPos.transpose()  * ab_mat *  deltaPos;

            // Add constraint to OCP problem
            OCP_problem.subjectTo(c_k >= 1);
        }
    }
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
	  Expression sqp = (x(0) - goal_pose(0)) * (x(0) - goal_pose(0))
						;

	  OCP_problem.minimizeLagrangeTerm( sqp );
  }

}

VariablesGrid pd_frame_tracker::solveOptimalControlProblem(const Eigen::VectorXd &last_position,
												  const Eigen::Vector3d &prev_pose,
												  const Eigen::Vector3d &next_pose,
												  const Eigen::Vector3d &goal_pose,
												  const obstacle_feed::Obstacles &obstacles,
												  geometry_msgs::Twist& controlled_velocity)
{

	// control initialize with previous command
	control_initialize_(0) = controlled_velocity.linear.x;
	control_initialize_(1) = controlled_velocity.angular.z;

	// state initialize
	state_initialize_(0) = last_position(0);
	state_initialize_(1) = last_position(1);
	state_initialize_(2) = last_position(2);
	//state_initialize_(3) = s_;

	obstacles_ = obstacles;

	// OCP variables
	// Optimal control problem
	OCP OCP_problem_(start_time_, end_time_);

	//Equal constraints
	OCP_problem_.subjectTo(f);

	//OCP_problem_.subjectTo(AT_START, x_(0) ==  last_position(0));
	//OCP_problem_.subjectTo(AT_START, x_(1) ==  last_position(1));
	//OCP_problem_.subjectTo(AT_START, x_(2) ==  last_position(2));
	//OCP_problem_.subjectTo(AT_START, v_(0) ==  control_initialize_(0));
	//OCP_problem_.subjectTo(AT_START, v_(1) ==  control_initialize_(1));

  // generate cost function
  generateCostFunction(OCP_problem_, x_, v_, goal_pose);
  //path_function_spline_direct(OCP_problem_, x_, v_, goal_pose);

  //setCollisionConstraints(OCP_problem_, x_, obstacles_, discretization_intervals_);

  // Optimal Control Algorithm
  RealTimeAlgorithm OCP_solver(OCP_problem_); // 0.025 sampling time

	setAlgorithmOptions(OCP_solver);
	DVector state_ini(4);
	OCP_solver.solve(0.0,state_ini);

	OCP_solver.getDifferentialStates(pred_states);


  // get control at first step and update controlled velocity vector
	//VariablesGrid control;
	//OCP_solver.getControls(control);
	//control.print(std::cout);
	DVector control;
	OCP_solver.getU(control);
	u = control;
	ROS_INFO_STREAM("cONTROL: " << u);
	if (u.size()>0){
		controlled_velocity.linear.x = u(0);
		controlled_velocity.angular.z = u(1);

		// Simulate path varible
		// this should be later replaced by a process

		s_ += u(0) * sampling_time_;
	}
	//x_.clearStaticCounters();
	//v_.clearStaticCounters();
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
  OCP_solver.set(SPARSE_QP_SOLUTION, CONDENSING);      // CONDENSING, FULL CONDENSING, SPARSE SOLVER
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
