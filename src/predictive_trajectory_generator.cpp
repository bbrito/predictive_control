
//This file containts cost function intsert in to generated trajectory.

#include <predictive_control/predictive_trajectory_generator.h>

pd_frame_tracker::pd_frame_tracker()
{
  //clearDataMember();
}

pd_frame_tracker::~pd_frame_tracker()
{
  clearDataMember();
}

// diallocated memory
void pd_frame_tracker::clearDataMember()
{
  // resize matrix and vectors
  const int jacobian_matrix_rows = 6, jacobian_matrix_columns = predictive_configuration::degree_of_freedom_;
  Jacobian_Matrix_.resize(jacobian_matrix_rows, jacobian_matrix_columns);
  state_initialize_.resize(jacobian_matrix_rows);
  control_initialize_.resize(jacobian_matrix_columns);

  control_min_constraint_.resize(jacobian_matrix_columns);
  control_max_constraint_.resize(jacobian_matrix_columns);

  lsq_state_weight_factors_.resize(jacobian_matrix_rows);
  lsq_control_weight_factors_.resize(jacobian_matrix_columns);
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
  const int jacobian_matrix_rows = 6, jacobian_matrix_columns = predictive_configuration::degree_of_freedom_;
  Jacobian_Matrix_.resize(jacobian_matrix_rows, jacobian_matrix_columns);
  state_initialize_.resize(jacobian_matrix_rows);
  state_initialize_.setAll(0.0);
  control_initialize_.resize(jacobian_matrix_columns);
  control_initialize_.setAll(0.0);

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
  use_lagrange_term_ = predictive_configuration::use_lagrange_term_;
  use_LSQ_term_ = predictive_configuration::use_LSQ_term_;
  use_mayer_term_ = predictive_configuration::use_mayer_term_;

  // initialize state and control weight factors
  if (use_LSQ_term_)
  {
    lsq_state_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_state_weight_factors_);
    lsq_control_weight_factors_ = transformStdVectorToEigenVector(predictive_configuration::lsq_control_weight_factors_);
  }

  //state_vector_size_ = predictive_configuration::lsq_state_weight_factors_.size();
  //control_vector_size_ = predictive_configuration::lsq_control_weight_factors_.size();
  state_vector_size_ = lsq_state_weight_factors_.rows()* lsq_state_weight_factors_.cols();
  control_vector_size_ = lsq_control_weight_factors_.rows()*lsq_control_weight_factors_.cols();

  // initialize hard constraints vector
  control_min_constraint_ = transformStdVectorToEigenVector(predictive_configuration::joints_vel_min_limit_);
  control_max_constraint_ = transformStdVectorToEigenVector(predictive_configuration::joints_vel_max_limit_);

  // slef collision cost constant term
  self_collision_cost_constant_term_ = discretization_intervals_/ (end_time_-start_time_);

  ROS_WARN("PD_FRAME_TRACKER INITIALIZED!!");
  return true;
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
bool pd_frame_tracker::getTransform(const std::string& from, const std::string& to, Eigen::VectorXd& stamped_pose)
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
  if (use_mayer_term_)
  {
    ;
  }

  if (use_lagrange_term_)
  {

    DVector normal_vector(Jacobian_Matrix.rows());
    normal_vector.setAll(1.0);

    DVector create_expression_vec = -(normal_vector.transpose() * Jacobian_Matrix_);

    Expression expression(create_expression_vec);
    // http://doc.aldebaran.com/2-1/naoqi/motion/reflexes-collision-avoidance.html
    expression = expression.transpose() * v + total_distance * (self_collision_cost_constant_term_);
    //  d / t , t = 1.0 / (L/n)

    OCP_problem.minimizeLagrangeTerm(expression);

  }

  if (use_LSQ_term_)
  {
    DVector normal_vector(Jacobian_Matrix.rows());
    normal_vector.setAll(1.0);

    DVector create_expression_vec = -(normal_vector.transpose() * Jacobian_Matrix_);

     Expression expression(create_expression_vec);
     // http://doc.aldebaran.com/2-1/naoqi/motion/reflexes-collision-avoidance.html
     expression = expression.transpose() * v + total_distance * (self_collision_cost_constant_term_);
     //  d / t , t = 1.0 / (L/n)

    Function h;
    h << expression;

    DMatrix Q(1,1);
    Q(0,0) = 1.0;

    DVector ref(1);
    ref.setAll(0.0);

    // create objective function
    OCP_problem.minimizeLSQ(Q, h, ref);

    // set constraints related to collision cost
    //OCP_problem.subjectTo(0.0 <= expression <= 5.0);
    OCP_problem.subjectTo(expression <= 5.0);
  }

}

// Generate cost function of optimal control problem
void pd_frame_tracker::generateCostFunction(OCP &OCP_problem,
                                            const DifferentialState &x,
                                            const Control &v,
                                            const Eigen::VectorXd& goal_pose)
{
  if (use_mayer_term_)
  {
    OCP_problem.minimizeMayerTerm( 10.0 * ( (x(0) - goal_pose(0)) * (x(0) - goal_pose(0))
                                             +(x(1) - goal_pose(1)) * (x(1) - goal_pose(1))
                                             +(x(2) - goal_pose(2)) * (x(2) - goal_pose(2)) )
                                     + 1.0 *( (x(3) - goal_pose(3)) * (x(3) - goal_pose(3))
                                             +(x(4) - goal_pose(4)) * (x(4) - goal_pose(4))
                                             +(x(5) - goal_pose(5)) * (x(5) - goal_pose(5)) )
                                     + 10.0 * (v.transpose() * v)
                                  );

    //OCP_problem.minimizeMayerTerm( 10.0* ( (x-goal_pose) * (x-goal_pose) ) + 1.00* (v.transpose() * v) );

  }

  if (use_lagrange_term_)
  {
    ;
  }

  if (use_LSQ_term_)
  {
   /*
    // Solve Ax = b using LSQ method where A is weight matrix, x is function to be compute, b reference vector is zero

    Function h, t;

    // initialize function with states
    uint32_t state_vector_size = x.getNumRows()*x.getNumCols();
    uint32_t goal_vector_size = goal_pose.rows() * goal_pose.cols();
    for (int i = 0u; i < (state_vector_size && goal_vector_size); ++i)
    {
      h << ( x(i) - goal_pose(i) );
    }

    // initialize function with controls
    uint32_t control_vector_size = v.getNumRows()*v.getNumCols();
    for (int i = 0u; i < state_vector_size; ++i)
    {
      h << v(i);
    }

    // weighting matrix
    DMatrix Q(h.getDim(), h.getDim());
    // weighting of state weight
    uint32_t lsq_state_vector_size = lsq_state_weight_factors_.rows() * lsq_state_weight_factors_.cols();
    for (int i = 0u; i < (state_vector_size && lsq_state_vector_size); ++i)
    {
      Q(i,i) = lsq_state_weight_factors_(i);
    }

    // weighting of control weight, should filled after state wieghting factor
    uint32_t lsq_control_vector_size = lsq_control_weight_factors_.rows() * lsq_control_weight_factors_.cols();
    for (int i = (state_vector_size); i < (state_vector_size && lsq_control_vector_size); ++i)
    {
      Q(i,i) = lsq_control_weight_factors_(i);
    }

    // reference vectors
    DVector r(h.getDim());
    r.setAll(0.0);

    OCP_problem.minimizeLSQ(Q, h, r);*/
    // Solve Ax = b using LSQ method where A is weight matrix, x is function to be compute, b reference vector is zero

    Function h;

    //uint32_t state_vector_size = lsq_state_weight_factors_.rows()* lsq_state_weight_factors_.cols();
    //uint32_t control_vector_size = lsq_control_weight_factors_.rows()*lsq_control_weight_factors_.cols();

    // initialize function with states
    for (int i = 0u; i < (state_vector_size_ ); ++i) //&& goal_vector_size
    {
      h << ( x(i) - goal_pose(i) );
    }

    // initialize function with controls
    for (int i = 0u; i < control_vector_size_; ++i)
    {
      h << v(i);
    }

    // initialization of weighting matrix
    DMatrix Q(h.getDim(), h.getDim());

    // weighting of state weight
    for (int i = 0u; i < (state_vector_size_); ++i) //&& lsq_state_vector_size
    {
      Q(i,i) = lsq_state_weight_factors_(i);
    }

    // weighting of control weight, should filled after state wieghting factor
    for (int i = 0u, j = (state_vector_size_); i < (control_vector_size_); ++i, ++j) // && lsq_control_vector_size
    {
      Q(j,j) = lsq_control_weight_factors_(i);
    }

    // reference vectors
    DVector r(h.getDim());
    r.setAll(0.0);

    OCP_problem.minimizeLSQ(Q, h, r);
    //OCP_problem.minimizeLSQ(Q, h, r);
    //OCP_problem.minimizeLSQEndTerm(Q, t, r);
    //OCP_problem.minimizeLSQEndTerm(Q_t, t, r_t);
    //OCP_problem.minimizeLSQEndTerm(Q_v, t_v, r_v);

  }

}




void pd_frame_tracker::solveOptimalControlProblem(const Eigen::MatrixXd &Jacobian_Matrix,
                                                  const Eigen::VectorXd &last_position,
                                                  const Eigen::VectorXd &goal_pose,
                                                  const Eigen::VectorXd& self_collision_vector,
                                                  std_msgs::Float64MultiArray& controlled_velocity)
{
  Jacobian_Matrix_ = Jacobian_Matrix;
  // control initialize
  control_initialize_(0) = controlled_velocity.data[0];
  control_initialize_(1) = controlled_velocity.data[1];
  control_initialize_(2) = controlled_velocity.data[2];
  control_initialize_(3) = controlled_velocity.data[3];
  control_initialize_(4) = controlled_velocity.data[4];
  control_initialize_(5) = controlled_velocity.data[5];
  control_initialize_(6) = controlled_velocity.data[6];

  // state initialize
  state_initialize_(0) = last_position(0);
  state_initialize_(1) = last_position(1);
  state_initialize_(2) = last_position(2);
  state_initialize_(3) = last_position(3);
  state_initialize_(4) = last_position(4);
  state_initialize_(5) = last_position(5);

  std::cout<<"\033[95m"<<"________________________"<<self_collision_vector.sum()<<"___________________"<<"\033[36;0m"<<std::endl;

  const unsigned int jacobian_matrix_rows = 6;//Jacobian_Matrix.rows();
  const unsigned int jacobian_matrix_columns = 7;//Jacobian_Matrix.cols();

  // OCP variables
  DifferentialState x("", jacobian_matrix_rows, 1);       // position
  Control v("", jacobian_matrix_columns, 1);            // velocity

  // Clear state, control variable
  x.clearStaticCounters();
  v.clearStaticCounters();

  // Differential Equation
  DifferentialEquation f;

  // Differential Kinematic
  f << dot(x) == (Jacobian_Matrix_ * v);

  // Optimal control problem
  // here end time interpriate as control and/or prdiction horizon, choose maximum 4.0 till that gives better results
  OCP OCP_problem( start_time_, end_time_, discretization_intervals_);

  // generate cost function
  generateCostFunction(OCP_problem, x, v, goal_pose);

  // generate collision cost function
  generateCollisionCostFunction(OCP_problem, v, Jacobian_Matrix_, self_collision_vector.sum(), 0.0);

  OCP_problem.subjectTo(f);
  OCP_problem.subjectTo(-0.50 <= v <= 0.50);
 // OCP_problem.subjectTo(AT_START, v == );
  OCP_problem.subjectTo(AT_END, v == control_initialize_); //0.0

  // Optimal Control Algorithm
  RealTimeAlgorithm OCP_solver(OCP_problem, 0.025); // 0.025 sampling time

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
  ROS_WARN("================");
  u.print();
  ROS_WARN("================");
  controlled_velocity.data.resize(jacobian_matrix_columns, 0.0);
  for (int i=0u; i < jacobian_matrix_columns; ++i)
  {
    controlled_velocity.data[i] = u(i);
  }

 /* controlled_velocity.data[0] = u(0);
  controlled_velocity.data[1] = u(1);
  controlled_velocity.data[2] = u(2);
  controlled_velocity.data[3] = u(3);
  controlled_velocity.data[4] = u(4);
  controlled_velocity.data[5] = u(5);
  controlled_velocity.data[6] = u(6);*/
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
  //OCP_solver.set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
  //OCP_solver.set(PRINT_SCP_METHOD_PROFILE, false);        // default false
  //OCP_solver.set(PRINT_COPYRIGHT, false);                 // default true


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
