
//This file containts cost function intsert in to generated trajectory.

#include <predictive_control/predictive_trajectory_generator.h>

pd_frame_tracker::pd_frame_tracker()
{
  clearDataMember();
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
  control_initialize_.resize(jacobian_matrix_columns);

  // initialize hard constraints vector
  control_min_constraint_ = transformStdVectorToEigenVector(predictive_configuration::joints_vel_min_limit_);
  control_max_constraint_ = transformStdVectorToEigenVector(predictive_configuration::joints_vel_max_limit_);

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


// Generate cost function of optimal control problem
void pd_frame_tracker::generateCostFunction(OCP &OCP_problem,
                                            const DifferentialState &x,
                                            const Control &v,
                                            const Eigen::VectorXd& goal_pose)
{
  if (predictive_configuration::use_mayer_term_)
  {
    //ROS_ERROR_STREAM("generateCostFunction: dimension of state: "<< x.dim);
    //ROS_ERROR_STREAM("generateCostFunction: dimension of control: "<< v.dim);
    OCP_problem.minimizeMayerTerm( 10.0 * ( (x(0) - goal_pose(0)) * (x(0) - goal_pose(0))
                                           +(x(1) - goal_pose(1)) * (x(1) - goal_pose(1))
                                           +(x(2) - goal_pose(2)) * (x(2) - goal_pose(2))
                                           +(x(3) - goal_pose(3)) * (x(3) - goal_pose(3))
                                           +(x(4) - goal_pose(4)) * (x(4) - goal_pose(4))
                                           +(x(5) - goal_pose(5)) * (x(5) - goal_pose(5)) )
                                   + 1.0 * (v.transpose() * v)
                                );

    //OCP_problem.minimizeMayerTerm( 10.0* ( (x-goal_pose) * (x-goal_pose) ) + 1.00* (v.transpose() * v) );

  }

  if (predictive_configuration::use_lagrange_term_)
  {
    ;
  }

  if (predictive_configuration::use_LSQ_term_)
  {
    ;
  }

}


std_msgs::Float64MultiArray pd_frame_tracker::solveOptimalControlProblem(const Eigen::MatrixXd &Jacobian_Matrix,
                                                                         const Eigen::VectorXd &last_position,
                                                                         const Eigen::VectorXd &goal_pose)
{

  const unsigned int jacobian_matrix_rows = Jacobian_Matrix.rows();
  const unsigned int jacobian_matrix_columns = Jacobian_Matrix.cols();

  // initialize variables
  Jacobian_Matrix_ = Jacobian_Matrix;
  state_initialize_ = last_position;

  // OCP variables
  DifferentialState x("state", jacobian_matrix_rows, 1);       // position
  Control v("control", jacobian_matrix_columns, 1);            // velocity

  // Clear state, control variable
  x.clearStaticCounters();
  v.clearStaticCounters();

  // Differential Equation
  DifferentialEquation f;

  // Differential Kinematic
  f << dot(x) == Jacobian_Matrix_ * v;

  // Optimal control problem
  OCP OCP_problem( start_time_horizon_, end_time_horizon_, discretization_intervals_);
  generateCostFunction(OCP_problem, x, v, goal_pose);

  // set hard and soft constraints
  OCP_problem.subjectTo(f);
  OCP_problem.subjectTo(control_min_constraint_ <= v <= control_max_constraint_);
  OCP_problem.subjectTo(AT_END, v == 0.0);

  // Optimal Control Algorithm


}

// setup acado algorithm options, need to set solver when calling this function
template<typename T>
void pd_frame_tracker::setAlgorithmOptions(T& OCP_solver)
{
  // intergrator
  OCP_solver.set(INTEGRATOR_TYPE, LEVENBERG_MARQUARDT);                                      // default INT_RK45: (INT_RK12, INT_RK23, INT_RK45, INT_RK78, INT_BDF)
  OCP_solver.set(INTEGRATOR_TOLERANCE, predictive_configuration::integrator_tolerance_);     // default 1e-8

  // SQP method
  OCP_solver.set(HESSIAN_APPROXIMATION, EXACT_HESSIAN);                                      // default BLOCK_BFGS_UPDATE: (CONSTANT_HESSIAN, FULL_BFGS_UPDATE, BLOCK_BFGS_UPDATE, GAUSS_NEWTON, EXACT_HESSIAN)
  OCP_solver.set(MAX_NUM_ITERATIONS, predictive_configuration::max_num_iteration_);          // default 1000
  OCP_solver.set(KKT_TOLERANCE, predictive_configuration::kkt_tolerance_);                   // default 1e-6
//  OCP_solver.set(HOTSTART_QP, true);                   // default true
//  OCP_solver.set(SPARSE_QP_SOLUTION, CONDENSING);      // CONDENSING, FULL CONDENSING, SPARSE SOLVER

  // Discretization Type
  OCP_solver.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING); // default MULTIPLE_SHOOTING: (SINGLE_SHOOTING, MULTIPLE_SHOOTING)
  OCP_solver.set(TERMINATE_AT_CONVERGENCE, true);         // default true

  // output
  OCP_solver.set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
  OCP_solver.set(PRINT_SCP_METHOD_PROFILE, false);        // default false
  OCP_solver.set(PRINT_COPYRIGHT, false);                 // default true

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
