
//This file containts cost function intsert in to generated trajectory.

#include <predictive_control/predictive_trajectory_generator.h>

pd_frame_tracker::pd_frame_tracker()
{
  ;
}

pd_frame_tracker::~pd_frame_tracker()
{
  ;
}

bool pd_frame_tracker::initialize()
{
  boost::shared_ptr<RealTimeAlgorithm> ocp;
  ocp.reset(new RealTimeAlgorithm());
  setAlgorithmOptions<RealTimeAlgorithm>(ocp);

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

// setup acado algorithm options, need to set solver when calling this function
template<typename T>
void pd_frame_tracker::setAlgorithmOptions(boost::shared_ptr<T> OCP_solver)
{
  // intergrator
  OCP_solver->set(INTEGRATOR_TYPE, LEVENBERG_MARQUARDT);  // default INT_RK45: (INT_RK12, INT_RK23, INT_RK45, INT_RK78, INT_BDF)
  OCP_solver->set(INTEGRATOR_TOLERANCE, 1e-8);            // default 1e-8

  // SQP method
  OCP_solver->set(HESSIAN_APPROXIMATION, EXACT_HESSIAN);  // default BLOCK_BFGS_UPDATE: (CONSTANT_HESSIAN, FULL_BFGS_UPDATE, BLOCK_BFGS_UPDATE, GAUSS_NEWTON, EXACT_HESSIAN)
  OCP_solver->set(MAX_NUM_ITERATIONS, 10);                // default 1000
  OCP_solver->set(KKT_TOLERANCE, 1e-6);                   // default 1e-6
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
