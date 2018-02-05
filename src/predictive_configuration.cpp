
#include<predictive_control/predictive_configuration.h>

predictive_configuration::predictive_configuration()
{
  set_position_constrints_ = true;
  set_velocity_constrints_ = true;
  set_effort_constraints_ = true;
  initialize_success_ = false;
}

predictive_configuration::~predictive_configuration()
{
  free_allocated_memory();
}

// read predicitve configuration paramter from paramter server
bool predictive_configuration::initialize() //const std::string& node_handle_name
{
  ros::NodeHandle nh_config;//("predictive_config");
  ros::NodeHandle nh;

  // read paramter from parameter server if not set than terminate code, as this parameter is essential parameter
  if (!nh.getParam ("chain_base_link", chain_base_link_) )
  {
    ROS_WARN(" Parameter 'chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("chain_tip_link", chain_tip_link_) )
  {
    ROS_WARN(" Parameter 'chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("chain_root_link", chain_root_link_) )
  {
    ROS_WARN(" Parameter 'chain_root_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("target_frame", target_frame_) )
  {
    ROS_WARN(" Parameter 'target_frame' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("tracking_frame", tracking_frame_) )
  {
    ROS_WARN(" Parameter 'tracking_frame' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("joints_name", joints_name_) )
  {
    ROS_WARN(" Parameter 'joints_name' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  // initialize degree of freedom, assume that number of joint equal to degree of freedom
  degree_of_freedom_ = joints_name_.size();

  if (!nh.getParam ("self_collision/collision_check_links", collision_check_links_) )
  {
    ROS_WARN(" Parameter 'self_collision/collision_check_links' not set on %s node please look at ../self_collision.yaml" , ros::this_node::getName().c_str());
    collision_check_links_.resize(degree_of_freedom_, std::string(""));
  }

  // read and set joint constrints
  if (!nh_config.getParam ("constraints/position_constraints/min", joints_min_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/position_constraints/min' not set on %s node" , ros::this_node::getName().c_str());
    joints_min_limit_.resize(degree_of_freedom_, -3.14);
    set_position_constrints_ = false;

    for (int i = 0u; i < joints_name_.size() && joints_min_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit value %f", joints_name_.at(i).c_str(), joints_min_limit_.at(i));
    }
  }

  if (!nh_config.getParam ("constraints/position_constraints/max", joints_max_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/position_constraints/max' not set on %s node " ,  ros::this_node::getName().c_str());
    joints_max_limit_.resize(degree_of_freedom_, 3.14);
    set_position_constrints_ = false;

    for (int i = 0u; i < joints_name_.size() && joints_max_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit value %f", joints_name_.at(i).c_str(), joints_max_limit_.at(i));
    }
  }

  // read and set joint velocity constrints
  if (!nh_config.getParam ("constraints/velocity_constraints/min", joints_vel_min_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/velocity_constraints/min' not set on %s node" , ros::this_node::getName().c_str());
    joints_vel_min_limit_.resize(degree_of_freedom_, -1.0);
    set_velocity_constrints_ = false;

    for (int i = 0u; i < joints_name_.size() && joints_vel_min_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit value %f", joints_name_.at(i).c_str(), joints_vel_min_limit_.at(i));
    }
  }

  if (!nh_config.getParam ("constraints/velocity_constraints/max", joints_vel_max_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/velocity_constraints/max' not set on %s node " ,  ros::this_node::getName().c_str());
    joints_vel_max_limit_.resize(degree_of_freedom_, 1.0);
    set_velocity_constrints_ = false;

    for (int i = 0u; i < joints_name_.size() && joints_vel_max_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit value %f", joints_name_.at(i).c_str(), joints_vel_max_limit_.at(i));
    }
  }

  // read and set joint effort/acceleration constrints
  if (!nh_config.getParam ("constraints/effort_constraints/min", joints_effort_min_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/effort_constraints/min' not set on %s node" , ros::this_node::getName().c_str());
    joints_effort_min_limit_.resize(degree_of_freedom_, -0.0);
    set_effort_constraints_ = false;

    for (int i = 0u; i < joints_name_.size() && joints_effort_min_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit value %f", joints_name_.at(i).c_str(), joints_effort_min_limit_.at(i));
    }
  }

  if (!nh_config.getParam ("constraints/effort_constraints/max", joints_effort_max_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/effort_constraints/max' not set on %s node " ,  ros::this_node::getName().c_str());
    joints_effort_max_limit_.resize(degree_of_freedom_, 1.0);
    set_effort_constraints_ = false;

    for (int i = 0u; i < joints_name_.size() && joints_effort_max_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit value %f", joints_name_.at(i).c_str(), joints_effort_max_limit_.at(i));
    }
  }

  // read and set goal tolerance/threshold
  if (!nh_config.getParam ("tolerance/goal_tolerance", goal_pose_tolerance_) )
  {
    ROS_WARN(" Parameter 'tolerance/goal_tolerance' not set on %s node " ,  ros::this_node::getName().c_str());
    // 3 position and 3 orientation(rpy) tolerance
    goal_pose_tolerance_.resize(6, 0.05);

    for (int i = 0u; i < goal_pose_tolerance_.size(); ++i)
    {
      ROS_INFO("Defualt goal tolerance value %f", goal_pose_tolerance_.at(i));
    }
  }

  // read and set lsq state weight factors
  if (!nh_config.getParam ("acado_config/weight_factors/lsq_state_weight_factors", lsq_state_weight_factors_) )
  {
    ROS_WARN(" Parameter 'acado_config/weight_factors/lsq_state_weight_factors' not set on %s node " ,
             ros::this_node::getName().c_str());
    // 3 position and 3 orientation(rpy) tolerance
    lsq_state_weight_factors_.resize(6, 5.0);

    for (int i = 0u; i < lsq_state_weight_factors_.size(); ++i)
    {
      ROS_INFO("Defualt lsq state weight factors value %f", lsq_state_weight_factors_.at(i));
    }
  }

  // read and set lsq control weight factors
  if (!nh_config.getParam ("acado_config/weight_factors/lsq_control_weight_factors", lsq_control_weight_factors_) )
  {
    ROS_WARN(" Parameter 'acado_config/weight_factors/lsq_control_weight_factors' not set on %s node " ,
             ros::this_node::getName().c_str());
    // same as degree of freedom
    lsq_control_weight_factors_.resize(degree_of_freedom_, 1.0);

    for (int i = 0u; i < lsq_control_weight_factors_.size(); ++i)
    {
      ROS_INFO("Defualt lsq control weight factors value %f", lsq_control_weight_factors_.at(i));
    }
  }

  // check requested parameter availble on parameter server if not than set default value
  nh.param("robot_description", robot_description_, std::string("robot_description")); // robot description
  nh.param("clock_frequency", clock_frequency_, double(50.0)); // 50 hz
  nh.param("sampling_time", sampling_time_, double(0.025)); // 0.025 second
  nh.param("activate_output", activate_output_, bool(false));  // debug
  nh.param("activate_controller_node_output", activate_controller_node_output_, bool(false));  // debug
  nh.param("plotting_result", plotting_result_, bool(false));  // plotting

  // self collision avoidance parameter
  nh_config.param("self_collision/ball_radius", ball_radius_, double(0.12));  // self collision avoidance ball radius
  nh_config.param("self_collision/minimum_collision_distance", minimum_collision_distance_, double(0.12));  // self collision avoidance minimum distance
  nh_config.param("self_collision/collision_weight_factor", collision_weight_factor_, double(0.01));  // self collision avoidance weight factor

  // acado configuration parameter
  nh_config.param("acado_config/max_num_iteration", max_num_iteration_, int(10));  // maximum number of iteration for slution of OCP
  nh_config.param("acado_config/discretization_intervals", discretization_intervals_, int(4));  // discretization_intervals for slution of OCP
  nh_config.param("acado_config/kkt_tolerance", kkt_tolerance_, double(1e-6));  // kkt condition for optimal solution
  nh_config.param("acado_config/integrator_tolerance", integrator_tolerance_, double(1e-8));  // intergrator tolerance
  nh_config.param("acado_config/start_time_horizon", start_time_horizon_, double(0.0));  // start time horizon for defining OCP problem
  nh_config.param("acado_config/end_time_horizon", end_time_horizon_, double(1.0));  // end time horizon for defining OCP problem
  nh_config.param("acado_config/use_LSQ_term", use_LSQ_term_, bool(false));  // use for minimize objective function
  nh_config.param("acado_config/use_lagrange_term", use_lagrange_term_, bool(false));  // use for minimize objective function
  nh_config.param("acado_config/use_mayer_term", use_mayer_term_, bool(true));  // use for minimize objective function

  initialize_success_ = true;

  if (activate_output_)
  {
    print_configuration_parameter();
  }

  ROS_WARN(" PREDICTIVE PARAMETER INITIALIZED!!");
  return true;
}

// update configuration parameter
bool predictive_configuration::updateConfiguration(const predictive_configuration &new_config)
{
  activate_output_ = new_config.activate_output_;
  activate_controller_node_output_ = new_config.activate_controller_node_output_;
  initialize_success_ = new_config.initialize_success_;
  set_position_constrints_ = new_config.set_position_constrints_;
  set_velocity_constrints_ = new_config.set_velocity_constrints_;
  set_effort_constraints_ = new_config.set_effort_constraints_;
  plotting_result_ = new_config.plotting_result_;

  degree_of_freedom_ = new_config.degree_of_freedom_;
  chain_base_link_ = new_config.chain_base_link_;
  chain_tip_link_ = new_config.chain_tip_link_;
  chain_root_link_ = new_config.chain_root_link_;
  target_frame_ = new_config.target_frame_;
  tracking_frame_ = new_config.tracking_frame_;

  joints_name_ = new_config.joints_name_;
  collision_check_links_ = new_config.collision_check_links_;
  joints_min_limit_ = new_config.joints_min_limit_;
  joints_max_limit_ = new_config.joints_max_limit_;
  joints_vel_min_limit_ = new_config.joints_vel_min_limit_;
  joints_vel_max_limit_ = new_config.joints_vel_max_limit_;
  joints_effort_min_limit_ = new_config.joints_effort_min_limit_;
  joints_effort_max_limit_ = new_config.joints_effort_max_limit_;
  goal_pose_tolerance_ = new_config.goal_pose_tolerance_;
  lsq_state_weight_factors_ = new_config.lsq_state_weight_factors_;
  lsq_control_weight_factors_ = new_config.lsq_control_weight_factors_;

  clock_frequency_ = new_config.clock_frequency_;
  sampling_time_ = new_config.sampling_time_;
  ball_radius_ = new_config.ball_radius_;
  minimum_collision_distance_ = new_config.minimum_collision_distance_;
  collision_weight_factor_ = new_config.collision_weight_factor_;

  use_lagrange_term_ = new_config.use_lagrange_term_;
  use_LSQ_term_ = new_config.use_LSQ_term_;
  use_mayer_term_ = new_config.use_mayer_term_;
  max_num_iteration_ = new_config.max_num_iteration_;
  discretization_intervals_ = new_config.discretization_intervals_;
  kkt_tolerance_ = new_config.kkt_tolerance_;
  integrator_tolerance_ = new_config.integrator_tolerance_;
  start_time_horizon_ = new_config.start_time_horizon_;
  end_time_horizon_ = new_config.end_time_horizon_;

  if (activate_output_)
  {
    print_configuration_parameter();
  }

  return initialize_success_;
}

// print all data member of this class
void predictive_configuration::print_configuration_parameter()
{
  ROS_INFO_STREAM("Activate_controller_node_output: " << std::boolalpha << activate_controller_node_output_);
  ROS_INFO_STREAM("Initialize_success: " << std::boolalpha << initialize_success_);
  ROS_INFO_STREAM("Set position constrints: " << std::boolalpha << set_position_constrints_);
  ROS_INFO_STREAM("Set velocity constrints: " << std::boolalpha << set_velocity_constrints_);
  ROS_INFO_STREAM("Set effort constraints: " << std::boolalpha << set_effort_constraints_);
  ROS_INFO_STREAM("Plotting results: " << std::boolalpha << plotting_result_);
  ROS_INFO_STREAM("Degree_of_freedom: " << degree_of_freedom_);
  ROS_INFO_STREAM("Chain_base_link: " << chain_base_link_);
  ROS_INFO_STREAM("Chain_tip_link: " << chain_tip_link_);
  ROS_INFO_STREAM("Chain_root_link: " << chain_root_link_);
  ROS_INFO_STREAM("Target_frame: " << target_frame_);
  ROS_INFO_STREAM("Tracking_frame: " << tracking_frame_);
  ROS_INFO_STREAM("Clock_frequency: " << clock_frequency_);
  ROS_INFO_STREAM("Sampling_time: " << sampling_time_);
  ROS_INFO_STREAM("Ball_radius: " << ball_radius_);
  ROS_INFO_STREAM("Minimum collision distance: " << minimum_collision_distance_);
  ROS_INFO_STREAM("Collision weight factor: " << collision_weight_factor_);
  ROS_INFO_STREAM("Use lagrange term: " << std::boolalpha << use_lagrange_term_);
  ROS_INFO_STREAM("Use LSQ term: " << std::boolalpha << use_LSQ_term_);
  ROS_INFO_STREAM("Use mayer term: " << std::boolalpha << use_mayer_term_);
  ROS_INFO_STREAM("Max num iteration: " << max_num_iteration_);
  ROS_INFO_STREAM("Discretization intervals: " << discretization_intervals_);
  ROS_INFO_STREAM("KKT tolerance: " << kkt_tolerance_);
  ROS_INFO_STREAM("Integrator tolerance: " << integrator_tolerance_);
  ROS_INFO_STREAM("Start time horizon: " << start_time_horizon_);
  ROS_INFO_STREAM("End time horizon: " << end_time_horizon_);


  // print joints name
  std::cout << "Joint names: [";
  for_each(joints_name_.begin(), joints_name_.end(), [](std::string& str)
  {
    std::cout << str << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joints name
  std::cout << "self collision map: [";
  for_each(collision_check_links_.begin(), collision_check_links_.end(), [](std::string& str)
  {
    std::cout << str << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint min limits
  std::cout << "Joint min limit: [";
  for_each(joints_min_limit_.begin(), joints_min_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint max limit
  std::cout << "Joint max limit: [";
  for_each(joints_max_limit_.begin(), joints_max_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint vel min limit
  std::cout << "Joint vel min limit: [";
  for_each(joints_vel_min_limit_.begin(), joints_vel_min_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint vel max limit
  std::cout << "Joint vel max limit: [";
  for_each(joints_vel_max_limit_.begin(), joints_vel_max_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint effort min limit
  std::cout << "Joint effort min limit: [";
  for_each(joints_effort_min_limit_.begin(), joints_effort_min_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint effort max limit
  std::cout << "Joint effort max limit: [";
  for_each(joints_effort_max_limit_.begin(), joints_effort_max_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print goal pose tolerance/threshold
  std::cout << "Goal pose tolerance: [";
  for_each(goal_pose_tolerance_.begin(), goal_pose_tolerance_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print lsq state weight factors
  std::cout << "LSQ state weight factors: [";
  for_each(lsq_state_weight_factors_.begin(), lsq_state_weight_factors_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print lsq control weight factors
  std::cout << "LSQ control weight factors: [";
  for_each(lsq_control_weight_factors_.begin(), lsq_control_weight_factors_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

}

// clear allocated data from vector
void predictive_configuration::free_allocated_memory()
{
  joints_name_.clear();
  collision_check_links_.clear();
  joints_min_limit_.clear();
  joints_max_limit_.clear();
  joints_vel_min_limit_.clear();
  joints_vel_max_limit_.clear();
  joints_effort_min_limit_.clear();
  joints_effort_max_limit_.clear();
  goal_pose_tolerance_.clear();
  lsq_state_weight_factors_.clear();
  lsq_control_weight_factors_.clear();
}
