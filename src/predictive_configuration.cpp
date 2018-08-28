
#include<predictive_control/predictive_configuration.h>

predictive_configuration::predictive_configuration()
{

  set_velocity_constraints_ = true;

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
  if (!nh.getParam ("robot_base_link", robot_base_link_) )
  {
    ROS_WARN(" Parameter 'robot_base_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("tracking_frame", tracking_frame_) )
  {
    ROS_WARN(" Parameter 'tracking_frame' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

	if (!nh.getParam ("tracking_frame", target_frame_) )
	{
		ROS_WARN(" Parameter 'tracking_frame' not set on %s node " , ros::this_node::getName().c_str());
		return false;
	}

  // read and set constraints
  // read and set velocity constrints
  if (!nh_config.getParam ("constraints/velocity_constraints/min", vel_min_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/velocity_constraints/min' not set on %s node" , ros::this_node::getName().c_str());
	//constraining vx vy and w
    vel_min_limit_.resize(3, -1.0);
    set_velocity_constraints_ = false;

    for (int i = 0u; i < 3; ++i)
    {
      ROS_INFO("Velocity default min limit value %f", vel_min_limit_.at(i));
    }
  }

  if (!nh_config.getParam ("constraints/velocity_constraints/max", vel_max_limit_) )
  {
    ROS_WARN(" Parameter '/constraints/velocity_constraints/max' not set on %s node " ,  ros::this_node::getName().c_str());
    vel_max_limit_.resize(3, 1.0);
    set_velocity_constraints_ = false;

    for (int i = 0u; i < 3 ; ++i)
    {
      ROS_INFO("Velocity default min limit value %f", vel_max_limit_.at(i));
    }
  }

  // read and set other constraints...

  // read and set contour weight factors
  if (!nh_config.getParam ("acado_config/weight_factors/contour_weight_factors", contour_weight_factors_) )
  {
    ROS_WARN(" Parameter 'acado_config/weight_factors/contour_weight_factors' not set on %s node " ,
             ros::this_node::getName().c_str());
    // 3 position and 3 orientation(rpy) tolerance
    contour_weight_factors_.resize(6, 5.0);

    for (int i = 0u; i < contour_weight_factors_.size(); ++i)
    {
      ROS_INFO("Defualt contour weight factors value %f", contour_weight_factors_.at(i));
    }
  }

  ROS_WARN("Set optimal control problem dimensions");
  if (!nh.getParam("state_dim", state_dim_) )
  {
    ROS_WARN(" Parameter 'state_dim' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("control_dim", control_dim_) )
  {
    ROS_WARN(" Parameter 'control_dim' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  // read and set control weight factors
  if (!nh_config.getParam ("acado_config/weight_factors/control_weight_factors", control_weight_factors_) )
  {
    ROS_WARN(" Parameter 'acado_config/weight_factors/control_weight_factors' not set on %s node " ,
             ros::this_node::getName().c_str());
    // same as degree of freedom
    control_weight_factors_.resize(control_dim_, 1.0);

    for (int i = 0u; i < control_weight_factors_.size(); ++i)
    {
      ROS_INFO("Default control weight factors value %f", control_weight_factors_.at(i));
    }
  }



  if (!nh.getParam ("output_cmd", output_cmd) )
  {
    ROS_WARN(" Parameter 'output_cmd' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("robot_state_topic", robot_state_topic_) )
  {
    ROS_WARN(" Parameter 'robot_state_topic' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("obstacles/n_obstacles", n_obstacles_) )
  {
    ROS_WARN(" Parameter 'n_obstacles' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("obstacles/sub_ellipse_topic", sub_ellipse_topic_) )
  {
    ROS_WARN(" Parameter 'robot_state_topic' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("obstacles/n_discs", n_discs_) )
  {
    ROS_WARN(" Parameter 'n_discs' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("obstacles/ego_l", ego_l_) )
  {
    ROS_WARN(" Parameter 'ego_l' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("obstacles/ego_w", ego_w_) )
  {
    ROS_WARN(" Parameter 'ego_w' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("n_points_spline", n_points_spline_) )
  {
    ROS_WARN(" Parameter 'n_points_spline_' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  // check requested parameter availble on parameter server if not than set default value
  nh.param("clock_frequency", clock_frequency_, double(50.0)); // 50 hz
  nh.param("sampling_time", sampling_time_, double(0.025)); // 0.025 second
  nh.param("slack_weight", slack_weight_, double(1000.0)); // 1000 by default
  nh.param("reference_velocity", reference_velocity_, double(0.5)); // 0.5 by default
  nh.param("repulsive_weight", repulsive_weight_, double(0.1)); // 0.1 by default
  nh.param("activate_debug_output", activate_debug_output_, bool(false));  // debug
  nh.param("activate_controller_node_output", activate_controller_node_output_, bool(false));  // debug
  nh.param("plotting_result", plotting_result_, bool(false));  // plotting

  // acado configuration parameter
  nh_config.param("acado_config/max_num_iteration", max_num_iteration_, int(10));  // maximum number of iteration for slution of OCP
  nh_config.param("acado_config/discretization_intervals", discretization_intervals_, int(4));  // discretization_intervals for slution of OCP
  nh_config.param("acado_config/kkt_tolerance", kkt_tolerance_, double(1e-6));  // kkt condition for optimal solution
  nh_config.param("acado_config/integrator_tolerance", integrator_tolerance_, double(1e-8));  // intergrator tolerance
  nh_config.param("acado_config/start_time_horizon", start_time_horizon_, double(0.0));  // start time horizon for defining OCP problem
  nh_config.param("acado_config/end_time_horizon", end_time_horizon_, double(1.0));  // end time horizon for defining OCP problem

  initialize_success_ = true;

  if (activate_debug_output_)
  {
    print_configuration_parameter();
  }

  ROS_WARN(" PREDICTIVE PARAMETER INITIALIZED!!");
  return true;
}

// update configuration parameter
bool predictive_configuration::updateConfiguration(const predictive_configuration &new_config)
{
  activate_debug_output_ = new_config.activate_debug_output_;
  activate_controller_node_output_ = new_config.activate_controller_node_output_;
  initialize_success_ = new_config.initialize_success_;

  set_velocity_constraints_ = new_config.set_velocity_constraints_;

  plotting_result_ = new_config.plotting_result_;

  robot_base_link_ = new_config.robot_base_link_;

  tracking_frame_ = new_config.tracking_frame_;

  vel_min_limit_ = new_config.vel_min_limit_;
  vel_max_limit_ = new_config.vel_max_limit_;

  contour_weight_factors_ = new_config.contour_weight_factors_;
  control_weight_factors_ = new_config.control_weight_factors_;

  slack_weight_ = new_config.slack_weight_;
  repulsive_weight_ = new_config.repulsive_weight_;
  reference_velocity_ = new_config.reference_velocity_;

  clock_frequency_ = new_config.clock_frequency_;
  sampling_time_ = new_config.sampling_time_;

  max_num_iteration_ = new_config.max_num_iteration_;
  discretization_intervals_ = new_config.discretization_intervals_;
  kkt_tolerance_ = new_config.kkt_tolerance_;
  integrator_tolerance_ = new_config.integrator_tolerance_;
  start_time_horizon_ = new_config.start_time_horizon_;
  end_time_horizon_ = new_config.end_time_horizon_;

  if (activate_debug_output_)
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

  ROS_INFO_STREAM("Set velocity constrints: " << std::boolalpha << set_velocity_constraints_);

  ROS_INFO_STREAM("Plotting results: " << std::boolalpha << plotting_result_);
  ROS_INFO_STREAM("robot_base_link: " << robot_base_link_);

  ROS_INFO_STREAM("Clock_frequency: " << clock_frequency_);
  ROS_INFO_STREAM("Sampling_time: " << sampling_time_);
  ROS_INFO_STREAM("Max num iteration: " << max_num_iteration_);
  ROS_INFO_STREAM("Discretization intervals: " << discretization_intervals_);
  ROS_INFO_STREAM("KKT tolerance: " << kkt_tolerance_);
  ROS_INFO_STREAM("Integrator tolerance: " << integrator_tolerance_);
  ROS_INFO_STREAM("Start time horizon: " << start_time_horizon_);
  ROS_INFO_STREAM("End time horizon: " << end_time_horizon_);

  // print joints name
  std::cout << "self collision map: [";
  for_each(collision_check_obstacles_.begin(), collision_check_obstacles_.end(), [](std::string& str)
  {
    std::cout << str << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint vel min limit
  std::cout << "Joint vel min limit: [";
  for_each(vel_min_limit_.begin(), vel_min_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint vel max limit
  std::cout << "Joint vel max limit: [";
  for_each(vel_max_limit_.begin(), vel_max_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print contour weight factors
  std::cout << "Contour weight factors: [";
  for_each(contour_weight_factors_.begin(), contour_weight_factors_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print control weight factors
  std::cout << "Control weight factors: [";
  for_each(control_weight_factors_.begin(), control_weight_factors_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

}

// clear allocated data from vector
void predictive_configuration::free_allocated_memory()
{

  collision_check_obstacles_.clear();

  vel_min_limit_.clear();
  vel_max_limit_.clear();

  contour_weight_factors_.clear();
  control_weight_factors_.clear();
}
