
#include<lmpcc/predictive_configuration.h>

predictive_configuration::predictive_configuration()
{

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

  if (!nh.getParam ("simulation_mode", simulation_mode_) )
  {
    ROS_WARN(" Parameter 'simulation_mode' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("gazebo_simulation", gazebo_simulation_) )
  {
    ROS_WARN(" Parameter 'gazebo_simulation' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("sync_mode", sync_mode_) )
  {
    ROS_WARN(" Parameter 'sync_mode' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  // read paramter from parameter server if not set than terminate code, as this parameter is essential parameter
  if (!nh.getParam ("robot_base_link", robot_base_link_) )
  {
    ROS_WARN(" Parameter 'robot_base_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("global_path_frame", global_path_frame_) )
  {
    ROS_WARN(" Parameter 'global_path_frame' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("target_frame", target_frame_) )
  {
    ROS_WARN(" Parameter 'target_frame' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh_config.getParam ("global_path/x", ref_x_) )
  {
    ROS_WARN(" Parameter '/global_path/x not set on %s node" , ros::this_node::getName().c_str());
    return false;
  }
//
  if (!nh_config.getParam ("global_path/y", ref_y_) )
  {
    ROS_WARN(" Parameter '/global_path/y not set on %s node" , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh_config.getParam ("global_path/theta", ref_theta_) )
  {
    ROS_WARN(" Parameter '/global_path/theta not set on %s node" , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh_config.getParam ("global_path/n_points_clothoid", n_points_clothoid_) )
  {
    ROS_WARN(" Parameter '/global_path/n_points_clothoid not set on %s node" , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh_config.getParam ("global_path/n_points_spline", n_points_spline_) )
  {
    ROS_WARN(" Parameter '/global_path/n_points_spline not set on %s node" , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("publish/cmd", cmd_) )
  {
    ROS_WARN(" Parameter 'publish/cmd' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }
  if (!nh.getParam ("publish/cmd_sim", cmd_sim_) )
  {
    ROS_WARN(" Parameter 'publish/cmd_sim' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }
  if (!nh.getParam ("robot_state_topic", robot_state_topic_) )
  {
    ROS_WARN(" Parameter 'robot_state_topic' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("obs_state_topic", obs_state_topic_) )
  {
    ROS_WARN(" Parameter 'obs_state_topic' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("vref_topic", vref_topic_) )
  {
    ROS_WARN(" Parameter 'vref_topic' not set on %s node " , ros::this_node::getName().c_str());
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

  if (!nh.getParam ("waypoint_topic", waypoint_topic_) )
  {
    ROS_WARN(" Parameter 'waypoint_topic' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("reset_topic", reset_topic_) )
  {
      ROS_WARN(" Parameter 'reset_topic' not set on %s node " , ros::this_node::getName().c_str());
      return false;
  }

    ROS_INFO("acado configuration parameter");
  // check requested parameter availble on parameter server if not than set default value
  nh.param("clock_frequency", clock_frequency_, double(20.0)); // 25 hz

  nh.param("reference_velocity", reference_velocity_, double(2.0)); // 0.5 by default

  nh.param("activate_debug_output", activate_debug_output_, bool(false));  // debug
  nh.param("activate_controller_node_output", activate_controller_node_output_, bool(false));  // debug

  initialize_success_ = true;

  if (activate_debug_output_)
  {
    print_configuration_parameter();
  }

  ROS_WARN(" PREDICTIVE PARAMETER INITIALIZED!!");
  return true;
}

// clear allocated data from vector
void predictive_configuration::free_allocated_memory()
{
  vel_min_limit_.clear();
  vel_max_limit_.clear();

  ref_x_.clear();
  ref_y_.clear();
  ref_theta_.clear();

  contour_weight_factors_.clear();
  control_weight_factors_.clear();
}
