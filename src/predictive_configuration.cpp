
#include<predictive_control/predictive_configuration.h>

predictive_configuration::predictive_configuration()
{
  ;
}

predictive_configuration::~predictive_configuration()
{
  ;
}

// read predicitve configuration paramter from paramter server
bool predictive_configuration::initialize(const std::string& node_handle_name)
{
  ros::NodeHandle nh_config(node_handle_name);
  ros::NodeHandle nh;

  // read paramter from parameter server if not set than terminate code, as this parameter is essential parameter
  if (!nh.getParam ("/chain_base_link", chain_base_link_) )
  {
    ROS_WARN(" Parameter 'chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("/chain_tip_link", chain_tip_link_) )
  {
    ROS_WARN(" Parameter 'chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("/chain_root_link", chain_root_link_) )
  {
    ROS_WARN(" Parameter 'chain_root_link' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("/target_frame", target_frame_) )
  {
    ROS_WARN(" Parameter 'target_frame' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("/tracking_frame", tracking_frame_) )
  {
    ROS_WARN(" Parameter 'tracking_frame' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh.getParam ("/joints_name:", joints_name_) )
  {
    ROS_WARN(" Parameter 'joints_name' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  if (!nh_config.getParam ("/constraints/joint_constraints/min:", joints_min_limit_) )
  {
    ROS_WARN(" Parameter 'joints_min_limit' not set on %s node" , ros::this_node::getName().c_str());
    joints_min_limit_.resize(degree_of_freedom_, -3.14);

    for (int i = 0u; i < joints_name_.size() && joints_min_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit %f", joints_name_.at(i).c_str(), joints_min_limit_.at(i));
    }
  }

  if (!nh_config.getParam ("/constraints/joint_constraints/max:", joints_max_limit_) )
  {
    ROS_WARN(" Parameter 'joints_max_limit' not set on %s node " , ros::this_node::getName().c_str());
    joints_max_limit_.resize(degree_of_freedom_, 3.14);

    for (int i = 0u; i < joints_name_.size() && joints_max_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit %f", joints_name_.at(i).c_str(), joints_max_limit_.at(i));
    }
  }

  // check requested parameter availble on parameter server if not than set default value
  nh.param("/clock_frequency", clock_frequency_, double(50.0)); // 50 hz
  nh.param("/active_output", active_output_, bool(false));  // debug

  degree_of_freedom_ = joints_name_.size();
  initialize_success_ = true;

  ROS_WARN(" PREDICTIVE PARAMETER INITIALIZED!!");
  return true;
}

bool predictive_configuration::updateConfiguration(const predictive_configuration &new_config)
{

  return true;
}
