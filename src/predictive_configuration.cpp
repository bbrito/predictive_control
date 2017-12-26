
#include<predictive_control/predictive_configuration.h>

predictive_configuration::predictive_configuration()
{
  ;
}

predictive_configuration::~predictive_configuration()
{
  free_allocated_memory();
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

  if (!nh.getParam ("/joints_name", joints_name_) )
  {
    ROS_WARN(" Parameter 'joints_name' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

  degree_of_freedom_ = joints_name_.size();

  if (!nh_config.getParam ("/constraints/joint_constraints/min", joints_min_limit_) )
  {
    ROS_WARN(" Parameter 'joints_min_limit' not set on %s node" , ros::this_node::getName().c_str());
    joints_min_limit_.resize(degree_of_freedom_, -3.14);

    for (int i = 0u; i < joints_name_.size() && joints_min_limit_.size(); ++i)
    {
      ROS_INFO("%s defualt min limit %f", joints_name_.at(i).c_str(), joints_min_limit_.at(i));
    }
  }

  if (!nh_config.getParam ("/constraints/joint_constraints/max", joints_max_limit_) )
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
  nh_config.param("/ball_radius", ball_radius_, double(0.12));  // self collision avoidance ball radius
  nh.param("/active_output", active_output_, bool(false));  // debug

  initialize_success_ = true;

  if (active_output_)
  {
    print_configuration_parameter();
  }

  ROS_WARN(" PREDICTIVE PARAMETER INITIALIZED!!");
  return true;
}

// update configuration parameter
bool predictive_configuration::updateConfiguration(const predictive_configuration &new_config)
{
  active_output_ = new_config.active_output_;
  initialize_success_ = new_config.initialize_success_;

  degree_of_freedom_ = new_config.degree_of_freedom_;
  chain_base_link_ = new_config.chain_base_link_;
  chain_tip_link_ = new_config.chain_tip_link_;
  chain_root_link_ = new_config.chain_root_link_;
  target_frame_ = new_config.target_frame_;
  tracking_frame_ = new_config.tracking_frame_;

  joints_name_ = new_config.joints_name_;
  joints_min_limit_ = new_config.joints_min_limit_;
  joints_max_limit_ = new_config.joints_max_limit_;

  clock_frequency_ = new_config.clock_frequency_;
  ball_radius_ = new_config.ball_radius_;

  if (active_output_)
  {
    print_configuration_parameter();
  }

  return initialize_success_;
}

// print all data member of this class
void predictive_configuration::print_configuration_parameter()
{
  ROS_INFO_STREAM("Initialize_success: " << std::boolalpha << initialize_success_);
  ROS_INFO_STREAM("Degree_of_freedom: " << degree_of_freedom_);
  ROS_INFO_STREAM("Chain_base_link: " << chain_base_link_);
  ROS_INFO_STREAM("Chain_tip_link: " << chain_tip_link_);
  ROS_INFO_STREAM("Chain_root_link: " << chain_root_link_);
  ROS_INFO_STREAM("Target_frame: " << target_frame_);
  ROS_INFO_STREAM("Tracking_frame: " << tracking_frame_);
  ROS_INFO_STREAM("Clock_frequency: " << clock_frequency_);
  ROS_INFO_STREAM("Ball_radius: " << ball_radius_);

  // print joints name
  std::cout << "Joint names: [";
  for_each(joints_name_.begin(), joints_name_.end(), [](std::string& str)
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
}

// clear allocated data from vector
void predictive_configuration::free_allocated_memory()
{
  joints_name_.clear();
  joints_min_limit_.clear();
  joints_max_limit_.clear();
}
