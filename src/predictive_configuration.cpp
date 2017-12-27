
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
  ros::NodeHandle nh_config("predictive_config");
  ros::NodeHandle nh;

  // read paramter from parameter server if not set than terminate code, as this parameter is essential parameter
  if (!nh.getParam ("robot_description", robot_description_) )
  {
    ROS_WARN(" Parameter 'robot_description' not set on %s node " , ros::this_node::getName().c_str());
    return false;
  }

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

  // check requested parameter availble on parameter server if not than set default value
  nh.param("/clock_frequency", clock_frequency_, double(50.0)); // 50 hz
  nh.param("/activate_output", activate_output_, bool(false));  // debug
  nh_config.param("self_collision/ball_radius", ball_radius_, double(0.12));  // self collision avoidance ball radius

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
  initialize_success_ = new_config.initialize_success_;
  set_position_constrints_ = new_config.set_position_constrints_;
  set_velocity_constrints_ = new_config.set_velocity_constrints_;
  set_effort_constraints_ = new_config.set_effort_constraints_;

  degree_of_freedom_ = new_config.degree_of_freedom_;
  chain_base_link_ = new_config.chain_base_link_;
  chain_tip_link_ = new_config.chain_tip_link_;
  chain_root_link_ = new_config.chain_root_link_;
  target_frame_ = new_config.target_frame_;
  tracking_frame_ = new_config.tracking_frame_;

  joints_name_ = new_config.joints_name_;
  joints_min_limit_ = new_config.joints_min_limit_;
  joints_max_limit_ = new_config.joints_max_limit_;
  joints_vel_min_limit_ = new_config.joints_vel_min_limit_;
  joints_vel_max_limit_ = new_config.joints_vel_max_limit_;
  joints_effort_min_limit_ = new_config.joints_effort_min_limit_;
  joints_effort_max_limit_ = new_config.joints_effort_max_limit_;

  clock_frequency_ = new_config.clock_frequency_;
  ball_radius_ = new_config.ball_radius_;

  if (activate_output_)
  {
    print_configuration_parameter();
  }

  return initialize_success_;
}

// print all data member of this class
void predictive_configuration::print_configuration_parameter()
{
  ROS_INFO_STREAM("Initialize_success: " << std::boolalpha << initialize_success_);
  ROS_INFO_STREAM("Set position constrints: " << std::boolalpha << set_position_constrints_);
  ROS_INFO_STREAM("Set velocity constrints: " << std::boolalpha << set_velocity_constrints_);
  ROS_INFO_STREAM("Set effort constraints: " << std::boolalpha << set_effort_constraints_);
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
}

// clear allocated data from vector
void predictive_configuration::free_allocated_memory()
{
  joints_name_.clear();
  joints_min_limit_.clear();
  joints_max_limit_.clear();
  joints_vel_min_limit_.clear();
  joints_vel_max_limit_.clear();
  joints_effort_min_limit_.clear();
  joints_effort_max_limit_.clear();
}
