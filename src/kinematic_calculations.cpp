
#include <predictive_control/kinematic_calculations.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <geometry_msgs/TransformStamped.h>

Kinematic_calculations::Kinematic_calculations()
{
  segments_ = 7;
  clear_data_member();
}

Kinematic_calculations::~Kinematic_calculations()
{
  clear_data_member();
}

// diallocated memory
void Kinematic_calculations::clear_data_member()
{
  Transformation_Matrix_.clear();
  FK_Homogenous_Matrix_.clear();
  axis.clear();
}

// initialize chain and urdf model using robot description
bool Kinematic_calculations::initialize(const std::string rbt_description)
{
  // make sure predictice_configuration class initialized
  if (!predictive_configuration::initialize_success_)
  {
    predictive_configuration::initialize();
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromParam("/robot_description", tree))
  {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
  }

  // construct chain using tree inforamtion. Note: make sure chain root link or chain base link
  tree.getChain( predictive_configuration::chain_root_link_, predictive_configuration::chain_tip_link_, chain);
  if (chain.getNrOfJoints() == 0 || chain.getNrOfSegments() == 0)
  {
    ROS_ERROR("Failed to initialize kinematic chain");
    return false;
  }

  // construct/initialize urdf model using robot description
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file for JointLimits");
    return false;
  }

  this->initializeDataMember(chain);
  this->initializeLimitParameter(model);

  ROS_WARN("KINEMATIC CALCULATION INTIALIZED!!");
  return true;
}

// initialize data using chain
void Kinematic_calculations::initializeDataMember(const KDL::Chain &chain)
{
  segments_ = chain.getNrOfSegments();

  Transformation_Matrix_.resize(segments_, Eigen::Matrix4d::Identity()); //matrix = Eigen::Matrix4d::Identity();
  FK_Homogenous_Matrix_.resize(segments_, Eigen::Matrix4d::Identity());

  for (int i = 0u; i < segments_; ++i)
  {
    /// convert kdl frame to eigen matrix, give tranformation matrix between two concecutive frame
    transformKDLToEigenMatrix(chain.getSegment(i).getFrameToTip(), Transformation_Matrix_[i]);
    //std::cout << Transformation_Matrix_[i] << std::endl;
  }

}

// initialize limiter parameter
void Kinematic_calculations::initializeLimitParameter(const urdf::Model &model)
{
  // set axis of joint rotation
  axis.resize(predictive_configuration::degree_of_freedom_);
  for (int i = 0u; i < predictive_configuration::degree_of_freedom_; ++i)
  {
    axis[i](0) = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->axis.x;
    axis[i](1) = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->axis.y;
    axis[i](2) = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->axis.z;
  }

  // todo: create function for enforce velocity and effort.
  // update position constrints if not set
  if (!predictive_configuration::set_position_constrints_)
  {
    for (int i=0u; i < predictive_configuration::degree_of_freedom_; ++i)
    {
      predictive_configuration::joints_min_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->lower;
      predictive_configuration::joints_max_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->upper;
    }
    predictive_configuration::set_position_constrints_ = true;

    for (int i = 0u; i < predictive_configuration::joints_name_.size()
         && predictive_configuration::joints_min_limit_.size() && predictive_configuration::joints_max_limit_.size();
         ++i)
    {
      ROS_INFO("%s min velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_effort_min_limit_.at(i));
      ROS_INFO("%s max velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_max_limit_.at(i));
    }

    /*for (int i = 0u; i < predictive_configuration::joints_name_.size() && predictive_configuration::joints_max_limit_.size(); ++i)
    {
      ROS_INFO("%s max velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_max_limit_.at(i));
    }*/

  }

  // update velocity constrints if not set
  if (!predictive_configuration::set_velocity_constrints_)
  {
    for (int i=0u; i < predictive_configuration::degree_of_freedom_; ++i)
    {
      predictive_configuration::joints_vel_min_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->velocity - 0.50;
      predictive_configuration::joints_vel_max_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->velocity + 0.50;
    }
    predictive_configuration::set_velocity_constrints_ = true;

    for (int i = 0u; i < predictive_configuration::joints_name_.size()
         && predictive_configuration::joints_vel_min_limit_.size() && predictive_configuration::joints_vel_max_limit_.size();
         ++i)
    {
      ROS_INFO("%s min velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_vel_min_limit_.at(i));
      ROS_INFO("%s max velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_vel_max_limit_.at(i));
    }

    /*for (int i = 0u; i < predictive_configuration::joints_name_.size() && predictive_configuration::joints_effort_max_limit_.size(); ++i)
    {
      ROS_INFO("%s max velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_effort_max_limit_.at(i));
    }*/

  }

  // update effort constrints if not set
  if (!predictive_configuration::set_effort_constraints_)
  {
    for (int i=0u; i < predictive_configuration::degree_of_freedom_; ++i)
    {
      predictive_configuration::joints_effort_min_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->effort - 0.1;
      predictive_configuration::joints_effort_max_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->effort + 0.1;
    }
    predictive_configuration::set_effort_constraints_ = true;

    for (int i = 0u; i < predictive_configuration::joints_name_.size()
         && predictive_configuration::joints_effort_min_limit_.size() && predictive_configuration::joints_effort_max_limit_.size();
         ++i)
    {
      ROS_INFO("%s min effort limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_effort_min_limit_.at(i));
      ROS_INFO("%s max effort limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_effort_max_limit_.at(i));
    }

    /*for (int i = 0u; i < predictive_configuration::joints_name_.size() && predictive_configuration::joints_effort_max_limit_.size(); ++i)
    {
      ROS_INFO("%s max effort limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::joints_effort_max_limit_.at(i));
    }*/
  }

  else
  {
    ROS_INFO("initializeLimitParameter: All constraints already sat");
  }

}

// generate rotation matrix using joint angle and axis of rotation
/// Note: if angle value has less floating point accuracy than gives wrong answers like wrong 1.57, correct 1.57079632679.
void Kinematic_calculations::generateTransformationMatrixFromJointValues(const unsigned int& current_segment_id, const double &joint_value, Eigen::MatrixXd &trans_matrix)
{
  trans_matrix = Eigen::Matrix4d::Identity();

  // check axis of ration about x-axis
  if (axis.at(current_segment_id) == Eigen::Vector3i(1, 0, 0))
  {
    trans_matrix(1,1) = cos(joint_value);	trans_matrix(1,2) = -1*sin(joint_value);
    trans_matrix(2,1) = sin(joint_value);	trans_matrix(2,2) = cos(joint_value);
  }

  // check axis of ration about y-axis
  if (axis.at(current_segment_id) == Eigen::Vector3i(0, 1, 0))
  {
    trans_matrix(0,0) = cos(joint_value);    trans_matrix(0,2) = sin(joint_value);
    trans_matrix(2,0) = -1*sin(joint_value); trans_matrix(2,2) = cos(joint_value);
  }

  // check axis of ration about z-axis
  if (axis.at(current_segment_id) == Eigen::Vector3i(0, 0, 1))
  {
    trans_matrix(0,0) = cos(joint_value);	trans_matrix(0,1) = -1*sin(joint_value);
    trans_matrix(1,0) = sin(joint_value);	trans_matrix(1,1) = cos(joint_value);
  }

  else
  {
    ROS_ERROR("generateTransformationMatrixFromJointValues: Given rotation axis is wrong, check urdf files specifically %s ",
              chain.getSegment(current_segment_id).getName().c_str());
  }
}

// calculate end effector pose using joint angles
void Kinematic_calculations::calculateForwardKinematics(const Eigen::VectorXd& joints_angle, Eigen::MatrixXd& FK_Matrix)
{
  FK_Matrix = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd till_joint_FK_Matrix = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd dummy_RotTrans_Matrix = Eigen::Matrix4d::Identity();

  ROS_INFO_STREAM("Forward Kinematics with joint values: ");
  ROS_INFO_STREAM(joints_angle.transpose());

  if (predictive_configuration::chain_root_link_ != predictive_configuration::chain_base_link_)
  {
    ROS_WARN("'%s' and '%s' are not same from it's root", chain_root_link_.c_str(), chain_base_link_.c_str());
  }

  // segments - degree_of_freedom_ gives information about fixed frame
  for (int i = 0u, angle_id = 0u; i < segments_; ++i) //(segments_- degree_of_freedom_)
  {
    // revolute joints update
    if (chain.getSegment(i).getJoint().getType() == 0)
    {
      generateTransformationMatrixFromJointValues(angle_id, joints_angle(angle_id), dummy_RotTrans_Matrix);
      till_joint_FK_Matrix = till_joint_FK_Matrix * ( Transformation_Matrix_[i] * dummy_RotTrans_Matrix);
      FK_Homogenous_Matrix_[i] = till_joint_FK_Matrix;
      ++angle_id;
    }

    // fixed joints update
    if (chain.getSegment(i).getJoint().getType() == 8)
    {
      ROS_INFO("calculateForwardKinematics: Fixed Joint");
      till_joint_FK_Matrix = till_joint_FK_Matrix * Transformation_Matrix_[i];
      FK_Homogenous_Matrix_[i] = till_joint_FK_Matrix;
    }

    /*
    else  // presmatic joint
    {
      std::cout<<"\033[36;1m" << chain.getSegment(i).getName() <<"\033[36;0m"<<std::endl;
      till_joint_FK_Matrix = till_joint_FK_Matrix * Transformation_Matrix_[i];
      FK_Homogenous_Matrix_[i] = till_joint_FK_Matrix;
    }*/
  }

  // take last segment value that is FK Matrix
  FK_Matrix = FK_Homogenous_Matrix_[segments_-1];
}

//convert KDL to Eigen matrix
void Kinematic_calculations::transformKDLToEigenMatrix(const KDL::Frame &frame, Eigen::MatrixXd &matrix)
{
  // translation
  for (unsigned int i = 0; i < 3; ++i)
  {
    matrix(i, 3) = frame.p[i];
  }

  // rotation matrix
  for (unsigned int i = 0; i < 9; ++i)
  {
    matrix(i/3, i%3) = frame.M.data[i];
  }
}

//convert Eigen matrix to KDL::Frame
void Kinematic_calculations::transformEigenMatrixToKDL(const Eigen::MatrixXd& matrix, KDL::Frame& frame)
{
  // translation
  for (unsigned int i = 0; i < 3; ++i)
  {
    frame.p[i] = matrix(i, 3);
  }

  // rotation matrix
  for (unsigned int i = 0; i < 9; ++i)
  {
    frame.M.data[i] = matrix(i/3, i%3);
  }
}

/*
//convert KDL vectors to Eigen vector
void Kinematic_calculations::transformKDLToEigen(const KDL::JntArray& joints_value, Eigen::VectorXd& vector)
{
  ROS_INFO("transformKDLToEigen: Number of Joint Values: %f", vector.size());
  joints_value.resize(vector.size());
  for (unsigned int i = 0; i < vector.size(); ++i)
  {
    joints_value(i) = vector(i);
  }
}

//convert Eigen vector to KDL vectors
void Kinematic_calculations::transformEigenToKDL(const Eigen::VectorXd& vector, KDL::JntArray& joints_value)
{
  ROS_INFO("transformKDLToEigen: Number of Joint Values: %f", vector.size());
  vector.resize(joints_value.data.size());
  for (unsigned int i = 0; i < vector.size(); ++i)
  {
    vector(i) = joints_value.data[i];
  }
}*/

void Kinematic_calculations::printDataMembers()
{
  ROS_INFO("\n ------------------------- ");

  // print transformation matrix of each joint
  ROS_INFO("Transformation Matrix: ");
  for (int i=0u; i < segments_; ++i)
  {
    std::cout<<"\033[36;1m" << chain.getSegment(i).getName() <<"\033[36;0m"<<std::endl;
    std::cout << Transformation_Matrix_.at(i) << std::endl;
  }
  ROS_WARN("===================");

  // print joint rotation axis
  ROS_INFO("Joint rotation axis");
  for (int i=0u; i < degree_of_freedom_; ++i)
  {
    std::cout<<"\033[36;1m" << predictive_configuration::joints_name_.at(i)<< ": " << "\033[36;0m" <<
               axis.at(i).transpose() <<std::endl;
  }
  ROS_WARN("===================");

  // till joint forward kinematic matrix relative to root link
  ROS_INFO(" Joint Forward Kinematic matrix:");
  for (int i=0u; i < segments_; ++i)
  {
    std::cout<<"\033[36;1m" << chain.getSegment(i).getName()<< ": \n" << "\033[36;0m" <<
               FK_Homogenous_Matrix_[i] <<std::endl;
  }
  ROS_WARN("===================");

  // Forward kinematic matrix relative to root link
  ROS_INFO(" Forward Kinematic matrix:");
  std::cout<<"\033[36;1m" << FK_Homogenous_Matrix_[segments_-1] <<  "\033[36;0m" << std::endl;
  ROS_WARN("===================");

  ROS_INFO("------------------------- \n");
}
