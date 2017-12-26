
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


void Kinematic_calculations::clear_data_member()
{
  Transformation_Matrix_.clear();
  FK_Homogenous_Matrix_.clear();
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
  FK_Homogenous_Matrix_.resize(segments_);

  for (int i = 0u; i < segments_; ++i)
  {
    // convert kdl frame to eigen matrix, give tranformation matrix between two concecutive frame
    transformKDLToEigen(chain.getSegment(i).getFrameToTip(), Transformation_Matrix_[i]);
    //std::cout << Transformation_Matrix_[i] << std::endl;
  }
}

void Kinematic_calculations::initializeLimitParameter(const urdf::Model &model)
{
  // todo: create function for enforce velocity and effort.
  predictive_configuration pd_config;
  for (int i=0u; i < predictive_configuration::degree_of_freedom_; ++i)
  {
    pd_config.joints_min_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->lower;
    pd_config.joints_max_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->upper;
    model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->velocity;
    model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->effort;
  }
}

//convert KDL to Eigen matrix
void Kinematic_calculations::transformKDLToEigen(const KDL::Frame &frame, Eigen::MatrixXd &matrix)
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
void Kinematic_calculations::transformEigenToKDL(const Eigen::MatrixXd& matrix, KDL::Frame& frame)
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
