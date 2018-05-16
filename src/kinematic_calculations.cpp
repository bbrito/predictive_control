
#include <predictive_control/kinematic_calculations.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <geometry_msgs/TransformStamped.h>

Kinematic_calculations::Kinematic_calculations()
{
  segments_ = 7;
  clearDataMember();
}

Kinematic_calculations::~Kinematic_calculations()
{
  clearDataMember();
}

// diallocated memory
void Kinematic_calculations::clearDataMember()
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

  // TO BE REPLACED FIRST INITIALIZE MODEL SECOND LIMITS
  //this->initializeDataMember(chain);
  //this->initializeLimitParameter(model);

  ROS_WARN("MODEL CALCULATION INTIALIZED!!");
  return true;
}

// initialize data using chain
void Kinematic_calculations::initializeDataMember(const KDL::Chain &chain)
{
  //TO BE IMPLEMENTED
}

// initialize limiter parameter
void Kinematic_calculations::initializeLimitParameter(const urdf::Model &model)
{

  // update velocity constrints if not set
  if (!predictive_configuration::set_velocity_constraints_)
  {
    //nNEEDS TO BE UPDATED
    /*for (int i=0u; i < predictive_configuration::degree_of_freedom_; ++i)
    {
      predictive_configuration::vel_min_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->velocity - 0.50;
      predictive_configuration::vel_max_limit_[i] = model.getJoint(predictive_configuration::joints_name_.at(i)).get()->limits->velocity + 0.50;
    }
    predictive_configuration::set_velocity_constraints_ = true;

    for (int i = 0u; i < predictive_configuration::joints_name_.size()
         && predictive_configuration::vel_min_limit_.size() && predictive_configuration::vel_max_limit_.size();
         ++i)
    {
      ROS_INFO("%s min velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::vel_min_limit_.at(i));
      ROS_INFO("%s max velocity limit value %f", predictive_configuration::joints_name_.at(i).c_str(),
               predictive_configuration::vel_max_limit_.at(i));
    }
  */

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
	//TO BE IMPLEMENTED
}

// calculate end effector pose using joint angles
void Kinematic_calculations::calculateForwardKinematics(const Eigen::VectorXd& joints_angle, Eigen::MatrixXd& FK_Matrix)
{
  //TO BE IMPLEMENTED
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
*/
/*
//convert Eigen vector to KDL vectors
void Kinematic_calculations::transformEigenToKDL(const Eigen::VectorXd& vector, KDL::JntArray& joints_value)
{
  uint32_t size = vector.rows()*vector.cols();
  ROS_INFO("transformEigenToKDL: Number of Joint Values: %f", size);
  joints_value.resize(size);
  for (unsigned int i = 0; i < size; ++i)
  {
    joints_value.data[i] = vector(i);
  }

  joints_value.data = vector;
}
*/
void Kinematic_calculations::printDataMembers()
{
  ROS_INFO("\n ------------------------- ");

}
