
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
  tree.getChain( predictive_configuration::chain_base_link_, predictive_configuration::chain_tip_link_, chain);
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

  if (predictive_configuration::activate_output_)
  {
    ROS_WARN("===== TRANSFORMATION MATRIX =====");
    for (int i=0u; i < segments_; ++i)
    {
      std::cout<<"\033[36;1m" << chain.getSegment(i).getName() <<"\033[36;0m"<<std::endl;
      std::cout << Transformation_Matrix_.at(i) << std::endl;
    }
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
  // initialize local member and parameters
  FK_Matrix = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd till_joint_FK_Matrix = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd dummy_RotTrans_Matrix = Eigen::Matrix4d::Identity();

  ROS_INFO_STREAM("Forward Kinematics with joint values: ");
  std::cout << "[ " << joints_angle.transpose() << " ]" << std::endl;
  //ROS_INFO_STREAM(joints_angle.transpose());

  // compute tf transformation between root frame and base link of manipulator
  if (predictive_configuration::chain_root_link_ != predictive_configuration::chain_base_link_)
  {
    ROS_WARN("'%s' and '%s' are not same frame", chain_root_link_.c_str(), chain_base_link_.c_str());
  }

  // segments - degree_of_freedom_ gives information about fixed frame
  for (int i = 0u, revolute_joint_number = 0u; i < segments_; ++i) //(segments_- degree_of_freedom_)
  {
    // revolute joints update
    if (chain.getSegment(i).getJoint().getType() == 0)
    {
      generateTransformationMatrixFromJointValues(revolute_joint_number, joints_angle(revolute_joint_number), dummy_RotTrans_Matrix);
      till_joint_FK_Matrix = till_joint_FK_Matrix * ( Transformation_Matrix_[i] * dummy_RotTrans_Matrix);
      FK_Homogenous_Matrix_[i] = till_joint_FK_Matrix;
      ++revolute_joint_number;
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

  if (predictive_configuration::activate_output_)
  {
    ROS_WARN("===== FORWARD KINEMATICS MATRIX =======");
    std::cout << FK_Matrix << std::endl;
  }
}

// calculate diffrential velocity (linear and angular) called Jacobian Matrix
void Kinematic_calculations::calculateJacobianMatrix(const Eigen::VectorXd &joints_angle, Eigen::MatrixXd &FK_Matrix, Eigen::MatrixXd &Jacobian_Matrix)
{
  // initialize paramters and local variables
  const int jacobian_matrix_rows = 6, jacobian_matrix_columns = predictive_configuration::degree_of_freedom_;
  FK_Matrix = Eigen::Matrix4d::Identity();
  Jacobian_Matrix.resize(jacobian_matrix_rows, jacobian_matrix_columns);
  Eigen::Vector3d p( 0.0, 0.0, 0.0);
  Eigen::Vector3d z0( 0.0, 0.0, 1.0);
  Eigen::Vector3d p0( 0.0, 0.0, 0.0);

  // first calculate forward kinematic matrix
  calculateForwardKinematics(joints_angle, FK_Matrix);

  // use information about translation of end effector relative to root link
  p(0) = FK_Matrix(0,3);
  p(1) = FK_Matrix(1,3);
  p(2) = FK_Matrix(2,3);

  //compute differential velocity
  // Modelling and Control of Robot Manipulators by L. Sciavicco and B. Siciliano
  for (int i = 0u; i < segments_; ++i)
  {
    Eigen::Vector3d Jv( 0.0, 0.0, 0.0);
    Eigen::Vector3d Jo( 0.0, 0.0, 0.0);

    // for first joint
    if ( i == 0)
    {
      // revolute joints update
      if (chain.getSegment(i).getJoint().getType() == 0)
      {
        Jv = z0.cross(p);
        Jo = z0;
      }

      // fixed joints update
      if (chain.getSegment(i).getJoint().getType() == 8)
      {
        Jv = z0;
        Jo = p0;
      }
    }

    // for rest of joint execept first joint
    else
    {
      Eigen::Vector3d zi( 0.0, 0.0, 0.0);
      Eigen::Vector3d pi( 0.0, 0.0, 0.0);

      // third column of rotation matrix
      zi(0) = FK_Homogenous_Matrix_[i](0,2);
      zi(1) = FK_Homogenous_Matrix_[i](1,2);
      zi(2) = FK_Homogenous_Matrix_[i](2,2);

      // translation vector each joint relative to root link
      pi(0) = FK_Homogenous_Matrix_[i](0,3);
      pi(1) = FK_Homogenous_Matrix_[i](1,3);
      pi(2) = FK_Homogenous_Matrix_[i](2,3);

      // revolute joints update
      if (chain.getSegment(i).getJoint().getType() == 0)
      {
        Jv = zi.cross(p-pi);
        Jo = zi;
      }

      // fixed joints update
      if (chain.getSegment(i).getJoint().getType() == 8)
      {
        Jv = zi;
        Jo = p0;
      }
    }

    // root frame are not same as base frame, segments are more than degree of freedom
    // shift jacobian matrix update value from actual joint values
    if ( i >= (segments_ - degree_of_freedom_))
    {
      int point = i - (segments_ - degree_of_freedom_);
      /*// update Jacobian Matrix using computed differential velocity
      // linear velocity
      Jacobian_Matrix(0,i) = Jv(0);
      Jacobian_Matrix(1,i) = Jv(1);
      Jacobian_Matrix(2,i) = Jv(2);
      // angular velocity
      Jacobian_Matrix(3,i) = Jo(0);
      Jacobian_Matrix(4,i) = Jo(1);
      Jacobian_Matrix(5,i) = Jo(2);*/

      // update Jacobian Matrix using computed differential velocity
      // linear velocity
      Jacobian_Matrix(0,point) = Jv(0);
      Jacobian_Matrix(1,point) = Jv(1);
      Jacobian_Matrix(2,point) = Jv(2);
      // angular velocity
      Jacobian_Matrix(3,point) = Jo(0);
      Jacobian_Matrix(4,point) = Jo(1);
      Jacobian_Matrix(5,point) = Jo(2);
    }
  }

  if (predictive_configuration::activate_output_)
  {
    ROS_WARN("===== JACOBIAN_MATRIX: ======");
    std::cout << Jacobian_Matrix << std::endl;
  }
}

// calculate end effector pose using joint angles by using standard kdl pose recursive solver
void Kinematic_calculations::calculateForwardKinematicsUsingKDLSolver(const Eigen::VectorXd &joints_angle, Eigen::MatrixXd &FK_Matrix)
{
  FK_Matrix = Eigen::Matrix4d::Identity();
  KDL::Frame frame = KDL::Frame::Identity();
  KDL::JntArray joints_value; //= KDL::JntArray::resize(joints_angle.size())
  joints_value.data = joints_angle;

  ROS_INFO_STREAM("Forward Kinematics using KDL recursive solver with joint values: ");
  ROS_INFO_STREAM(joints_angle.transpose());

  // compute tf transformation between root frame and base link of manipulator
  if (predictive_configuration::chain_root_link_ != predictive_configuration::chain_base_link_)
  {
    ROS_WARN("'%s' and '%s' are not same frame", chain_root_link_.c_str(), chain_base_link_.c_str());
  }

  // kdl recursive solver to determine end effoctor position by given joint angle
  KDL::ChainFkSolverPos_recursive pose_recursive_solver(chain);
  int success_state = pose_recursive_solver.JntToCart( joints_value, frame);

  //success_state < 0 something went wrong
  if (success_state < 0)
  {
    ROS_ERROR("calculateForwardKinematicsUsingKDLSolver: Failed to compute forward kinematic using recursive solver");
  }
  else
  {
    ROS_INFO("calculateForwardKinematicsUsingKDLSolver: Successed to compute forward kinematic using recursive solver");
    transformKDLToEigenMatrix(frame, FK_Matrix);
  }
}

// calculate diffrential velocity (linear and angular) called Jacobian Matrix by using standard kdl pose recursive solver
void Kinematic_calculations::calculateJacobianMatrixUsingKDLSolver(const Eigen::VectorXd &joints_angle, Eigen::MatrixXd &FK_Matrix, Eigen::MatrixXd &Jacobian_Matrix)
{
  // initialize paramters and local variables
  const int jacobian_matrix_rows = 6, jacobian_matrix_columns = predictive_configuration::degree_of_freedom_;
  FK_Matrix = Eigen::Matrix4d::Identity();
  Jacobian_Matrix.resize(jacobian_matrix_rows, jacobian_matrix_columns);
  KDL::Frame frame = KDL::Frame::Identity();
  KDL::Jacobian KDL_Jacobian = KDL::Jacobian(predictive_configuration::degree_of_freedom_);
  KDL::JntArray joints_value;
  joints_value.data = joints_angle;

  ROS_INFO_STREAM("Forward Kinematics using KDL recursive solver with joint values: ");
  ROS_INFO_STREAM(joints_angle.transpose());

  // compute tf transformation between root frame and base link of manipulator
  if (predictive_configuration::chain_root_link_ != predictive_configuration::chain_base_link_)
  {
    ROS_WARN("'%s' and '%s' are not same frame", chain_root_link_.c_str(), chain_base_link_.c_str());
  }

  // kdl recursive solver to determine end effoctor position by given joint angle
  KDL::ChainFkSolverPos_recursive pose_recursive_solver(chain);
  int fk_success_state = pose_recursive_solver.JntToCart( joints_value, frame);

  // kdl recursive solver to determine jacobian matrix by given joint angle
  KDL::ChainJntToJacSolver jacobian_solver(chain);
  int success_state = jacobian_solver.JntToJac(joints_value, KDL_Jacobian);

  //success_state < 0 something went wrong
  if (success_state < 0 || fk_success_state < 0)
  {
    ROS_ERROR("calculateJacobianMatrixUsingKDLSolver: Failed to compute forward kinematic or Jacobian matrix using recursive solver");
  }
  else
  {
    ROS_INFO("calculateJacobianMatrixUsingKDLSolver: Successed to compute forward kinematic and Jacobian matrix using recursive solver");
    transformKDLToEigenMatrix(frame, FK_Matrix);
    Jacobian_Matrix = KDL_Jacobian.data;
  }
}

// calculate inverse of Jacobian matrix using Singular value decomposition
void Kinematic_calculations::calculateInverseJacobianbySVD(const Eigen::MatrixXd &jacobian, Eigen::MatrixXd &jacobianInv)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

  //singular values lie on diagonal of matrix, easily invert
  Eigen::VectorXd singularValues = svd.singularValues();
  Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());

  for (uint32_t i = 0; i < singularValues.rows(); ++i)
  {
    double denominator = singularValues(i) * singularValues(i);
    //singularValuesInv(i) = 1.0 / singularValues(i);
    singularValuesInv(i) = (singularValues(i) < 1e-6) ? 0.0 : singularValues(i) / denominator;
  }

  Eigen::MatrixXd result  = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
  jacobianInv = result;
}

// calculate inverse of Jacobian matrix using direct multilication method
void Kinematic_calculations::calculateInverseJacobianbyDirect(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv)
{
  Eigen::MatrixXd result;
  Eigen::MatrixXd jac_t = jacobian.transpose();
  uint32_t rows = jacobian.rows();
  uint32_t cols = jacobian.cols();

  if (cols >= rows)
  {
    result = jac_t * (jacobian * jac_t).inverse();
  }
  else
  {
    result = (jac_t * jacobian).inverse() * jac_t;
  }

  jacobianInv = result;
}

// get gripper pose using given forward kinematic matrix, orientation is in the form of rpy
void Kinematic_calculations::getGripperPoseVectorFromFK(const Eigen::MatrixXd &FK_Matrix, Eigen::VectorXd &vector)
{
  // make sure initialize and empty data
  vector = Eigen::VectorXd(6);
  KDL::Frame frame = KDL::Frame::Identity();

  // transform matrix to kdl frame
  transformEigenMatrixToKDL(FK_Matrix, frame);

  // position
  vector(0) = FK_Matrix(0,3);
  vector(1) = FK_Matrix(1,3);
  vector(2) = FK_Matrix(2,3);

  // orientation, filled data in the form of rpy
  frame.M.GetRPY(vector(3), vector(4), vector(5));
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
  ROS_WARN("arm_podest_joint: %f, %f, %f",model.getJoint("arm_podest_joint").get()->axis.x,
           model.getJoint("arm_podest_joint").get()->axis.y,
           model.getJoint("arm_podest_joint").get()->axis.z);
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
  std::cout<<"\033[0;32m" << FK_Homogenous_Matrix_[segments_-1] <<  "\033[36;0m" << std::endl;
  ROS_WARN("===================");

  ROS_INFO("------------------------- \n");
}
