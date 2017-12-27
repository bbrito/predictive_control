
#ifndef PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
#define PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/static_transform_broadcaster.h>

// eigen includes
#include <Eigen/Core>
#include <Eigen/LU>	//inverse of matrix

// boost include
#include <boost/shared_ptr.hpp>

// kdl includes
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// c++ includes
#include <iostream>
#include <map>
#include <string>
//#include <iomanip> std::setprecision(5)

// acado includes
#include <acado_optimal_control.hpp>

#include<predictive_control/predictive_configuration.h>

#define _DEBUG_  false

class Kinematic_calculations: public predictive_configuration
{
public:
  Kinematic_calculations();
  ~Kinematic_calculations();

  /**
   * @brief initialize: initialize kinematic chain and urdf model
   * @param rbt_description: urdf model of robot description
   * @return success if successful initialization otherwise false
   */
  bool initialize(const std::string rbt_description = "/robot_description");

  /**
   * @brief calculateForwardKinematics: Calculate forward kinematics start from root frame to tip link of manipulator
   * @param joints_angle: Current joint angle
   * @param FK_Matrix: Resultant Forward Kinematic Matrix
   */
  void calculateForwardKinematics(const Eigen::VectorXd& joints_angle, Eigen::MatrixXd& FK_Matrix);

  /**
   * @brief calculate_inverse_jacobian_bySVD: calculate inverse of Jacobian Matrix using Singular Value Decomposition
   * @param jacobian: Jacobian Matrix
   * @param jacobianInv: Inverse of Jacobian Matrix
   */
  void calculate_inverse_jacobian_bySVD( const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv );

  /**
   * @brief calculate_inverse_jacobian_bySVD: calculate inverse of Jacobian Matrix using 'Direct Method'
   * @param jacobian: Jacobian Matrix
   * @param jacobianInv: Inverse of Jacobian Matrix
   */
  void calculate_inverse_jacobian_byDirect( const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv );

  /** public data members */
  // transformation matrix between two concecutive frame
  //std::vector<Eigen::MatrixXd> Transformation_Matrix_;
  std::vector<Eigen::MatrixXd> Transformation_Matrix_;

  // Forward kinematic matrix from root link till current link
  std::vector<Eigen::MatrixXd> FK_Homogenous_Matrix_;

  // Forward kinematic matrix represent end effector position relative to root_link
  Eigen::MatrixXd FK_Matrix_;

private:

  unsigned int segments_;

  KDL::Chain chain;
  urdf::Model model;

  // Axis of Joints
  std::vector<Eigen::Vector3d> axis;

  /**
   * @brief initializeDataMember: initialize data member from kinematic chain
   * @param chain: kinematic chain of robotic description, usually it's full desciption of robots
   */
  void initializeDataMember(const KDL::Chain& chain);

  /**
   * @brief initializeLimitParameter: initialize joint limits using urdf model
   * @param model urdf model of robot desciption
   */
  void initializeLimitParameter(const urdf::Model& model);

  /*
  template<typedef T>
  void transformKDLTOEigen(const KDL::Frame& frame, T& matrix)
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
  }*/

  /**
   * @brief transformKDLToEigenMatrix: transform KDL Frame to Eigen Matrix
   * @param frame KDL::Frame which containts Rotation Matrix and Traslation vector
   * @param matrix transformation matrix
   */
  void transformKDLToEigenMatrix(const KDL::Frame& frame, Eigen::MatrixXd& matrix);

  /**
   * @brief transformEigenMatrixToKDL: transform Eigen Matrix to KDL Frame
   * @param matrix transformation matrix
   * @param frame KDL::Frame which containts Rotation Matrix and Traslation vector
   */
  void transformEigenMatrixToKDL(const Eigen::MatrixXd& matrix, KDL::Frame& frame);

  /**
   * @brief transformKDLToEigen: transform KDL Frame to Eigen Vector
   * @param frame KDL::JntArray containts joint values in KDL form
   * @param vector joint values in Eigen vectors
   */
//  void transformKDLToEigen(const KDL::JntArray& joints_value, Eigen::VectorXd& vector);

  /**
   * @brief transformEigenToKDL: transform Eigen Vector to KDL Frame
   * @param vector joint values in Eigen vectors
   * @param frame KDL::JntArray containts joint values in KDL form
   */
//  void transformEigenToKDL(const Eigen::VectorXd& vector, KDL::JntArray& joints_value);

  /**
   * @brief generateTransformationMatrixFromJointValues: Generate transformation matrix used for computing forward kinematics
   * @param joint_value: Joint angle
   * @param trans_matrix: Resultant transformation matrix
   */
  void generateTransformationMatrixFromJointValues(const double& joint_value, Eigen::MatrixXd& trans_matrix);

  /**
   * @brief clear_data_member: clear vectors means free allocated memory
   */
  void clear_data_member();

};

#endif //PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
