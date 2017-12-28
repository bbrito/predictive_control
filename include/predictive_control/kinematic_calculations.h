
#ifndef PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
#define PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/static_transform_broadcaster.h>

// eigen includes
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/LU>	//inverse of matrix

// boost include
#include <boost/shared_ptr.hpp>

// kdl includes
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
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

#include<predictive_control/predictive_configuration.h>

class Kinematic_calculations: public predictive_configuration
{
  /**
    * Class to determine kinematics of robotics manipulator with following functionality
    * - Computation of forward kinematics gives information about end effector pose in cartesian space
    * - Computation of Jacobian matrix gives information about differential velocity namely linear and angular velocity
    * - Computation of inverse of Jacobian matrix gives information about inverse of differential velocity
    */

public:

  /**
   * @brief Kinematic_calculations: Default constructor, allocate memory
   */
  Kinematic_calculations();

  /**
    *@brief ~Kinematic_calculations: Default distructor, free memory
    */
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
  void calculateForwardKinematics(const Eigen::VectorXd& joints_angle,
                                  Eigen::MatrixXd& FK_Matrix
                                  );

  /**
   * @brief calculateJacobianMatrix: calculate Jacobian Matrix and Forward Kinematics (end effector pose)
   * @param joints_angle: Current joint angle
   * @param FK_Matrix: Resultant formward kinematic matrix
   * @param Jacobian_Matrix: Resultant Jacobian Matrix
   */
  void calculateJacobianMatrix(const Eigen::VectorXd& joints_angle,
                               Eigen::MatrixXd& FK_Matrix,
                               Eigen::MatrixXd& Jacobian_Matrix
                               );

  /**
   * @brief calculateForwardKinematicsUsingKDLSolver: Calculate forward kinematics start from root frame to tip link of manipulator
   *                                                  by using kdl recursive solver
   * @param joints_angle: Current joint angle
   * @param FK_Matrix: Resultant Forward Kinematic Matrix
   */
  void calculateForwardKinematicsUsingKDLSolver(const Eigen::VectorXd& joints_angle,
                                  Eigen::MatrixXd& FK_Matrix
                                  );

  /**
   * @brief calculateJacobianMatrixUsingKDLSolver: calculate Jacobian Matrix and Forward Kinematics (end effector pose)
   *                                               by using kdl recursive solver
   * @param joints_angle: Current joint angle
   * @param FK_Matrix: Resultant formward kinematic matrix
   * @param Jacobian_Matrix: Resultant Jacobian Matrix
   */
  void calculateJacobianMatrixUsingKDLSolver(const Eigen::VectorXd& joints_angle,
                               Eigen::MatrixXd& FK_Matrix,
                               Eigen::MatrixXd& Jacobian_Matrix
                               );

  /**
   * @brief calculate_inverse_jacobian_bySVD: calculate inverse of Jacobian Matrix using Singular Value Decomposition
   * @param jacobian: Jacobian Matrix
   * @param jacobianInv: Inverse of Jacobian Matrix
   */
  void calculateInverseJacobianbySVD( const Eigen::MatrixXd& jacobian,
                                         Eigen::MatrixXd& jacobianInv
                                         );

  /**
   * @brief calculate_inverse_jacobian_bySVD: calculate inverse of Jacobian Matrix using 'Direct Method'
   * @param jacobian: Jacobian Matrix
   * @param jacobianInv: Inverse of Jacobian Matrix
   */
  void calculateInverseJacobianbyDirect( const Eigen::MatrixXd& jacobian,
                                            Eigen::MatrixXd& jacobianInv
                                            );

  /**
   * @brief printDataMembers: Print important data memeber of this class, Just for Debug purpose
   */
  void printDataMembers(void);

  /**
   * @brief getTranMatrixFromRootLink: Get transformation matrix relative to root link
   * @param segment_id: From segment id to root link find transformation matrix
   * @param trans_matrix_from_root_link: Resultant transformation matrix
   */
  //void getTranMatrixFromRootLink(const unsigned int& segment_id, Eigen::MatrixXd& trans_matrix_from_root_link);

  /** public data members */
  // transformation matrix between two concecutive frame
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
  std::vector<Eigen::Vector3i> axis;

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
  void transformKDLToEigenMatrix(const KDL::Frame& frame,
                                 Eigen::MatrixXd& matrix
                                 );

  /**
   * @brief transformEigenMatrixToKDL: transform Eigen Matrix to KDL Frame
   * @param matrix transformation matrix
   * @param frame KDL::Frame which containts Rotation Matrix and Traslation vector
   */
  void transformEigenMatrixToKDL(const Eigen::MatrixXd& matrix,
                                 KDL::Frame& frame
                                 );

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
   * @brief generateTransformationMatrixFromJointValues: Generate transformation matrix used for computing forward kinematics,
   *                                                     make it easy for multiplication create transformation matrix
   * @param joint_value: Joint angle
   * @param trans_matrix: Resultant transformation matrix
   */
  void generateTransformationMatrixFromJointValues(const unsigned int& current_segment_id,
                                                   const double& joint_value,
                                                   Eigen::MatrixXd& trans_matrix
                                                   );

  /**
   * @brief clear_data_member: clear vectors means free allocated memory
   */
  void clear_data_member();

};

#endif //PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
