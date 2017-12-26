
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
  std::vector<Eigen::Affine3d> Transformation_Matrix_;

  // Forward kinematic matrix from root link till current link
  std::vector<Eigen::MatrixXd> FK_Homogenous_Matrix_;

  // Forward kinematic matrix represent end effector position relative to root_link
  Eigen::MatrixXd FK_Matrix_;


private:

  unsigned int segments_;

  KDL::Chain chain;
  urdf::Model model;

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

  void transformKDLTOEigen(const KDL::Frame& frame, Eigen::MatrixXd& matrix);

  void clear_data_member();

};

#endif //PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
