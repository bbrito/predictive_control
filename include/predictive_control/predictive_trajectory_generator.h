
//This file containts cost function intsert in to generated trajectory.

#ifndef PREDICTIVE_CONTROL_PREDICITVE_TRAJECTORY_GENERATOR_H
#define PREDICTIVE_CONTROL_PREDICITVE_TRAJECTORY_GENERATOR_H

// ros includes
#include <pluginlib/class_loader.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

//Eigen includes
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

// std includes
#include <iostream>
#include <string>
#include <algorithm>
#include <iomanip>	//print false or true
#include <math.h>

// boost includes
#include <boost/shared_ptr.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

//adado includes
#include <acado/acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

// predictive includes
#include <predictive_control/predictive_configuration.h>

using namespace ACADO;

class pd_frame_tracker: public predictive_configuration
{
  /** Frame tracker node, optimal control problem solver using MPC
   * - solver optimal control problem using ACADO Toolkit
   * - Add constrints as self collision
   * - Generate controlled velocity
   */

public:

  /**
   * @brief pd_frame_tracker: Default constructor, allocate memory
   */
   pd_frame_tracker();

   /**
    *@brief ~pd_frame_tracker: Default distructor, free memory
    */
  ~pd_frame_tracker();

   /**
    * @brief initialize: initialize frame tracker class and data members
    * @return true with successful initialize else false
    */
   bool initialize();

private:

   // state initialization
   DVector state_initialize_;

   // control initialization
   DVector control_initialize_;

   // constraints
   DVector control_min_constraint_;
   DVector control_max_constraint_;

   /**
   * @brief calculateQuaternionProduct: calculate quternion product used for finding quternion error
   * @param quat_1: quternion 1
   * @param quat_2: quatenion 2
   * @param quat_resultant: resultant quternion, result after multiplication of two quternion
   */
  void calculateQuaternionProduct(const geometry_msgs::Quaternion& quat_1,
                          const geometry_msgs::Quaternion& quat_2,
                          geometry_msgs::Quaternion& quat_resultant
                          );

  /**
   * @brief calculateQuaternionInverse: Calculate quternion inverse used for getting quternion error
   * @param quat: Given quternion
   * @param quat_inv: Inversed quaternion
   */
  void calculateQuaternionInverse(const geometry_msgs::Quaternion& quat,
                                  geometry_msgs::Quaternion& quat_inv
                                  );

};


#endif
