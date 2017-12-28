
#ifndef PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H
#define PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>

// eigen includes
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/LU>

// std includes
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>

// boost includes
#include <boost/shared_ptr.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// predicitve includes
#include <predictive_control/predictive_configuration.h>
#include <predictive_control/kinematic_calculations.h>
#include <predictive_control/collision_detection.h>

class predictive_control
{
  /** Managing of all classes execution of predictive control
   * - Handle self collsion avoidance
   * - Extract current position and velocity of manipulator joints
   * - Publish controlled joint velocity
   */

public:

  /**
   * @brief predictive_control: Default constructor, allocate memory
   */
  predictive_control();

  /**
   * @brief ~predictive_control: Default distructor, free memory
   */
  ~predictive_control();

  bool initialize();

  /**
   * @brief jointStateCallBack: Get current position and velocity at each joint
   * @param msg: Read data from sensor_msgs::JointState
   */
  void jointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * @brief controlSquence: Known as main control of all classes
   */
  void controlSquence(void);


  /** public data member */
  // joint state subsciber to get current joint value
  ros::Subscriber joint_state_sub_;

  // joint velocity, should be control velocity of controller
  ros::Publisher joint_velocity_pub_;

private:

  ros::NodeHandle nh;

   tf::TransformListener tf_listener_;

   // degree of freedom
   uint32_t degree_of_freedom_;

  // Clock frequency
  double update_rate_;

  // Cartesian distance/error
  double cartesian_dist_;

  // Rotation distance/error
  double rotation_dist_;

  // Timmer
  ros::Timer timer_;

  // Current and last position and velocity from joint state callback
  Eigen::VectorXd current_position_;
  Eigen::VectorXd last_position_;
  Eigen::VectorXd current_velocity_;
  Eigen::VectorXd last_velocity_;

  // Type of variable used to publish joint velocity
  std_msgs::Float64MultiArray controlled_velocity;

  // predictive configuration
  boost::shared_ptr<predictive_configuration> pd_config_;

  // kinematic calculation
  boost::shared_ptr<Kinematic_calculations> kinematic_solver_;

  // self collision detector/avoidance
  boost::shared_ptr<CollisionRobot> collision_detect_;

  void spinNode();

  /**
   * @brief runNode: Continue updating this function depend on clock frequency
   * @param event: Used for computation of duration of first and last event
   */
  void runNode(const ros::TimerEvent& event);

  /**
   * @brief publishZeroJointVelocity: published zero joint velocity is statisfied cartesian distance
   */
  void publishZeroJointVelocity();

  /**
   * @brief clearDataMember: clear vectors means free allocated memory
   */
  void clearDataMember();
};

#endif
