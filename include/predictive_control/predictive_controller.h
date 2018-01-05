
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
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

// std includes
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>

// boost includes
#include <boost/shared_ptr.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// predicitve includes
#include <predictive_control/predictive_configuration.h>
#include <predictive_control/kinematic_calculations.h>
#include <predictive_control/collision_detection.h>
#include <predictive_control/predictive_trajectory_generator.h>

class predictive_control
{
  /** Managing execution of all classes of predictive control
   * - Handle self collsion avoidance
   * - Extract current position and velocity of manipulator joints
   * - Publish controlled joint velocity
   */
    //Info: static member for transform std::vector to Eigen::vector

public:

  /**
   * @brief predictive_control: Default constructor, allocate memory
   */
  predictive_control();

  /**
   * @brief ~predictive_control: Default distructor, free memory
   */
  ~predictive_control();

  /**
   * @brief initialize: Initialize all helper class of predictive control and subscibe joint state and publish controlled joint velocity
   * @return: True with successuflly initialize all classes else false
   */
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

  /**
   * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
   * @param from: source frame from find transformation
   * @param to: target frame till find transformation
   * @param stamped_pose: Resultant poseStamed between source and target frame
   * @return: true if transform else false
   */
  /*bool getTransform(const std::string& from,
                    const std::string& to,
                    geometry_msgs::PoseStamped& stamped_pose
                    );*/

  /**
   * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
   * @param from: source frame from find transformation
   * @param to: target frame till find transformation
   * @param stamped_pose: Resultant poseStamed between source and target frame
   * @return: true if transform else false
   */
  bool getTransform(const std::string& from,
                    const std::string& to,
                    Eigen::VectorXd& stamped_pose
                    );

  /** public data member */
  // joint state subsciber to get current joint value
  ros::Subscriber joint_state_sub_;

  // controlled joint velocity, should be control velocity of controller
  ros::Publisher controlled_velocity_pub_;

private:

   tf::TransformListener tf_listener_;

   // degree of freedom
   uint32_t degree_of_freedom_;

  // Clock frequency
  double clock_frequency_;

  // Cartesian distance/error
  double cartesian_dist_;

  // Rotation distance/error
  double rotation_dist_;

  // Timmer
  ros::Timer timer_;

  // activate output of this node
  bool activate_output_;

  // current end effector and goal frame pose relative to root link
  Eigen::VectorXd current_gripper_pose_;
  Eigen::VectorXd goal_gripper_pose_;

  // goal position, goal tolerance
  Eigen::VectorXd goal_tolerance_;
  Eigen::VectorXd min_position_limit_;
  Eigen::VectorXd max_position_limit_;
  Eigen::VectorXd min_velocity_limit_;
  Eigen::VectorXd max_velocity_limit_;

  // Kinematic variables, FK_Matrix, Jacobian_Matrix
  Eigen::MatrixXd FK_Matrix_;
  Eigen::MatrixXd Jacobian_Matrix_;

  // Current and last position and velocity from joint state callback
  //Eigen::VectorXd current_position_;
  Eigen::VectorXd last_position_;
  //Eigen::VectorXd current_velocity_;
  Eigen::VectorXd last_velocity_;

  // Type of variable used to publish joint velocity
  std_msgs::Float64MultiArray controlled_velocity_;

  // predictive configuration
  boost::shared_ptr<predictive_configuration> pd_config_;

  // kinematic calculation
  boost::shared_ptr<Kinematic_calculations> kinematic_solver_;

  // self collision detector/avoidance
  boost::shared_ptr<CollisionRobot> collision_detect_;

  // predictive trajectory generator
  boost::shared_ptr<pd_frame_tracker> pd_trajectory_generator_;

  /**
   * @brief spinNode: spin node means ROS is still running
   */
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
   * @brief checkPositionLimitViolation: check position limit violate, limit containts lower and upper limit
   * @param joint_position: current joint values
   * @param position_tolerance: tolerance in joint values after reaching minimum and maximum values
   * @return true with position limit violation else false
   */
  bool checkPositionLimitViolation(const Eigen::VectorXd& joint_position,
                                   const double& position_tolerance = 0.0
                                  );

  /**
   * @brief checkPositionLimitViolation: check velocity limit violate, limit containts lower and upper limit
   * @param joint_velocity: current velocity joint values
   * @param velocity_tolerance: tolerance in joint values after reaching minimum and maximum values
   * @return true with velocity limit violation else false
   */
  bool checkVelocityLimitViolation(const std_msgs::Float64MultiArray& joint_velocity,
                                   const double& velocity_tolerance = 0.0
                                  );

  /**
   * @brief enforcePositionInLimits: enforced position to be in limit, limit containts lower and upper limit
   * @param joint_position: current joint values
   * @param position_tolerance: tolerance in joint values after reaching minimum and maximum values
   * @param enforced_joint_position: enforced joint position when it reach to minimum or maximum values
   */
  void enforcePositionInLimits(const Eigen::VectorXd& joint_position,
                               Eigen::VectorXd& enforced_joint_position,
                               const double& position_tolerance = 0.0
                              );

  /**
   * @brief enforceVelocityInLimits: enforced velocity to be in limit, limit containts lower and upper limit
   * @param joint_velocity: current joint values
   * @param velocity_tolerance: tolerance in joint values after reaching minimum and maximum values
   * @param enforced_joint_velocity: enforced joint velocity when it reach to minimum or maximum values
   */
  void enforceVelocityInLimits(const std_msgs::Float64MultiArray& joint_velocity,
                               std_msgs::Float64MultiArray& enforced_joint_velocity,
                               const double& velocity_tolerance = 0.0
                              );

  /**
   * @brief checkInfinitesimalPose: check goal tolereance statisfied, check tolerance with goal_tolerance_ data member
   * @param pose: end effoctor pose
   * @return true statisfied else false
   */
  bool checkInfinitesimalPose(const Eigen::VectorXd& pose);

  /**
   * @brief transformStdVectorToEigenVector: tranform std vector to eigen vectors as std vectos are slow to random access
   * @param vector: std vectors want to tranfrom
   * @return Eigen vectors transform from std vectos
   */
  /*template<typename T>
  static inline Eigen::VectorXd transformStdVectorToEigenVector(const std::vector<T>& vector)
  {
    // resize eigen vector
    Eigen::VectorXd eigen_vector = Eigen::VectorXd(vector.size());

    // convert std to eigen vector
    for (uint32_t i = 0; i < vector.size(); ++i)
    {
      std::cout << vector.at(i) << std::endl;
      eigen_vector(i) = vector.at(i);
   }

    return eigen_vector;
  }*/

  /**
   * @brief clearDataMember: clear vectors means free allocated memory
   */
  void clearDataMember();
};

#endif
