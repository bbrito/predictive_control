
#ifndef PREDICTIVE_CONTROL_COLLISION_AVOIDACE_H_
#define PREDICTIVE_CONTROL_COLLISION_AVOIDACE_H_

// ros includes
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <shape_msgs/SolidPrimitive.h>

// eigen includes
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/LU>

// c++ includes
#include <iostream>
#include <map>
#include <string>
#include <fstream>

// cob_control includes
#include <cob_control_msgs/ObstacleDistance.h>
#include <cob_control_msgs/ObstacleDistances.h>
#include <cob_srvs/SetString.h>
#include <cob_srvs/SetStringRequest.h>
#include <cob_srvs/SetStringResponse.h>

// predictive includes
#include <predictive_control/predictive_configuration.h>

class CollisionAvoidance
{

public:
  CollisionAvoidance();
  ~CollisionAvoidance();

  bool initialize(const boost::shared_ptr<predictive_configuration>& pd_config_ptr);

  void obstaclesDistanceCallBack(const cob_control_msgs::ObstacleDistances::ConstPtr& msg);

  bool registerCollisionLinks();

private:

  ros::NodeHandle nh_;

  // chain base_link
  std::string chain_base_link_;

  // DEBUG
  ros::Publisher marker_pub_;

  ros::ServiceClient register_link_client_;

  // ros interfaces
  ros::Subscriber obstacle_distance_sub_;

  // predictive configuration
  boost::shared_ptr<predictive_configuration> pd_config_;

  // obstracle distances
  std::map<std::string, cob_control_msgs::ObstacleDistance> relevant_obstacle_distances_;

  void visualizeObstacleDistance(const std::map<std::string, cob_control_msgs::ObstacleDistance>& distnace_matrix);

};


#endif //PREDICTIVE_CONTROL_COLLISION_AVOIDACE_H_
