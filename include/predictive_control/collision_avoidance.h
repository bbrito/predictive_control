
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
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
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
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// include movit_msgs collision msgs type
#include <moveit_msgs/CollisionObject.h>

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

  bool registerCollisionOjbect(const std::string &obstacle_name);

private:

  ros::NodeHandle nh_;

  std::string obstacle_name_;

  // static frame broadcaster
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  tf::TransformListener tf_listener_;

  tf::StampedTransform target_pose_;

  interactive_markers::InteractiveMarkerServer* ia_server_;
  visualization_msgs::InteractiveMarker int_marker_;
  visualization_msgs::InteractiveMarker int_marker_menu_;
  //interactive_markers::MenuHandler menu_handler_;

  // chain base_link
  std::string chain_base_link_;

  // DEBUG
  ros::Publisher marker_pub_;

  ros::Publisher add_obstacle_pub_;

  ros::ServiceClient register_link_client_;

  // ros interfaces
  ros::Subscriber obstacle_distance_sub_;

  // predictive configuration
  boost::shared_ptr<predictive_configuration> pd_config_;

  // obstracle distances
  std::map<std::string, cob_control_msgs::ObstacleDistance> relevant_obstacle_distances_;

  void visualizeObstacleDistance(const std::map<std::string, cob_control_msgs::ObstacleDistance>& distnace_matrix);

  void configureInteractiveMarker();

};


#endif //PREDICTIVE_CONTROL_COLLISION_AVOIDACE_H_
