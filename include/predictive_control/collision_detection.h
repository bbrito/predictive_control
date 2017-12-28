
#ifndef PREDICTIVE_CONTROL_COLLISION_DETECTION_H_
#define PREDICTIVE_CONTROL_COLLISION_DETECTION_H_

// ros include
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/static_transform_broadcaster.h>

// geometry_msgs, visualization_msgs include
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// eigen includes
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/LU>

// c++ includes
#include <iostream>
#include <map>
#include <string>

// predictive includes
#include <predictive_control/predictive_configuration.h>

class CollisionRobot: public predictive_configuration
{
  /**
    * Class used to generate bounding volume around robot body,
    * - Development of distance cost function (logistic function) use to keep distance from robot body
    * - Visualize collision bounding ball aroung robot body
    * - Creat static frame for visulize center of intermidiate ball
    */

public:

  /**
   * @brief CollisionRobot: Default constructor, allocate memory
   */
  CollisionRobot();

  /**
    *@brief ~CollisionRobot: Default distructor, free memory
    */
  ~CollisionRobot();

  bool initialize(const std::vector<Eigen::MatrixXd>& FK_Homogenous_Matrix);

  /**
   * @brief generateCollisionVoulme: generate collision ball on given position
   * @param center: Pose of center of volume
   * @param radius: Ball radius
   * @param ball_id: Ball id should be unique for each ball
   */
  void generateCollisionVolume(const geometry_msgs::PoseStamped& center, const double& radius, const uint32_t& ball_id);

  /** public data member*/
  visualization_msgs::MarkerArray marker_array_;

private:

  /**
   * @brief clear_data_member: clear vectors means free allocated memory
   */
  void clear_data_member();

};


#endif //PREDICTIVE_CONTROL_COLLISION_DETECTION_H_
