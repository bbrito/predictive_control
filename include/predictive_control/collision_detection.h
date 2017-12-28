
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

// kdl,urdf includes
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

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

  /**
   * @brief createCollisionMatrix: Create collsion matrix using forward kinematic relative to root link
   * @param FK_Homogenous_Matrix: Forward kinematic replative to root link
   * @param Transformation_Matrix: Transformation matrix between two concecutive frame
   */
  void createCollisionMatrix(const std::vector<Eigen::MatrixXd>& FK_Homogenous_Matrix,
                             const std::vector<Eigen::MatrixXd>& Transformation_Matrix
                             );

  /**
   * @brief generateCollisionVoulme: generate collision ball on given position
   * @param center: Pose of center of volume
   * @param radius: Ball radius
   * @param ball_id: Ball id should be unique for each ball
   */
  void generateCollisionVolume(const geometry_msgs::PoseStamped& center,
                               const double& radius, const uint32_t& ball_id
                               );


  /** public data member*/
  // visulaize all volumes
  visualization_msgs::MarkerArray marker_array_;

  // collision matrix
  std::map<std::string, geometry_msgs::PoseStamped> collision_matrix_;

private:

  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  /**
   * @brief createStaticFrame: visulize intermidiate added frame, relative to root frame
   * @param stamped: Center position of ball
   * @param frame_name: Child frame name
   */
  void createStaticFrame(const geometry_msgs::PoseStamped& stamped,
                         const std::string& frame_name
                         );

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
   * @brief clear_data_member: clear vectors means free allocated memory
   */
  void clear_data_member();

};


#endif //PREDICTIVE_CONTROL_COLLISION_DETECTION_H_
