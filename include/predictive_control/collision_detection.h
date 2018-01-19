
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
#include <shape_msgs/SolidPrimitive.h>

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
#include <kdl/chain.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>

// predictive includes
#include <predictive_control/predictive_configuration.h>


enum  {
  UNKNOWN,
  REVOLUTE,
  CONTINUOUS,
  PRISMATIC,
  FLOATING,
  PLANAR,
  FIXED
};


class SelfCollision
{
  /**
    * Class used to generate bounding volume around robot body,
    * - Development of distance cost function (logistic function) use to keep distance from robot body
    * - Visualize collision bounding ball aroung robot body
    * - Creat static frame for visulize center of intermidiate ball
    * Info: static member for computation of getEuclideanDistance
    */

public:

  /**
   * @brief SelfCollision: Default constructor, allocate memory
   */
  SelfCollision();

  /**
   * @brief ~SelfCollision: Default distractor, remove memory
   */
  ~SelfCollision();


  /**
   * @brief initialize: Initialize self collision class
   * @param pd_config_param: predictive congiguration parameters
   * @return true with success, else false
   */
  bool initialize(const predictive_configuration& pd_config_param);

  /**
   * @brief visualizeCollisionVolume: visulize collision ball on given position
   * @param center: Pose of center of volume
   * @param radius: Ball radius
   * @param header_frame_id: header frame id for bounding ball
   * @param ball_id: Ball id should be unique for each ball
   */
  void visualizeCollisionVolume(const Eigen::VectorXd& center,
                                const Eigen::VectorXd& radius,
                                const std::string& header_frame_id,
                                const uint32_t &ball_id
                                );

  /**
   * @brief visualizeCollisionVolume: visulize collision ball on given position
   * @param center: Pose of center of volume
   * @param radius: Ball radius
   * @param ball_id: Ball id should be unique for each ball
   */
  void visualizeCollisionVolume(const geometry_msgs::PoseStamped &center,
                                const Eigen::Vector3d &radius,
                                const uint32_t &ball_id
                                );


  /**
   * @brief updateCollisionVolume: Update collsion matrix using joint angles
   * @param joints_angle: joint angle used to compute forward kinematics
   */
  void updateCollisionVolume(const Eigen::VectorXd& joints_angle);

  /**
   * @brief generateCollisionVolume: Create collsion matrix using forward kinematic relative to root link
   * @param FK_Homogenous_Matrix: Forward kinematic replative to root link
   * @param Transformation_Matrix: Transformation matrix between two concecutive frame
   */
  void generateCollisionVolume(const std::vector<Eigen::MatrixXd> &FK_Homogenous_Matrix,
                              const std::vector<Eigen::MatrixXd> &Transformation_Matrix);


  /**
   * @brief tranformEiegnMatrixToEigenVector: convert Eigen Matrix to Eigen Vector
   * @param matrix: eigen matrix from convert
   * @param vector: eigen vector to store converted vector data
   */
  static inline
  void tranformEiegnMatrixToEigenVector(const Eigen::MatrixXd& matrix, Eigen::VectorXd& vector)
  {
    vector = Eigen::VectorXd(7);

    // create rotation matrix
    tf::Matrix3x3 mat(matrix(0,0), matrix(0,1), matrix(0,2),
                      matrix(1,0), matrix(1,1), matrix(1,2),
                      matrix(2,0), matrix(2,1), matrix(2,2));

    // get quaterenion from rotation matrix
    tf::Quaternion quat;
    mat.getRotation(quat);

    // fill eigen vector
    vector(0) = matrix(0,3);
    vector(1) = matrix(1,3);
    vector(2) = matrix(2,3);
    vector(3) = quat.w();
    vector(4) = quat.x();
    vector(5) = quat.y();
    vector(6) = quat.z();
  }


  // Public Data Member of class
  visualization_msgs::MarkerArray marker_array_;

private:

  // marker publisher
  ros::Publisher marker_pub_;

  //std::map< std::string, boost::shared_ptr<urdf::Joint> > joints_;

  // collision matrix
  // transformation matrix between two concecutive frame
  std::vector<Eigen::MatrixXd> Transformation_Matrix_;

  // Forward kinematic matrix from root link till current link
  std::vector<Eigen::MatrixXd> FK_Homogenous_Matrix_;


  //std::map<std::string, std::string> types;
  std::vector<Eigen::MatrixXd> collision_matrix_;
  Eigen::VectorXi types_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> model_joint_names_;

  // Predictive configuration
  predictive_configuration pd_config_;

  // Kinematic configuration
  urdf::Model model_;
  KDL::Chain chain;

  // number of segments
  int segments_;

  // degree of freedom, calculate from chain segment
  int degree_of_freedom_;

  // Axis of Joints
  std::vector<Eigen::Vector3i> axis_;

  std::vector<double> ball_major_axis_;
  //Eigen::VectorXd ball_major_axis_;

  std::vector<Eigen::VectorXd> distance_vector_;

  // static frame broadcaster
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;


  /**
   * @brief initializeDataMember: initialize data member from kinematic chain and model
   * @param model: robot model extracted form robot desription
   * @param chain: kinematic chain of robotic description, usually it's full desciption of robots
   */
  void initializeDataMember(const std::map< std::string, boost::shared_ptr<urdf::Joint> >& joints);

  /**
   * @brief calculateForwardKinematics: Calculate forward kinematics start from root frame to tip link of manipulator
   * @param joints_angle: Current joint angle
   * @param FK_Matrix: Resultant Forward Kinematic Matrix
   */
  void calculateForwardKinematics(const Eigen::VectorXd& joints_angle
                                  );

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
   * @brief createStaticFrame: visulize intermidiate added frame, relative to root frame
   * @param stamped: Center position of ball
   * @param frame_name: Child frame name
   */
  void createStaticFrame(const Eigen::VectorXd& vector,
                         const std::string& frame_name
                         );


  /**
   * @brief transformKDLToEigenMatrix: transform KDL Frame to Eigen Matrix
   * @param frame KDL::Frame which containts Rotation Matrix and Traslation vector
   * @param matrix transformation matrix
   */
  void transformURDFToEigenMatrix(const urdf::Pose &pose,
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

  /** Templated paramter function
   * @brief computeIndexFromVector: compute location of string into vector
   * @param vector: vector from searching string
   * @param search_for: key use to search from vector
   * @param index: location of index into vector
   */
  template<typename PARAMETER_TYPE, typename RETURN_TYPE>
  void computeIndexFromVector(const std::vector<PARAMETER_TYPE>& vector, const PARAMETER_TYPE& search_for, RETURN_TYPE& index)
  {
    auto it = std::find(vector.begin(), vector.end(), search_for);
    index = static_cast<RETURN_TYPE>(std::distance(vector.begin(), it));
  }

  /** Templated paramter function and also defualt initialized
   * @brief computeIndexFromVector: compute location of string into vector
   * @param vector: vector from searching string
   * @param search_for: key use to search from vector
   * @return index: location of index into vector
   */
  template<typename PARAMETER_TYPE=std::string, typename RETURN_TYPE=int>
  RETURN_TYPE computeIndexFromVector(const std::vector<PARAMETER_TYPE>& vector, const PARAMETER_TYPE& search_for)
  {
    auto it = std::find(vector.begin(), vector.end(), search_for);
    return static_cast<RETURN_TYPE>(std::distance(vector.begin(), it));
  }

  /**
   * @brief clearDataMember: clear vectors means free allocated memory
   */
  void clearDataMember();
};


//------------------------------------------------------------------------------------------------------------------------------

class CollisionRobot: public predictive_configuration
{
  /**
    * Class used to generate bounding volume around robot body,
    * - Development of distance cost function (logistic function) use to keep distance from robot body
    * - Visualize collision bounding ball aroung robot body
    * - Creat static frame for visulize center of intermidiate ball
    * Info: static member for computation of getEuclideanDistance
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
   * @brief initializeCollisionRobot: Initialize Collision Robot class
   * @return true with success, else false
   */
  bool initializeCollisionRobot();

  /**
   * @brief updateCollisionVolume: Update collsion matrix using forward kinematic relative to root link
   * @param FK_Homogenous_Matrix: Forward kinematic replative to root link
   * @param Transformation_Matrix: Transformation matrix between two concecutive frame
   */
  void updateCollisionVolume(const std::vector<Eigen::MatrixXd>& FK_Homogenous_Matrix,
                             const std::vector<Eigen::MatrixXd>& Transformation_Matrix
                             );

  /**
   * @brief generateCollisionVolume: Create collsion matrix using forward kinematic relative to root link
   * @param FK_Homogenous_Matrix: Forward kinematic replative to root link
   * @param Transformation_Matrix: Transformation matrix between two concecutive frame
   */
  void generateCollisionVolume(const std::vector<Eigen::MatrixXd>& FK_Homogenous_Matrix,
                             const std::vector<Eigen::MatrixXd>& Transformation_Matrix
                             );


  /**
   * @brief visualizeCollisionVolume: visulize collision ball on given position
   * @param center: Pose of center of volume
   * @param radius: Ball radius
   * @param ball_id: Ball id should be unique for each ball
   */
  void visualizeCollisionVolume(const geometry_msgs::PoseStamped& center,
                               const double& radius, const uint32_t& ball_id
                               );


  /**
   * @brief computeCollisionCost: Computation collision distance cost,
   *                              Store Distance vectors represent distance from center of frame to other frame
   * @param collision_matrix: Collision matrix has information about distance between frames relative to root frame
   * @param collision_min_distance: Minimum collision distance, below that should not go
   * @param weight_factor: convergence rate
   */
  void computeCollisionCost(const std::map<std::string, geometry_msgs::PoseStamped> collision_matrix,
                                       const double& collision_min_distance,
                                       const double& weight_factor
                                       );


  /**
   * @brief getEuclideanDistance: compute 2D distance called EuclideanDistance
   * @param pose_a: Pose of one point
   * @param pose_b: Pose of second point
   * @return distance between pose_a and pose_b
   */
  static inline double getEuclideanDistance(const geometry_msgs::Pose& pose_a,
                                     const geometry_msgs::Pose& pose_b
                                     )
  {
    ROS_DEBUG_STREAM("point_a:" << pose_a.position);
    ROS_DEBUG_STREAM("point_b:" << pose_b.position);

    double distance = ( sqrt( (pose_a.position.x - pose_b.position.x) * (pose_a.position.x - pose_b.position.x) +
                  (pose_a.position.y - pose_b.position.y) * (pose_a.position.y - pose_b.position.y) +
                  (pose_a.position.z - pose_b.position.z) * (pose_a.position.z - pose_b.position.z)
            ));

    ROS_DEBUG_STREAM("getEuclideanDistance: ...  "<< distance);
    return distance;
  }

  /** public data member*/
  // visulaize all volumes
  visualization_msgs::MarkerArray marker_array_;

  // collision matrix
  std::map<std::string, geometry_msgs::PoseStamped> collision_matrix_;

  // collision cost vector
  Eigen::VectorXd collision_cost_vector_;

private:
  // marker publisher
  ros::Publisher marker_pub_;

  // static frame broadcaster
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
   * @brief clearDataMember: clear vectors means free allocated memory
   */
  void clearDataMember();

};


//------------------------------------------------------------------------------------------------------------------------------
//------------------------------------- Static Collision Object Avoidance -----------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------
class StaticCollision: public predictive_configuration
{
  /**
    * Class used to generate bounding volume around static object,
    * - Development of distance cost function (logistic function) use to keep distance from robot body
    * - Visualize collision bounding ball around StaticCollision
    * - Creat static frame for visulize center of intermidiate ball
    */

public:

  /**
   * @brief CollisionRobot: Default constructor, allocate memory
   */
  StaticCollision();

  /**
    *@brief ~CollisionRobot: Default distructor, free memory
    */
  ~StaticCollision();

  /**
   * @brief initializeStaticCollisionObject: Initialize Collision Robot class
   * @return true with success, else false
   */
  bool initializeStaticCollisionObject();

  /**
   * @brief updateStaticCollisionVolume: update static collision volume function at every runtime
   * @param robot_critical_points
   */
  void updateStaticCollisionVolume(const std::map<std::string, geometry_msgs::PoseStamped>& robot_critical_points);

  /**
   * @brief generateStaticCollisionVolume: generate static collision oject, fill collision_matrix
   */
  void generateStaticCollisionVolume();


  /**
   * @brief visualizeStaticCollisionVoulme: visulize static collision object
   * @param stamped: collsion object represented by size, pose, primitive_type, operation
   */
  void visualizeStaticCollisionVoulme(const geometry_msgs::PoseStamped& stamped);

  /**
   * @brief computeStaticCollisionCost: Computation collision distance cost,
   *                              Store Distance vectors represent distance from center of frame to other frame
   * @param collision_matrix: Collision matrix has information about distance between frames relative to root frame
   * @param collision_min_distance: Minimum collision distance, below that should not go
   * @param weight_factor: convergence rate
   */
  void computeStaticCollisionCost(const std::map<std::string, geometry_msgs::PoseStamped> static_collision_matrix,
                                  const std::map<std::string, geometry_msgs::PoseStamped> robot_collision_matrix,
                                  const double& collision_threshold_distance,
                                  const double& weight_factor
                                 );


  /** public data member*/
  // visulaize all volumes
  visualization_msgs::MarkerArray marker_array_;

  // collision matrix
  std::map<std::string, geometry_msgs::PoseStamped> collision_matrix_;

  // collision cost vector
  Eigen::VectorXd collision_cost_vector_;

private:
  // marker publisher
  ros::Publisher marker_pub_;

  // static frame broadcaster
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
   * @brief clearDataMember: clear vectors means free allocated memory
   */
  void clearDataMember();

};

#endif //PREDICTIVE_CONTROL_COLLISION_DETECTION_H_
