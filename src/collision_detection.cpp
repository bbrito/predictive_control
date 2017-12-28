
#include <predictive_control/collision_detection.h>

CollisionRobot::CollisionRobot()
{
  ;
}

CollisionRobot::~CollisionRobot()
{
  ;
}

// diallocated memory
void CollisionRobot::clear_data_member()
{
  marker_array_.markers.clear();
}

// initialize collision detection, crete array of collision matrix
bool CollisionRobot::initialize(const std::vector<Eigen::MatrixXd> &FK_Homogenous_Matrix)
{
  // make sure predictice_configuration class initialized
  if (!predictive_configuration::initialize_success_)
  {
    predictive_configuration::initialize();
  }

  // construct/initialize urdf model using robot description
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("CollisionRobot: Failed to parse urdf file for JointLimits");
    return false;
  }



}


void CollisionRobot::initializeDataMember(const std::vector<Eigen::MatrixXd> &FK_Homogenous_Matrix)
{
  int point = 0u;
  std::string key = "point_";
  for (const auto& it: FK_Homogenous_Matrix)
  {
    geometry_msgs::PoseStamped stamped;
    KDL::Frame frame;

    // transform Eigen Matrix to Kdl Frame
    transformEigenMatrixToKDL(it, frame);

    // fill up pose stamped
    stamped.header.frame_id = predictive_configuration::chain_root_link_;
    stamped.header.stamp = ros::Time().now();
    stamped.pose.position.x = frame.p.x();
    stamped.pose.position.y = frame.p.y();
    stamped.pose.position.z = frame.p.z();
    frame.M.GetQuaternion(stamped.pose.orientation.x,
                          stamped.pose.orientation.y,
                          stamped.pose.orientation.z,
                          stamped.pose.orientation.w
                          );

    collision_matrix_[key + std::to_string(point)] = stamped;
    point = point + 1;
  }

}

// generate collision around robot body
void CollisionRobot::generateCollisionVolume(const geometry_msgs::PoseStamped &center, const double &radius, const uint32_t &ball_id)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "preview";

  // texture
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.1;

  // dimension
  marker.scale.x = 2.0 * radius;
  marker.scale.y = 2.0 * radius;
  marker.scale.z = 2.0 * radius;

  // position into world
  marker.id = ball_id;
  marker.header.frame_id = center.header.frame_id;
  marker.pose.position.x = center.pose.position.x;
  marker.pose.position.y = center.pose.position.y;
  marker.pose.position.z = center.pose.position.z;
  marker.pose.orientation.w = center.pose.orientation.w;
  marker.pose.orientation.x = center.pose.orientation.x;
  marker.pose.orientation.y = center.pose.orientation.y;
  marker.pose.orientation.z = center.pose.orientation.z;

  // store created marker
  marker_array_.markers.push_back(marker);
}

//convert KDL to Eigen matrix
void CollisionRobot::transformKDLToEigenMatrix(const KDL::Frame &frame, Eigen::MatrixXd &matrix)
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
}

//convert Eigen matrix to KDL::Frame
void CollisionRobot::transformEigenMatrixToKDL(const Eigen::MatrixXd& matrix, KDL::Frame& frame)
{
  // translation
  for (unsigned int i = 0; i < 3; ++i)
  {
    frame.p[i] = matrix(i, 3);
  }

  // rotation matrix
  for (unsigned int i = 0; i < 9; ++i)
  {
    frame.M.data[i] = matrix(i/3, i%3);
  }
}
