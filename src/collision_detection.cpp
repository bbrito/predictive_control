
#include <predictive_control/collision_detection.h>

CollisionRobot::CollisionRobot()
{
  // make sure predictice_configuration class initialized
  if (!predictive_configuration::initialize_success_)
  {
    predictive_configuration::initialize();
  }

  clear_data_member();
}

CollisionRobot::~CollisionRobot()
{
  clear_data_member();
}

// diallocated memory
void CollisionRobot::clear_data_member()
{
  marker_array_.markers.clear();
  collision_matrix_.clear();
}

// update collision detection, specifically center position of collision matrix
void CollisionRobot::createCollisionMatrix(const std::vector<Eigen::MatrixXd> &FK_Homogenous_Matrix, const std::vector<Eigen::MatrixXd> &Transformation_Matrix)
{
  int point = 0u, counter = 0u;
  std::string key = "point_";

  //for (auto const& it: FK_Homogenous_Matrix)
  for (auto it = FK_Homogenous_Matrix.begin(); it != FK_Homogenous_Matrix.end(); ++it)
  {
    if (Transformation_Matrix[counter](2,3) > 0.10)
    {
      // distance between two frame are more than ball randius than add intermidate ball
      if( Transformation_Matrix[counter](2,3) > predictive_configuration::ball_radius_)
      {
        ROS_INFO("CollisionRobot: Add intermidiate volume with point: %s", (key+std::to_string(point)).c_str());

        geometry_msgs::PoseStamped stamped;
        KDL::Frame frame, previous_frame;

        // transform Eigen Matrix to Kdl Frame
        transformEigenMatrixToKDL(*(it-1), previous_frame);
        transformEigenMatrixToKDL(*it, frame);

        // fill up pose stamped
        stamped.header.frame_id = predictive_configuration::chain_root_link_;
        stamped.header.stamp = ros::Time().now();
        stamped.pose.position.x = previous_frame.p.x() + (frame.p.x() - previous_frame.p.x()*0.5);
        stamped.pose.position.y = previous_frame.p.y() + (frame.p.y() - previous_frame.p.y()*0.5);
        stamped.pose.position.z = previous_frame.p.z() + (frame.p.z() - previous_frame.p.z()*0.5);
        previous_frame.M.GetQuaternion(stamped.pose.orientation.x,
                              stamped.pose.orientation.y,
                              stamped.pose.orientation.z,
                              stamped.pose.orientation.w
                              );

        collision_matrix_[key + std::to_string(point)] = stamped;
        point = point + 1;
      }

      // as usally add ball at every joint
      geometry_msgs::PoseStamped stamped;
      KDL::Frame frame;

      // transform Eigen Matrix to Kdl Frame
      transformEigenMatrixToKDL(*it, frame);

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
    else
    {
      ;
    }
    counter = counter + 1;
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

// create static frame, just for visualization purpose
void CollisionRobot::createStaticFrame(const geometry_msgs::PoseStamped &stamped, const std::string &frame_name)
{
  geometry_msgs::TransformStamped static_transformStamped;

  // frame information
  static_transformStamped.header.stamp = stamped.header.stamp;
  static_transformStamped.header.frame_id = stamped.header.frame_id;
  static_transformStamped.child_frame_id = frame_name;

  // pose of frame relative to header frame_id
  static_transformStamped.transform.translation.x = stamped.pose.position.x;
  static_transformStamped.transform.translation.y = stamped.pose.position.y;
  static_transformStamped.transform.translation.z = stamped.pose.position.z;
  static_transformStamped.transform.rotation = stamped.pose.orientation;

  ROS_INFO("Created intermediate 'Static Frame' with '%s' parent frame id and '%s' child frame id",
           stamped.header.frame_id.c_str(), frame_name.c_str()
           );

  static_broadcaster_.sendTransform(static_transformStamped);
  ros::spinOnce();
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
