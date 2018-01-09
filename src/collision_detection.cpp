
#include <predictive_control/collision_detection.h>

CollisionRobot::CollisionRobot()
{
;
}

CollisionRobot::~CollisionRobot()
{
  clearDataMember();
}

// diallocated memory
void CollisionRobot::clearDataMember()
{
  marker_array_.markers.clear();
  collision_matrix_.clear();
}

// initialize and create publisher for publishing collsion ball marker
bool CollisionRobot::initializeCollisionRobot()
{
  // make sure predictice_configuration class initialized
  if (!predictive_configuration::initialize_success_)
  {
    predictive_configuration::initialize();
  }

  clearDataMember();

  ros::NodeHandle nh_collisionRobot("predictive_control/collisionRobot");
  marker_pub_ = nh_collisionRobot.advertise<visualization_msgs::MarkerArray>("collision_ball", 1);

  ROS_INFO("===== Collision Ball marker published with topic: ~/predictive_control/collisionRobot/collision_ball =====");
  ROS_WARN("COLLISIONROBOT INITIALIZED!!");

  return true;
}

// update collsion ball position, publish new position of collision ball
void CollisionRobot::updateCollisionVolume(const std::vector<Eigen::MatrixXd> &FK_Homogenous_Matrix,
                                           const std::vector<Eigen::MatrixXd> &Transformation_Matrix)
{
  // make sure collsion matrix and marker array should be empty
  clearDataMember();

  // DEBUG
  if (predictive_configuration::activate_output_)
  {
   ROS_WARN("########### Print FK HOMOGENOUS MATRIX ############");
   for (auto it = FK_Homogenous_Matrix.begin(); it != FK_Homogenous_Matrix.end(); ++it)
   {
      std::cout <<"\n" <<*it << std::endl;
   }
   ROS_WARN("########## Print Transformation MATRIX ############");
   for (auto it = Transformation_Matrix.begin(); it != Transformation_Matrix.end(); ++it)
   {
      std::cout <<"\n" <<*it << std::endl;
   }
  }

  // generate/update collision matrix
  generateCollisionVolume(FK_Homogenous_Matrix, Transformation_Matrix);

  // DEBUG
  if (predictive_configuration::activate_output_)
  {
    ROS_WARN("===== COLLISION MATRIX =====");
    for (auto const& it: collision_matrix_)
    {
      ROS_INFO_STREAM("CollisionRobot: "<<it.first << " -> stamped: \n" << it.second);
    }
  }

  // visualize marker array
  int id = 0u;
  for (auto it = collision_matrix_.begin(); it != collision_matrix_.end(); ++it, ++id)
  {
    visualizeCollisionVolume(it->second, predictive_configuration::ball_radius_, id);
  }

  // publish
  marker_pub_.publish(marker_array_);

  // compute collision cost vectors
  computeCollisionCost(collision_matrix_, predictive_configuration::minimum_collision_distance_,
                       predictive_configuration::collision_weight_factor_);

  // DEBUG
  if (true) //predictive_configuration::activate_output_
  {
    ROS_WARN("===== COLLISION COST VECTOR =====");
    std::cout << collision_cost_vector_.transpose() << std::endl;
  }
}

// create collision detection, specifically center position of collision matrix
void CollisionRobot::generateCollisionVolume(const std::vector<Eigen::MatrixXd> &FK_Homogenous_Matrix,
                                             const std::vector<Eigen::MatrixXd> &Transformation_Matrix)
{

  int point = 0u, counter = 0u;
  std::string key = "point_";

  //for (auto const& it: FK_Homogenous_Matrix)
  for (auto it = FK_Homogenous_Matrix.begin(); it != FK_Homogenous_Matrix.end(); ++it)
  {
    if (Transformation_Matrix[counter](2,3) > 0.10 && counter != 0)
    {
      // distance between two frame are more than ball randius than add intermidate ball
      if( Transformation_Matrix[counter](2,3) > predictive_configuration::ball_radius_) //0.30
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
        stamped.pose.position.x = previous_frame.p.x() + ((frame.p.x() - previous_frame.p.x())*0.5);
        stamped.pose.position.y = previous_frame.p.y() + ((frame.p.y() - previous_frame.p.y())*0.5);
        stamped.pose.position.z = previous_frame.p.z() + ((frame.p.z() - previous_frame.p.z())*0.5);
        previous_frame.M.GetQuaternion(stamped.pose.orientation.x,
                              stamped.pose.orientation.y,
                              stamped.pose.orientation.z,
                              stamped.pose.orientation.w
                              );
        // broadcast static frame
        createStaticFrame(stamped, key + std::to_string(point));
        collision_matrix_[key + std::to_string(point)] = stamped;
        point = point + 1;
      }
      /*
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
      point = point + 1;*/
    }
    else
    {
      ;
    }
    counter = counter + 1;
  }

}

// generate collision around robot body
void CollisionRobot::visualizeCollisionVolume(const geometry_msgs::PoseStamped &center,
                                              const double &radius, const uint32_t &ball_id)
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

// compute collsion cost such way that below minimum distance cost goes to infinite, and far distance costs goes to zero
void CollisionRobot::computeCollisionCost(const std::map<std::string, geometry_msgs::PoseStamped> collision_matrix,
                                                     const double &collision_min_distance,
                                                     const double &weight_factor)
{
  collision_cost_vector_ = Eigen::VectorXd(collision_matrix.size());

  // iterate to one by one point in collision matrix
  int loop_counter = 0u;
  for (auto it_out = collision_matrix.begin(); it_out != collision_matrix.end(); ++it_out, ++loop_counter)
  {
    double dist = 0.0;
    for (auto it_in = collision_matrix.begin(); it_in != collision_matrix.end(); ++it_in)
    {
      // both string are not equal than execute if loop
      if (it_out->first.find(it_in->first) == std::string::npos)
      {
        // logistic cost function
        // Nonlinear Model Predictive Control for Multi-Micro Aerial Vehicle Robust Collision Avoidance
        // https://arxiv.org/pdf/1703.01164.pdf ... equation(10)
        ROS_DEBUG(" '%s'  <---> '%s'", it_out->first.c_str(),it_in->first.c_str());
        //dist += exp( ((collision_min_distance) -
        //              (std::abs(getEuclideanDistance(it_out->second.pose, it_in->second.pose))) ) / weight_factor);
        dist += exp( ((collision_min_distance*collision_min_distance) -
                      (std::abs(getEuclideanDistance(it_out->second.pose, it_in->second.pose)) * std::abs(getEuclideanDistance(it_out->second.pose, it_in->second.pose))) ) / weight_factor);

        ROS_DEBUG_STREAM("Exponential term: "<<
                         exp(collision_min_distance - std::abs(getEuclideanDistance(it_out->second.pose, it_in->second.pose)) / weight_factor));
      }
    }
    //store cost of each point into vector
    collision_cost_vector_(loop_counter) = dist;
  }
}

// create static frame, just for visualization purpose
void CollisionRobot::createStaticFrame(const geometry_msgs::PoseStamped &stamped,
                                       const std::string &frame_name)
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
void CollisionRobot::transformKDLToEigenMatrix(const KDL::Frame &frame,
                                               Eigen::MatrixXd &matrix)
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
void CollisionRobot::transformEigenMatrixToKDL(const Eigen::MatrixXd& matrix,
                                               KDL::Frame& frame)
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


//-------------------------------------------------------------------------------------------------------------------------------
//--------------------------- Static Collision Object Avoidance ----------------------------------
//--------------------------------------------------------------------------------------------------------------------------------

StaticCollision::StaticCollision()
{
;
}

StaticCollision::~StaticCollision()
{
  clearDataMember();
}

// diallocated memory
void StaticCollision::clearDataMember()
{
  marker_array_.markers.clear();
  collision_matrix_.clear();
}

// initialize and create publisher for publishing collsion ball marker
bool StaticCollision::initializeStaticCollisionObject()
{
  // make sure predictice_configuration class initialized
  if (!predictive_configuration::initialize_success_)
  {
    predictive_configuration::initialize();
  }

  clearDataMember();

  ros::NodeHandle nh_collisionRobot("predictive_control/collisionRobot");
  marker_pub_ = nh_collisionRobot.advertise<visualization_msgs::MarkerArray>("collision_ball", 1);

  geometry_msgs::PoseStamped stamped;

  // generate static collision volume
  generateStaticCollisionVolume(stamped);

  // visualize static collision volume
  visualizeStaticCollisionVoulme(stamped);

  ROS_INFO("===== Collision Ball marker published with topic: ~/predictive_control/collisionRobot/collision_ball =====");
  ROS_WARN("COLLISIONROBOT INITIALIZED!!");

  return true;
}

// create collision cost for static objects
void StaticCollision::generateStaticCollisionVolume(geometry_msgs::PoseStamped& stamped)
{
  // position and orientation

  stamped.header.frame_id = predictive_configuration::chain_root_link_;
  stamped.header.stamp = ros::Time().now();

  //position of static collision object
  stamped.pose.position.x = 1.0;
  stamped.pose.position.y = 1.0;
  stamped.pose.position.z = 0.0;

  // orientation of static collision object
  stamped.pose.orientation.w = 1.0;
  stamped.pose.orientation.x = 0.0;
  stamped.pose.orientation.y = 0.0;
  stamped.pose.orientation.z = 0.0;

  collision_matrix_["box"] = stamped;

  // visualize static collision voulume
  createStaticFrame(stamped, "box");
}

// visualize static collision object
void StaticCollision::visualizeStaticCollisionVoulme(const geometry_msgs::PoseStamped &stamped)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "preview";

  // texture
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.1;

  // dimension
  marker.scale.x = 0.30;
  marker.scale.y = 0.30;
  marker.scale.z = 0.30;

  // position into world
  marker.id = 101;
  marker.header.frame_id = stamped.header.frame_id;
  marker.pose.position.x = stamped.pose.position.x;
  marker.pose.position.y = stamped.pose.position.y;
  marker.pose.position.z = stamped.pose.position.z;
  marker.pose.orientation.w = stamped.pose.orientation.w;
  marker.pose.orientation.x = stamped.pose.orientation.x;
  marker.pose.orientation.y = stamped.pose.orientation.y;
  marker.pose.orientation.z = stamped.pose.orientation.z;

  // store created marker
  marker_array_.markers.push_back(marker);
}
