
#include <predictive_control/collision_avoidance.h>

CollisionAvoidance::CollisionAvoidance()
{
  ;
}

CollisionAvoidance::~CollisionAvoidance()
{
  ;
}

bool CollisionAvoidance::initialize(const boost::shared_ptr<predictive_configuration> &controller_config_ptr)
{
  controller_config_.reset(new predictive_configuration());
  controller_config_->initialize();
  //controller_config_ = controller_config_ptr;
  robot_base_link_ = controller_config_->robot_base_link_;

  // visulize distance information just for debugging purpose
  marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("CollisionAvoidance/obstacle_distance_markers", 1, true);

  add_obstacle_pub_ = this->nh_.advertise<moveit_msgs::CollisionObject>("obstacle_distance/registerObstacle", 1, true);
  //add_obstracle_pub_.getNumSubscribers() < 1

  //ia_server_ = new interactive_markers::InteractiveMarkerServer("marker_server", "", false);

  // subscribe obstacle distances
  obstacle_distance_sub_ = this->nh_.subscribe("obstacle_distance", 1 , &CollisionAvoidance::obstaclesDistanceCallBack, this);

  ROS_WARN("COLLIISION_AVOIDANCE SUCCESFFULLY INITIALIZED!!");

  return true;
}

void CollisionAvoidance::obstaclesDistanceCallBack(const cob_control_msgs::ObstacleDistances::ConstPtr &msg)
{
  relevant_obstacle_distances_.clear();

  for (uint32_t i = 0; i < msg->distances.size(); ++i)
  {
    const std::string id = msg->distances.at(i).link_of_interest;
    if (relevant_obstacle_distances_.count(id) > 0)
    {
      if (relevant_obstacle_distances_[id].distance > msg->distances[i].distance)
      {
        relevant_obstacle_distances_[id] = msg->distances[i];
      }
    }
    else
    {
      relevant_obstacle_distances_[id] = msg->distances.at(i);
    }
  }

  // DEBUG
  if (controller_config_->activate_debug_output_)
  {
    for (std::map<std::string, cob_control_msgs::ObstacleDistance>::const_iterator it = relevant_obstacle_distances_.begin();
             it != relevant_obstacle_distances_.end(); ++it)
    {
      ROS_WARN_STREAM("link of interest: "<< it->second.link_of_interest);
      ROS_WARN_STREAM("Obstacle_id: "<< it->second.obstacle_id);
      ROS_INFO_STREAM("Frame Vector: "<<it->second.frame_vector);
      ROS_INFO_STREAM("Nearest_point_frame_vector: " <<it->second.nearest_point_obstacle_vector);
      ROS_INFO_STREAM("Nearest_point_obstacle_vector: "<<it->second.nearest_point_obstacle_vector);
      }

  }

  this->getDistanceCostFunction();

  this->visualizeObstacleDistance(relevant_obstacle_distances_);

}

double CollisionAvoidance::getDistanceCostFunction()
{
  double cost_distance(0.0);


  if (ignore_obstacles_.empty())
    {
    for (std::map<std::string, cob_control_msgs::ObstacleDistance>::const_iterator it = relevant_obstacle_distances_.begin();
         it != relevant_obstacle_distances_.end(); ++it)
    {
      ROS_ERROR_STREAM(it->second.link_of_interest);
      ROS_ERROR_STREAM(it->second.obstacle_id);
      ROS_WARN_STREAM(it->second.distance);
      ROS_INFO_STREAM("Frame Vector: "<<it->second.frame_vector);
      ROS_INFO_STREAM("Nearest_point_frame_vector: " <<it->second.nearest_point_obstacle_vector);
      ROS_INFO_STREAM("Nearest_point_obstacle_vector: "<<it->second.nearest_point_obstacle_vector);

      cost_distance += exp( ( (controller_config_->minimum_collision_distance_*controller_config_->minimum_collision_distance_) -
                             (it->second.distance * it->second.distance) )/ controller_config_->collision_weight_factor_);

    }
  }
  else
  {
    // ignoring obstacles which recieved request of allowed collision
    for (auto it = ignore_obstacles_.begin(); it != ignore_obstacles_.end(); ++it)
    {
      for (std::map<std::string, cob_control_msgs::ObstacleDistance>::const_iterator it_map = relevant_obstacle_distances_.begin();
           it_map != relevant_obstacle_distances_.end(); ++it_map)
      {
        // both string are equal than execute if loop
        ROS_WARN_STREAM(it_map->second.obstacle_id);
        if (it->find(it_map->second.obstacle_id) != std::string::npos || it->find(it_map->second.link_of_interest) != std::string::npos)
        {
          ROS_INFO("Ignoring %s obstacle from list", it->c_str());
        }
        else
        {
          cost_distance += exp( ( (controller_config_->minimum_collision_distance_*controller_config_->minimum_collision_distance_) -
                                 (it_map->second.distance * it_map->second.distance) )/ controller_config_->collision_weight_factor_);
        }
      }
     }
    }

  ROS_WARN_STREAM("COLLISION COST: "<<cost_distance);

  return cost_distance;
}

bool CollisionAvoidance::registerCollisionOjbect(const std::string& obstacle_name)
{

  moveit_msgs::CollisionObject collision_object;
  collision_object.id = "Interactive Box";
  collision_object.header.frame_id =  obstacle_name;
  collision_object.operation = moveit_msgs::CollisionObject::ADD;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.push_back(0.15);
  collision_object.primitives.push_back(primitive);

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  collision_object.primitive_poses.push_back(pose);

  //compose interactive marker
  geometry_msgs::PoseStamped stamped;
  stamped.header.frame_id = ""; // update frame
  stamped.header.stamp = ros::Time(0).now();

  // updated with service
  stamped.pose.position.x = -0.5;
  stamped.pose.position.y = 0.3;
  stamped.pose.position.z = 0.8;
  stamped.pose.orientation.w = 1.0;

}


void CollisionAvoidance::visualizeObstacleDistance(const std::map<std::string, cob_control_msgs::ObstacleDistance> &distnace_matrix)
{
  visualization_msgs::MarkerArray marker_array;

  for (std::map<std::string, cob_control_msgs::ObstacleDistance>::const_iterator it = distnace_matrix.begin();
       it != distnace_matrix.end(); ++it)
  {
    // show distance vector as arrow
    visualization_msgs::Marker marker_vector;
    marker_vector.type = visualization_msgs::Marker::ARROW;
    marker_vector.lifetime = ros::Duration(0.5);
    marker_vector.action = visualization_msgs::Marker::ADD;
    marker_vector.ns = it->first;
    marker_vector.id = 42;
    marker_vector.header.frame_id = robot_base_link_;

    marker_vector.scale.x = 0.01;
    marker_vector.scale.y = 0.05;

    // Vector pointing to the nearest point on the obstacle collision geometry (e.g. mesh)
    geometry_msgs::Point start;
    start.x = it->second.nearest_point_obstacle_vector.x;
    start.y = it->second.nearest_point_obstacle_vector.y;
    start.z = it->second.nearest_point_obstacle_vector.z;

    //ROS_WARN_STREAM(start);

    // Vector pointing to the nearest point on the link collision geometry (e.g. mesh)
    geometry_msgs::Point end;
    end.x = it->second.nearest_point_frame_vector.x;
    end.y = it->second.nearest_point_frame_vector.y;
    end.z = it->second.nearest_point_frame_vector.z;

    //ROS_WARN_STREAM(end);

    marker_vector.color.a = 1.0;
    marker_vector.color.g = 1.0;

    marker_vector.points.push_back(start);
    marker_vector.points.push_back(end);
    marker_array.markers.push_back(marker_vector);

    // show distance as text
    visualization_msgs::Marker marker_distance;
    marker_distance.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_distance.lifetime = ros::Duration(0.5);
    marker_distance.action = visualization_msgs::Marker::ADD;
    marker_distance.ns = it->first;
    marker_distance.id = 69;
    marker_distance.header.frame_id = robot_base_link_;
    marker_distance.text = boost::lexical_cast<std::string>(boost::format("%.3f") % it->second.distance);

    marker_distance.scale.x = 0.1;
    marker_distance.scale.y = 0.1;
    marker_distance.scale.z = 0.1;

    marker_distance.color.a = 1.0;
    // marker_distance.color.r = 1.0;
    // marker_distance.color.g = 1.0;
    // marker_distance.color.b = 1.0;
    marker_distance.color.r = 0.0;
    marker_distance.color.g = 0.0;
    marker_distance.color.b = 0.0;

    marker_distance.pose.position.x = it->second.nearest_point_frame_vector.x;
    marker_distance.pose.position.y = it->second.nearest_point_frame_vector.y + 0.05;
    marker_distance.pose.position.z = it->second.nearest_point_frame_vector.z;

    marker_array.markers.push_back(marker_distance);
  }

  this->marker_pub_.publish(marker_array);
}

bool CollisionAvoidance::getTransform(const std::string& from, const std::string& to, geometry_msgs::PoseStamped& stamped_pose)
{
  bool transform = false;
  tf::StampedTransform stamped_tf;

  // make sure source and target frame exist
  if (tf_listener_.frameExists(to) & tf_listener_.frameExists(from))
  {
    try
    {
      // find transforamtion between souce and target frame
      tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.2));
      tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);

      // rotation
      stamped_pose.pose.orientation.w =  stamped_tf.getRotation().getW();
      stamped_pose.pose.orientation.x =  stamped_tf.getRotation().getX();
      stamped_pose.pose.orientation.y =  stamped_tf.getRotation().getY();
      stamped_pose.pose.orientation.z =  stamped_tf.getRotation().getZ();

      // translation
      stamped_pose.pose.position.x = stamped_tf.getOrigin().x();
      stamped_pose.pose.position.y = stamped_tf.getOrigin().y();
      stamped_pose.pose.position.z = stamped_tf.getOrigin().z();

      // header frame_id should be parent frame
      stamped_pose.header.frame_id = stamped_tf.frame_id_;  //from or to
      stamped_pose.header.stamp = ros::Time(0);

      transform = true;
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("MPCC_node::getTransform: \n%s", ex.what());
    }
  }

  else
  {
    ROS_WARN("%s or %s frame doesn't exist, pass existing frame", from.c_str(), to.c_str());
  }

  return transform;
}
