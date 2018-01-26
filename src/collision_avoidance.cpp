
#include <predictive_control/collision_avoidance.h>

CollisionAvoidance::CollisionAvoidance()
{
  ;
}

CollisionAvoidance::~CollisionAvoidance()
{
  ;
}

bool CollisionAvoidance::initialize(const boost::shared_ptr<predictive_configuration> &pd_config_ptr)
{
  /*pd_config_.reset(new predictive_configuration());
  pd_config_->initialize();*/
  pd_config_ = pd_config_ptr;
  chain_base_link_ = pd_config_->chain_base_link_;

  // visulize distance information just for debugging purpose
  marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("CollisionAvoidance/obstacle_distance_markers", 1, true);

  // register collision links
  register_link_client_ = nh_.serviceClient<cob_srvs::SetString>("obstacle_distance/registerLinkOfInterest");
  register_link_client_.waitForExistence(ros::Duration(5.0));

  // subscribe obstacle distances
  obstacle_distance_sub_ = this->nh_.subscribe("obstacle_distance", 1 , &CollisionAvoidance::obstaclesDistanceCallBack, this);

  return 0;
}
/*
bool CobTwistController::registerCollisionLinks()
{
    ROS_WARN_COND(twist_controller_params_.collision_check_links.size() <= 0,
                  "No collision_check_links set for this chain. Nothing will be registered. Ensure parameters are set correctly.");

    for (std::vector<std::string>::const_iterator it = twist_controller_params_.collision_check_links.begin();
         it != twist_controller_params_.collision_check_links.end();
         it++)
    {
        ROS_INFO_STREAM("Trying to register for " << *it);
        cob_srvs::SetString r;
        r.request.data = *it;
        if (register_link_client_.call(r))
        {
            ROS_INFO_STREAM("Called registration service with success: " << int(r.response.success) << ". Got message: " << r.response.message);
            if (!r.response.success)
            {
                return false;
            }
        }
        else
        {
            ROS_WARN_STREAM("Failed to call registration service for namespace: " << nh_.getNamespace());
            return false;
        }
    }

    return true;
}
*/
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

  this->visualizeObstacleDistance(relevant_obstacle_distances_);

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
    marker_vector.header.frame_id = chain_base_link_;

    marker_vector.scale.x = 0.01;
    marker_vector.scale.y = 0.05;

    // Vector pointing to the nearest point on the obstacle collision geometry (e.g. mesh)
    geometry_msgs::Point start;
    start.x = it->second.nearest_point_obstacle_vector.x;
    start.y = it->second.nearest_point_obstacle_vector.y;
    start.z = it->second.nearest_point_obstacle_vector.z;

    ROS_WARN_STREAM(start);

    // Vector pointing to the nearest point on the link collision geometry (e.g. mesh)
    geometry_msgs::Point end;
    end.x = it->second.nearest_point_frame_vector.x;
    end.y = it->second.nearest_point_frame_vector.y;
    end.z = it->second.nearest_point_frame_vector.z;

    ROS_WARN_STREAM(end);

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
    marker_distance.header.frame_id = chain_base_link_;
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
