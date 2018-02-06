
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
  pd_config_.reset(new predictive_configuration());
  pd_config_->initialize();
  //pd_config_ = pd_config_ptr;
  chain_base_link_ = pd_config_->chain_base_link_;
  chain_root_link_ = pd_config_->chain_root_link_;

  // visulize distance information just for debugging purpose
  marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("CollisionAvoidance/obstacle_distance_markers", 1, true);

  add_obstacle_pub_ = this->nh_.advertise<moveit_msgs::CollisionObject>("obstacle_distance/registerObstacle", 1, true);
  //add_obstracle_pub_.getNumSubscribers() < 1

  // register collision links
  register_link_client_ = nh_.serviceClient<cob_srvs::SetString>("obstacle_distance/registerLinkOfInterest");
  register_link_client_.waitForExistence(ros::Duration(5.0));

  if (register_link_client_.exists())
  {
    ROS_INFO("Collision Avoidance has been activated! Register links!");
    if (!this->registerCollisionLinks())
    {
        ROS_ERROR("Registration of links failed. CA not possible");
    }
  }
  else
  {
    ROS_ERROR("Service is not exist yet");
  }

  //ia_server_ = new interactive_markers::InteractiveMarkerServer("marker_server", "", false);

  // subscribe obstacle distances
  obstacle_distance_sub_ = this->nh_.subscribe("obstacle_distance", 1 , &CollisionAvoidance::obstaclesDistanceCallBack, this);

  // initialize ros services
  add_static_obstacles_ = this->nh_.advertiseService("pd_control/add_static_obstacles", &CollisionAvoidance::addStaticObstacleServiceCallBack, this);
  delete_static_obstacles_ = this->nh_.advertiseService("pd_control/delete_static_obstacles", &CollisionAvoidance::deleteStaticObstacleServiceCallBack, this);

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
  if (pd_config_->activate_output_)
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

      cost_distance += exp( ( (pd_config_->minimum_collision_distance_*pd_config_->minimum_collision_distance_) -
                             (it->second.distance * it->second.distance) )/ pd_config_->collision_weight_factor_);

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
          cost_distance += exp( ( (pd_config_->minimum_collision_distance_*pd_config_->minimum_collision_distance_) -
                                 (it_map->second.distance * it_map->second.distance) )/ pd_config_->collision_weight_factor_);
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
  stamped.header.frame_id = pd_config_->chain_root_link_;
  stamped.header.stamp = ros::Time(0).now();

  // updated with service
  stamped.pose.position.x = -0.5;
  stamped.pose.position.y = 0.3;
  stamped.pose.position.z = 0.8;
  stamped.pose.orientation.w = 1.0;

  this->configureInteractiveMarker();
}

void CollisionAvoidance::configureInteractiveMarker()
{
  int_marker_.header.frame_id = pd_config_->chain_root_link_;
}

bool CollisionAvoidance::registerCollisionLinks()
{
    ROS_WARN_COND( this->pd_config_->collision_check_links_.size() <= 0,
                  "No collision_check_links set for this chain. Nothing will be registered. Ensure parameters are set correctly.");

    for (std::vector<std::string>::const_iterator it = this->pd_config_->collision_check_links_.begin();
         it != this->pd_config_->collision_check_links_.end();
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

bool CollisionAvoidance::addStaticObstacleServiceCallBack(predictive_control::StaticObstacleRequest &request,
                                                          predictive_control::StaticObstacleResponse &response)
{
  if (request.file_name.empty())
  {

    // first remove form environment
    moveit_msgs::CollisionObject co = request.static_collision_object;
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    add_obstacle_pub_.publish(co);

    // check already exist into environment, this operation known as disallowed collision object
    for (auto it = ignore_obstacles_.begin(); it != ignore_obstacles_.end(); ++it)
    {
      // already exist that remove from allowed collision matrix list
      if (it->find(request.static_collision_object.id) != std::string::npos)
      {
        ROS_INFO("%s already exist", it->c_str());
        ignore_obstacles_.erase(it);
        --it;
      }
    }
    add_obstacle_pub_.publish(request.static_collision_object);
    response.message = "Allowed static obstacles Successfully!!";
    response.success = true;
  }

  else if (!request.file_name.empty())
  {

    moveit_msgs::CollisionObject co;
    this->readDataFromFile(request.file_name, request.static_collision_object.id, co);

    /*for (auto it = co.begin(); it != co.end(); ++it)
    {
      ROS_WARN_STREAM(it->primitive_poses);

      add_obstacle_pub_.publish(*it);
      ros::Duration(2.0).sleep();
      response.message = "Add static obstacles Successfully!!";
      response.success = true;
    }*/

    add_obstacle_pub_.publish(co);
    response.message = "Add static obstacles Successfully!!";
    response.success = true;
  }

  else
  {
    response.message = "No specific correct formate of file!!";
    response.success = false;
    return false;
  }

  return true;
}

bool CollisionAvoidance::deleteStaticObstacleServiceCallBack(predictive_control::StaticObstacleRequest &request,
                                                             predictive_control::StaticObstacleResponse &response)
{
  if (request.file_name.empty())
  {
    ignore_obstacles_.push_back(request.static_collision_object.id);
    add_obstacle_pub_.publish(request.static_collision_object);
    response.message = "Delete Successfully!!";
    response.success = true;
  }
  return true;
}


void CollisionAvoidance::readDataFromFile(const std::string &file_name,
                                          const std::string &object_name,
                                          moveit_msgs::CollisionObject &co)
{
    geometry_msgs::PoseStamped stamped;
    shape_msgs::SolidPrimitive primitive;
    primitive.dimensions.resize(3);

    // get transformation, assume object name same as frame of object
    getTransform(chain_root_link_, object_name, stamped);

    // initialize static objects
    std::ifstream myfile;
    std::string line, id;

    std::string object_id;
    object_id = object_name;

    std::string filename = ros::package::getPath("predictive_control") + "/planning_scene/"+ file_name + ".scene";
    myfile.open(filename.c_str());

    // check file is open
    if (myfile.is_open())
    {
      getline(myfile, line);  //1 line

      while(!myfile.eof())
          {

            getline(myfile, id);  //2 line

            if(id == ".")
              break;

            ROS_ERROR_STREAM("object id:"<<id);
            co.id = id;

            getline(myfile, line);  // 3 line not useful line
            getline(myfile, line);  // 4 line

            if(line.compare("box") == 0)
            {
              primitive.type = shape_msgs::SolidPrimitive::BOX;
            }

            else if( line.compare("cylinder") == 0)
            {
              primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
            }

            else if( line.compare("sphere") == 0)
            {
              primitive.type = shape_msgs::SolidPrimitive::SPHERE;
            }

            else
            {
              std::string message("Shape of object is not defined correctly, check file on location " + filename);
              ROS_ERROR("StaticCollision: %s", message.c_str());
              return;
            }

            //myfile>>primitive.dimensions[0]>>primitive.dimensions[1]>>primitive.dimensions[2];  //5 line
            myfile>>primitive.dimensions[0] >>primitive.dimensions[1]>>primitive.dimensions[2];  //5 line
            getline (myfile,line);

            // primitive shapes
            co.primitives.push_back(primitive);

            myfile>>stamped.pose.position.x>>stamped.pose.position.y>>stamped.pose.position.z;   //6 line

            getline (myfile,line);
            myfile>>stamped.pose.orientation.x>>stamped.pose.orientation.y>>stamped.pose.orientation.z >> stamped.pose.orientation.w; //7 line

            if( (sqrt(stamped.pose.orientation.w*stamped.pose.orientation.w +
                stamped.pose.orientation.x*stamped.pose.orientation.x +
                stamped.pose.orientation.y*stamped.pose.orientation.y +
                stamped.pose.orientation.z*stamped.pose.orientation.z) ) == 0.0)
            {
                stamped.pose.orientation.w = 1.0;
                stamped.pose.orientation.x = 0.0;
                stamped.pose.orientation.y = 0.0;
                stamped.pose.orientation.z = 0.0;
            }

            ROS_WARN_STREAM(stamped);

            co.operation = moveit_msgs::CollisionObject::ADD;

            // primitive poses
            co.header.stamp = stamped.header.stamp; //request.primitive_pose.header.stamp;
            co.header.frame_id = object_name; //request.primitive_pose.header.frame_id;
            // add object into collision matrix for cost calculation
            co.primitive_poses.push_back(stamped.pose); //request.primitive_pose;

            getline(myfile, line);	// 8 line not useful line
            getline(myfile, line);	// 9 line not useful line

            //add_obstacle_pub_.publish(co);
            //ros::Duration(2.0).sleep();
      }
      myfile.close();
    }
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
      ROS_ERROR("predictive_control_ros_node::getTransform: \n%s", ex.what());
    }
  }

  else
  {
    ROS_WARN("%s or %s frame doesn't exist, pass existing frame", from.c_str(), to.c_str());
  }

  return transform;
}
