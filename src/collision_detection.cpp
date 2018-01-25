
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
    if (Transformation_Matrix[counter](2,3) > 0.15 && counter != 0)
    {
      // distance between two frame are more than ball randius than add intermidate ball
      if( Transformation_Matrix[counter](2,3) > 0.20) //predictive_configuration::ball_radius_
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

  ros::NodeHandle nh_collisionRobot("predictive_control/StaticCollision");
  marker_pub_ = nh_collisionRobot.advertise<visualization_msgs::MarkerArray>("static_collision_object", 1);
  ROS_INFO("===== static collision marker published with topic: ~/predictive_control/collisionRobot/static_collision_object =====");

  // serice publisher
  add_static_object_ = nh_collisionRobot.advertiseService("add_static_object", &StaticCollision::addStaticObjectServiceCB, this);
  remove_static_object_ = nh_collisionRobot.advertiseService("remove_static_object", &StaticCollision::removeStaticObjectServiceCB, this);
  remove_all_static_objects_ = nh_collisionRobot.advertiseService("remove_all_static_objects", &StaticCollision::removeAllStaticObjectsServiceCB, this);
  ROS_INFO("===== add static object published with topic: ~/predictive_control/collisionRobot/add_static_object =====");
  ROS_INFO("===== remove static object published with topic: ~/predictive_control/collisionRobot/remove_static_object =====");
  ROS_INFO("===== remove static object published with topic: ~/predictive_control/collisionRobot/remove_all_static_objects =====");


  ROS_WARN("STATIC COLLISION INITIALIZED!!");

  // generate static collision volume
  //generateStaticCollisionVolume();


  // DEBUG
  if (true)
  {
    ROS_WARN("===== STATIC COLLISION MATRIX =====");
    for (auto const& it: collision_matrix_)
    {
      ROS_INFO_STREAM("StaticCollision: "<<it.first << " -> stamped: \n" << it.second);
    }
  }

  // visualize marker array
  int id = 0u;
  for (auto it = collision_matrix_.begin(); it != collision_matrix_.end(); ++it, ++id)
  {
    visualizeStaticCollisionVoulme(it->second);
  }

  return true;
}

// add collision object into environement
bool StaticCollision::addStaticObjectServiceCB(predictive_control::StaticCollisionObjectRequest &request,
                                               predictive_control::StaticCollisionObjectResponse &response)
{
  // default response
  response.success = true;
  response.message = "Default set added";

  // read data from service request
  if (request.file_name.empty())
  {
    ROS_WARN("Recieved add static object service call ...");

  // id should be unique
  std::string object_id = request.object_id;

  if (request.object_id.empty())
    object_id = request.object_name;

    // add object into collision matrix for cost calculation
    collision_matrix_[object_id] = request.primitive_pose;
    createStaticFrame(request.primitive_pose, object_id);

    visualization_msgs::Marker marker;

    // box
    if (request.object_name == "box" || request.object_name == "BOX")
    {
      marker.type = marker.CUBE;
    }

    // cylinder
    else if (request.object_name == "cylinder" || request.object_name == "CYLINDER")
    {
      marker.type = marker.CYLINDER;
    }

    // sphere
    else if (request.object_name == "sphere" || request.object_name == "SPHERE")
    {
      marker.type = marker.SPHERE;
    }

    else
    {
      response.success = false;
      std::string message("Shape of object is not defined correctly");
      ROS_ERROR("StaticCollision: %s", message.c_str());
      response.message = message;
      return false;
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "preview";
    marker.header.frame_id = request.primitive_pose.header.frame_id;
    marker.header.stamp = request.primitive_pose.header.stamp;
    marker.text = object_id;

    // texture
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1;

    // dimension
    marker.scale.x = request.dimension.x;
    marker.scale.y = request.dimension.y;
    marker.scale.z = request.dimension.z;

    /*marker.pose.position.x = request.collision_object.primitive_poses.at(0).pose.position.x;
    marker.pose.position.y = request.collision_object.primitive_poses.at(0).pose.position.y;
    marker.pose.position.z = request.collision_object.primitive_poses.at(0).pose.position.z;

    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;*/

    marker.pose = request.primitive_pose.pose;

    // store created marker
    marker_array_.markers.push_back(marker);
  }


  //---------------------------------------------- READ DATA FROM FILES -----------------------------
  // Here assume that at this moment object name is same as frame name
  // read static object dimenstion from files
  if (!request.file_name.empty())
  {
    geometry_msgs::PoseStamped stamped;

    // get transformation
    getTransform(predictive_configuration::chain_root_link_, request.object_name, stamped);

    // initialize static objects
    std::ifstream myfile;
    std::string line, id;

    std::string object_id;
    object_id = request.object_id;

    std::string filename = ros::package::getPath("predictive_control") + "/planning_scene/"+ request.file_name + ".scene";
    myfile.open(filename.c_str());

    // check file is open
    if (myfile.is_open())
    {
      getline(myfile, line);  //1 line

      while(!myfile.eof())
          {
            visualization_msgs::Marker marker;

            getline(myfile, id);  //2 line

            if(id == ".")
              break;

            ROS_ERROR_STREAM("object id:"<<id);
            object_id = id;

            getline(myfile, line);  // 3 line not useful line
            getline(myfile, line);  // 4 line

            if(line.compare("box") == 0)
            {
              marker.type = marker.CUBE;
            }

            else if( line.compare("cylinder") == 0)
            {
              marker.type = marker.CYLINDER;
            }

            else if( line.compare("sphere") == 0)
            {
              marker.type = marker.SPHERE;
            }

            else
            {
              response.success = false;
              std::string message("Shape of object is not defined correctly, check file on location " + filename);
              ROS_ERROR("StaticCollision: %s", message.c_str());
              response.message = message;
              return false;
            }

            //myfile>>primitive.dimensions[0]>>primitive.dimensions[1]>>primitive.dimensions[2];  //5 line
            myfile>>marker.scale.x >>marker.scale.y>>marker.scale.z;  //5 line
            getline (myfile,line);

            myfile>>marker.pose.position.x>>marker.pose.position.y>>marker.pose.position.z;   //6 line

            getline (myfile,line);
            myfile>>marker.pose.orientation.x>>marker.pose.orientation.y>>marker.pose.orientation.z >> marker.pose.orientation.w; //7 line

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

            marker.header.stamp = stamped.header.stamp; //request.primitive_pose.header.stamp;
            marker.header.frame_id = request.object_name; //request.primitive_pose.header.frame_id;
            //marker.pose = stamped.pose;

            // add object into collision matrix for cost calculation
            collision_matrix_[object_id] = stamped; //request.primitive_pose;

            getline(myfile, line);	// 8 line not useful line
            getline(myfile, line);	// 9 line not useful line

            // texture
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "preview" + object_id;
            marker.text = request.file_name+ " " + object_id;
            marker_array_.markers.push_back(marker);
      }
      myfile.close();
    }
  }

  marker_pub_.publish(marker_array_);

  // response
  response.success = true;
  response.message = ("Successfully add to the environment");

  return true;
}

// remove collision object from environment
bool StaticCollision::removeStaticObjectServiceCB(predictive_control::StaticCollisionObjectRequest &request,
                                                  predictive_control::StaticCollisionObjectResponse &response)
{
  if (request.file_name.empty())
  {
    // id should be unique
    std::string object_id = request.object_id;

    if (request.object_id.empty())
      object_id = request.object_name;

    // erase that object from collision matrix list
    auto it = collision_matrix_.find(object_id);
    collision_matrix_.erase(it);

    // remove that object form marker list, we can requst only one object to remove
    for (auto it = marker_array_.markers.begin(); it != marker_array_.markers.end(); ++it)
    {
      if (it->text == request.object_id)
      {
        it->action = visualization_msgs::Marker::DELETE;
        it->ns = "preview";

        // make sure first publish it and than remove from list to maintain list
        marker_pub_.publish(marker_array_);
        marker_array_.markers.erase(it);
        break;
      }

      // iteration reach to end, considering not found requested object
      if (it == marker_array_.markers.end())
      {
        response.success = false;
        response.message = (" Not find requsted object into list ");
        return false;
      }
    }

    // response
    response.success = true;
    response.message = ("Successfully remove to the environment");

    return true;
  }
  //------------------------ REMOVE OBJECT WHICH HAS BEEN LOADED FROM FILE ----------------------------
  else
  {
    // id should be unique
    std::string object_id = request.object_id;

    if (request.object_id.empty())
      object_id = request.object_name;

    ROS_WARN("Hello");

    // erase that object from collision matrix list
    for (std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it = collision_matrix_.begin();
         it != collision_matrix_.end(); ++it)
    {
      //  just check required char inside the string, if yes than remove it
      if (it->first.find(request.file_name) !=std::string::npos)
      {
        collision_matrix_.erase(it);
      }
    }
    ROS_WARN("Hello");
    // remove that object form marker list, we can requst only one object to remove
    // Be careful, here we are erasing iteration so do not put increment of iteration in for loop
    for (auto it = marker_array_.markers.begin(); it != marker_array_.markers.end();)
    {
      // just check required char inside the string, if yes than remove it
      if (it->text.find(request.file_name) != std::string::npos)
      {
        it->action = visualization_msgs::Marker::DELETE;
        //it->ns = it->ns;

        // make sure first publish it and than remove from list to maintain list
        marker_pub_.publish(marker_array_);
        marker_array_.markers.erase(it);
      }

      // iteration reach to end, considering not found requested object
      else if (it == marker_array_.markers.end())
      {
        response.success = false;
        response.message = (" Not find requsted object into list ");
        return false;
      }

      else
      {
         ++it;
      }
    }
    ROS_WARN("Hello");
    // response
    response.success = true;
    response.message = ("Successfully remove to the environment");

    return true;

  }
}


bool StaticCollision::removeAllStaticObjectsServiceCB(predictive_control::StaticCollisionObjectRequest &request,
                                                      predictive_control::StaticCollisionObjectResponse &response)
{
  // remove all object from collision matrix list
  for (std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it = collision_matrix_.begin();
       it != collision_matrix_.end(); ++it)
  {
    collision_matrix_.erase(it);
  }

  // remove all object from environment
  for (auto it = marker_array_.markers.begin(); it != marker_array_.markers.end(); ++it)
  {
    it->action = visualization_msgs::Marker::DELETE;
    marker_pub_.publish(marker_array_);
    marker_array_.markers.erase(it);
  }

  response.success = true;
  response.message = "Successfully remove all objects from environment";
  return true;
}
// update collsion ball position, publish new position of collision ball
void StaticCollision::updateStaticCollisionVolume(const std::map<std::string, geometry_msgs::PoseStamped>& robot_critical_points)
{

  // DEBUG
  if (predictive_configuration::activate_output_)
  {
   ROS_WARN("########### Print ROBOT CRITICLE POINT MATRIX ############");
   for (auto it = robot_critical_points.begin(); it != robot_critical_points.end(); ++it)
   {
      std::cout<< it->first.c_str() <<": \n" << it->second << std::endl;
   }
  }

  // publish
  marker_pub_.publish(marker_array_);

  // compute collision cost vectors
  computeStaticCollisionCost(collision_matrix_, robot_critical_points, 0.10,
                       predictive_configuration::collision_weight_factor_); //predictive_configuration::minimum_collision_distance_

  // DEBUG
  if (true) //predictive_configuration::activate_output_
  {
    ROS_WARN("===== STATIC COLLISION COST VECTOR =====");
    std::cout << collision_cost_vector_.transpose() << std::endl;
  }

}

// create collision cost for static objects
void StaticCollision::generateStaticCollisionVolume()
{
  // position and orientation
  geometry_msgs::PoseStamped stamped;
  stamped.header.frame_id = predictive_configuration::chain_root_link_;
  stamped.header.stamp = ros::Time().now();

  //position of static collision object
  stamped.pose.position.x = 0.0;
  stamped.pose.position.y = 0.0;
  stamped.pose.position.z = 0.10;

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
  marker.scale.x = 1.30;
  marker.scale.y = 1.30;
  marker.scale.z = 0.10;

  // position into world
  marker.id = 0;
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

bool StaticCollision::getTransform(const std::string& from, const std::string& to, geometry_msgs::PoseStamped& stamped_pose)
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


// create static frame, just for visualization purpose
void StaticCollision::createStaticFrame(const geometry_msgs::PoseStamped &stamped,
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

void StaticCollision::computeStaticCollisionCost(const std::map<std::string, geometry_msgs::PoseStamped> static_collision_matrix,
                                                 const std::map<std::string, geometry_msgs::PoseStamped> robot_collision_matrix,
                                                 const double& collision_threshold_distance,
                                                 const double &weight_factor)

{
  collision_cost_vector_ = Eigen::VectorXd(robot_collision_matrix.size());
  collision_cost_vector_.resize(robot_collision_matrix.size());

  for (auto it: static_collision_matrix)
  {
    ROS_WARN_STREAM(it.first);
        ROS_WARN_STREAM(it.second);
  }


/*
  // iterate to one by one point in collision matrix
  int loop_counter = 0u;
  for (auto it_out = static_collision_matrix.begin(); it_out != static_collision_matrix.end(); ++it_out, ++loop_counter)
  {
    double dist = 0.0;
    for (auto it_in = robot_collision_matrix.begin(); it_in != robot_collision_matrix.end(); ++it_in)
    {
      // both string are not equal than execute if loop
      if (it_out->first.find(it_in->first) == std::string::npos)
      {
        // logistic cost function
        // Nonlinear Model Predictive Control for Multi-Micro Aerial Vehicle Robust Collision Avoidance
        // https://arxiv.org/pdf/1703.01164.pdf ... equation(10)
        ROS_ERROR(" '%s'  <---> '%s'", it_out->first.c_str(),it_in->first.c_str());
        //dist += exp( ((collision_min_distance) -
        //              (std::abs(getEuclideanDistance(it_out->second.pose, it_in->second.pose))) ) / weight_factor);
        dist += exp( ((collision_threshold_distance*collision_threshold_distance) -
                      (std::abs(CollisionRobot::getEuclideanDistance(it_out->second.pose, it_in->second.pose)) * std::abs(CollisionRobot::getEuclideanDistance(it_out->second.pose, it_in->second.pose))) ) / weight_factor);

        ROS_DEBUG_STREAM("Exponential term: "<<
                         exp(collision_threshold_distance - std::abs(CollisionRobot::getEuclideanDistance(it_out->second.pose, it_in->second.pose)) / weight_factor));
      }
    }
    //store cost of each point into vector
    collision_cost_vector_(loop_counter) = dist;
  }*/


  int loop_counter = 0u;
  for (auto it_out = robot_collision_matrix.begin(); it_out != robot_collision_matrix.end(); ++it_out, ++loop_counter)
  {
    double dist = 0.0;
    int static_object_counter = 0u;
    for (auto it_in = static_collision_matrix.begin(); it_in != static_collision_matrix.end(); ++it_in, ++static_object_counter)
    {

      // dimension should half of scale
      double dim_x = marker_array_.markers.at(static_object_counter).scale.x*0.5;
      double dim_y = marker_array_.markers.at(static_object_counter).scale.y*0.5;
      double dim_z = marker_array_.markers.at(static_object_counter).scale.z*0.5;

      // move to center of object
      geometry_msgs::Pose pose = it_in->second.pose;
      //pose.position.x += dim_x;
      //pose.position.y += dim_y;
      pose.position.z += dim_z;
      double v_x = (it_in->second.pose.position.x) + dim_x;
      double v_y = (it_in->second.pose.position.y) + dim_y;
      double v_z = (it_in->second.pose.position.z) + dim_z;
      ROS_WARN_STREAM("Z: "<<v_z);
      // distance threasold by considering dimensions
      double dist_threasold_x = 0.10+dim_x; //marker_array_.markers.at(static_object_counter).scale.x;
      double dist_threasold_y = 0.10+dim_y; //marker_array_.markers.at(static_object_counter).scale.y;
      double dist_threasold_z = 0.10+dim_z; //marker_array_.markers.at(static_object_counter).scale.z;

      // both string are not equal than execute if loop
      if (it_out->first.find(it_in->first) == std::string::npos)
      {
        // penlaty + relax barrier cost function
        /*dist += exp( ((dist_threasold_x*dist_threasold_x) -
                      ( (it_out->second.pose.position.x - v_x) *
                        (it_out->second.pose.position.x - v_x))) / weight_factor);*/

        /*dist += exp( ((dist_threasold_y*dist_threasold_y) -
                      ( (it_out->second.pose.position.x - v_y) *
                        (it_out->second.pose.position.x - v_y))) / weight_factor);*/

        /*dist += exp( ((dist_threasold_z*dist_threasold_z) -
                      ( (it_out->second.pose.position.x - v_z) *
                        (it_out->second.pose.position.x - v_z))) / weight_factor);*/

        dist = CollisionRobot::getEuclideanDistance(it_out->second.pose, pose); //it_in->second.pose
        ROS_INFO_STREAM("-------"<<dist);
        dist = exp(((0.10*0.10 - dist*dist) / weight_factor));
        ROS_WARN_STREAM(exp(((0.15*0.15 - dist*dist) / weight_factor)));


      }
    }
    //store cost of each point into vector
    collision_cost_vector_(loop_counter) = dist;
  }

  ROS_ERROR_STREAM("=============== STATIC COLLSION DISTANCE VECRTOR ================= ");
  std::cout << collision_cost_vector_.transpose()<< std::endl;

}
