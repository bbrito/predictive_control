
#include <predictive_control/collision_detection.h>


SelfCollision::SelfCollision()
{
  ;
}

SelfCollision::~SelfCollision()
{
  ;
}

bool SelfCollision::initialize(const predictive_configuration& pd_config_param)
{
  if (!pd_config_param.initialize_success_)
  {
    ROS_ERROR("SelfCollision::initialize: Failed because of uninitialization of predictive configuration");
    return false;
  }

  pd_config_ = pd_config_param;


  if (!model_.initParam("/robot_description"))
  {
    ROS_ERROR(" SelfCollision::initialize: Failed to parse urdf file");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model_, tree)) //kdl_parser::treeFromParam("/robot_description", tree)) //
  {
     ROS_ERROR("SelfCollision::initialize: Failed to construct kdl tree");
     return false;
  }

  // get joint information from urdf model
  joints_ = model_.joints_;

  segments = tree.getNrOfSegments(); //chain_.getNrOfSegments();
  std::cout<<"\033[32m"<<"________________________"<< segments <<"___________________"<<"\033[36;0m"<<std::endl;
  distance_vector_.resize(segments, Eigen::VectorXd(7));

  initializeDataMember(model_);

  ROS_WARN("SelfCollision::initialize: SUCCESSED!!!");
  return true;
}

void SelfCollision::initializeDataMember(const urdf::Model& model)
{

  int i = 0u;

  // DEBUG
  if (pd_config_.activate_output_)
  {
    for (auto it = joints_.begin(); it != joints_.end(); ++it)
    {
      std::cout<<"\033[95m"<<"________________________"<< it->first <<"___________________"<<"\033[36;0m"<<std::endl;
    }
  }


  //joint type
  i = 0u;
  types_.resize(segments);
  for (auto it = joints_.begin(); it != joints_.end(); ++it, ++i)
  {
	  types_[i] = model.getJoint(it->first)->type;
  }

  // DEBUG
  /*if (pd_config_.activate_output_)
  {
	ROS_INFO("=========== JOINT TYPE ============");
    for (auto it = types_.begin(); it != types_.end(); ++it)
    {
      std::cout<<"\033[20m"<< it->transpose() <<"\033[36;0m"<<std::endl;
    }
  }*/

  // set axis of joint rotation
  i = 0u;
  axis.resize(segments);
  for (auto it = joints_.begin(); it != joints_.end(); ++it, ++i)
  {
    axis[i](0) = model.getJoint(it->first).get()->axis.x;
    axis[i](1) = model.getJoint(it->first).get()->axis.y;
    axis[i](2) = model.getJoint(it->first).get()->axis.z;
  }

  // DEBUG
  if (pd_config_.activate_output_)
  {
	ROS_INFO("=========== JOINT AXIS ============");
    for (auto it = axis.begin(); it != axis.end(); ++it)
    {
      std::cout<<"\033[20m"<< it->transpose() <<"\033[36;0m"<<std::endl;
    }
  }

  // get collision matrix reference to root link in other word, transformation matrix
  Eigen::MatrixXd transformation_matrix_lcl = Eigen::Matrix4d::Identity();
  for (auto it = joints_.begin(); it != joints_.end(); ++it)
  {
    // if the any of position is more than distance threasold than take into consideration
    if ( it->second->parent_to_joint_origin_transform.position.x > pd_config_.minimum_collision_distance_ ||
        it->second->parent_to_joint_origin_transform.position.y > pd_config_.minimum_collision_distance_ ||
        it->second->parent_to_joint_origin_transform.position.z > pd_config_.minimum_collision_distance_)
    {
      ROS_WARN("Add %s into collision matrix", it->first.c_str());
      transformURDFToEigenMatrix( it->second->parent_to_joint_origin_transform , transformation_matrix_lcl);
      Transformation_Matrix_.push_back(transformation_matrix_lcl);
    }
  }

    // DEBUG
  if (pd_config_.activate_output_)
  {
    ROS_WARN("===== TRANSFORMATION MATRIX =====");
    for (auto it = Transformation_Matrix_.begin(); it != Transformation_Matrix_.end(); ++it)
    {
      std::cout << *it << std::endl;
      //std::cout << distance_vector_.at(i) << std::endl;
    }
  }

}


// generate collision around robot body
void SelfCollision::visualizeCollisionVolume(const Eigen::VectorXd& center,
                                             const Eigen::VectorXd &radius,
                                             const std::string& header_frame_id,
                                             const uint32_t &ball_id
                                             )
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
  marker.scale.x = 2.0 * radius(0);
  marker.scale.y = 2.0 * radius(1);
  marker.scale.z = 2.0 * radius(2);

  // position into world
  marker.id = ball_id;
  marker.header.frame_id = header_frame_id;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.pose.orientation.w = center(3);
  marker.pose.orientation.x = center(4);
  marker.pose.orientation.y = center(5);
  marker.pose.orientation.z = center(6);

  // store created marker
  marker_array_.markers.push_back(marker);
}

/*
// generate rotation matrix using joint angle and axis of rotation
/// Note: if angle value has less floating point accuracy than gives wrong answers like wrong 1.57, correct 1.57079632679.
void SelfCollision::generateTransformationMatrixFromJointValues(const unsigned int& current_segment_id, const double &joint_value, Eigen::MatrixXd &trans_matrix)
{
  trans_matrix = Eigen::Matrix4d::Identity();

  // check axis of ration about x-axis
  if (axis.at(current_segment_id) == Eigen::Vector3i(1, 0, 0))
  {
    trans_matrix(1,1) = cos(joint_value);	trans_matrix(1,2) = -1*sin(joint_value);
    trans_matrix(2,1) = sin(joint_value);	trans_matrix(2,2) = cos(joint_value);
  }

  // check axis of ration about y-axis
  if (axis.at(current_segment_id) == Eigen::Vector3i(0, 1, 0))
  {
    trans_matrix(0,0) = cos(joint_value);    trans_matrix(0,2) = sin(joint_value);
    trans_matrix(2,0) = -1*sin(joint_value); trans_matrix(2,2) = cos(joint_value);
  }

  // check axis of ration about z-axis
  if (axis.at(current_segment_id) == Eigen::Vector3i(0, 0, 1))
  {
    trans_matrix(0,0) = cos(joint_value);	trans_matrix(0,1) = -1*sin(joint_value);
    trans_matrix(1,0) = sin(joint_value);	trans_matrix(1,1) = cos(joint_value);
  }

  else
  {
    ROS_ERROR("generateTransformationMatrixFromJointValues: Given rotation axis is wrong, check urdf files");
  }
}

// calculate end effector pose using joint angles
void SelfCollision::calculateForwardKinematics(const Eigen::VectorXd& joints_angle, Eigen::MatrixXd& FK_Matrix)
{
  // initialize local member and parameters
  FK_Matrix = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd till_joint_FK_Matrix = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd dummy_RotTrans_Matrix = Eigen::Matrix4d::Identity();

  ROS_INFO_STREAM("Forward Kinematics with joint values: ");
  std::cout << "[ " << joints_angle.transpose() << " ]" << std::endl;
  //ROS_INFO_STREAM(joints_angle.transpose());

  // compute tf transformation between root frame and base link of manipulator
  if (predictive_configuration::chain_root_link_ != predictive_configuration::chain_base_link_)
  {
    ROS_WARN("'%s' and '%s' are not same frame", pd_config_.chain_root_link_.c_str(), pd_config_.chain_base_link_.c_str());
  }

  // segments - degree_of_freedom_ gives information about fixed frame
  for (int i = 0u, revolute_joint_number = 0u; i < segments; ++i) //(segments_- degree_of_freedom_)
  {
    // revolute joints update
    if (types_(i) == 0)
    {
      generateTransformationMatrixFromJointValues(revolute_joint_number, joints_angle(revolute_joint_number), dummy_RotTrans_Matrix);
      till_joint_FK_Matrix = till_joint_FK_Matrix * ( Transformation_Matrix_[i] * dummy_RotTrans_Matrix);
      FK_Homogenous_Matrix_[i] = till_joint_FK_Matrix;
      ++revolute_joint_number;
    }

    // fixed joints update
    if (chain.getSegment(i).getJoint().getType() == 8)
    {
      ROS_INFO("calculateForwardKinematics: Fixed Joint");
      till_joint_FK_Matrix = till_joint_FK_Matrix * Transformation_Matrix_[i];
      FK_Homogenous_Matrix_[i] = till_joint_FK_Matrix;
    }

    //////
    else  // presmatic joint
    {
      std::cout<<"\033[36;1m" << chain.getSegment(i).getName() <<"\033[36;0m"<<std::endl;
      till_joint_FK_Matrix = till_joint_FK_Matrix * Transformation_Matrix_[i];
      FK_Homogenous_Matrix_[i] = till_joint_FK_Matrix;
    }////
  }

  // take last segment value that is FK Matrix
  FK_Matrix = FK_Homogenous_Matrix_[segments_-1];

  if (predictive_configuration::activate_output_)
  {
    ROS_WARN("===== FORWARD KINEMATICS MATRIX =======");
    std::cout << FK_Matrix << std::endl;
  }
}
*/


//convert KDL to Eigen matrix
void SelfCollision::transformURDFToEigenMatrix(const urdf::Pose &pose, Eigen::MatrixXd& matrix)
{
  // 3 position and 3 rotation called rpy
  matrix = Eigen::Matrix4d::Identity();

  double shift_dist = pose.position.z / 2.0;
  ball_major_axis_.push_back(shift_dist);

  //tf::Quaternion quat;
  geometry_msgs::Quaternion quat;
  quat.w = pose.rotation.w;
  quat.x = pose.rotation.x;
  quat.y = pose.rotation.y;
  quat.z = pose.rotation.z;

  //tf::Matrix3x3 mat(quat);
  KDL::Rotation rot;
  tf::quaternionMsgToKDL(quat, rot);

  matrix(0,0) = rot(0,0);	  matrix(0,1) = rot(0,1);	  matrix(0,2) = rot(0,2);	  matrix(0,3) = pose.position.x;
  matrix(1,0) = rot(1,0);	  matrix(1,1) = rot(1,1);	  matrix(1,2) = rot(1,2);	  matrix(1,3) = pose.position.y;
  matrix(2,0) = rot(2,0);	  matrix(2,1) = rot(2,1);	  matrix(2,2) = rot(2,2);	  matrix(2,3) = shift_dist; //pose.position.z;
  matrix(3,0) = 0;	  matrix(3,1) = 0;	  matrix(3,2) = 0;	  matrix(3,3) = 1;

  if (pd_config_.activate_output_)
  {
	  std::cout << matrix << std::endl;
  }

}

//convert Eigen matrix to KDL::Frame
void SelfCollision::transformEigenMatrixToKDL(const Eigen::MatrixXd& matrix, KDL::Frame& frame)
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

//---------------------------------------------------------------------------------------------------------------------
//------------------------------------------------ Collision Robot ----------------------------------------------------

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

  ROS_INFO("===== Collision Ball marker published with topic: ~/predictive_control/collisionRobot/static_collision_object =====");
  ROS_WARN("STATICCOLLISION INITIALIZED!!");

  // generate static collision volume
  generateStaticCollisionVolume();


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
  collision_cost_vector_ = Eigen::VectorXd(static_collision_matrix.size());
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
  for (auto it_out = static_collision_matrix.begin(); it_out != static_collision_matrix.end(); ++it_out, ++loop_counter)
  {
    double dist = 0.0;
    for (auto it_in = robot_collision_matrix.begin(); it_in != robot_collision_matrix.end(); ++it_in)
    {
      // both string are not equal than execute if loop
      if (it_out->first.find(it_in->first) == std::string::npos)
      {
        // penlaty + relax barrier cost function
        dist += exp( ((collision_threshold_distance*collision_threshold_distance) - ( fabs(it_in->second.pose.position.x - ( 1.30 + 0.0)) * fabs(it_in->second.pose.position.x - ( 1.30 + 0.0)))) / 0.001);
        dist += exp( ((collision_threshold_distance*collision_threshold_distance) - ( fabs(it_in->second.pose.position.y - ( 1.30 + 0.0)) * fabs(it_in->second.pose.position.y - ( 1.30 + 0.0)))) / 0.001);
        dist += exp( ((collision_threshold_distance*collision_threshold_distance) - ( fabs(it_in->second.pose.position.z - ( 0.10 + 0.10)) * fabs(it_in->second.pose.position.z - ( 0.10 + 0.10)))) / 0.001);

/*        // log function
        dist += -0.01* log( collision_threshold_distance - fabs(it_in->second.pose.position.x - ( 1.30 + 0.0)));
        double test = collision_threshold_distance - (fabs(it_in->second.pose.position.x - ( 1.30 + 0.0)) );
        std::cout << " Log x function: " << test << std::endl;
        std::cout << " Log x function: " << log(test) << std::endl;
        dist += -0.01* log( collision_threshold_distance - fabs(it_in->second.pose.position.y - ( 1.30 + 0.0)));
        std::cout << " Log y function: " << log( collision_threshold_distance) << std::endl;;
        dist += -0.01* log( collision_threshold_distance - fabs(it_in->second.pose.position.z - ( 0.10 + 0.10)));
        std::cout << " Log z function: " << log( collision_threshold_distance) << std::endl;*/
      }
    }
    //store cost of each point into vector
    collision_cost_vector_(loop_counter) = dist;
  }

  ROS_ERROR_STREAM("=============== STATIC COLLSION DISTANCE VECRTOR ================= ");
  std::cout << collision_cost_vector_.transpose()<< std::endl;

}
