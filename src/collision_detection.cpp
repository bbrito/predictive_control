
#include <predictive_control/collision_detection.h>


SelfCollision::SelfCollision()
{
//  segments_ = 6;
//  degree_of_freedom_ = 6;
}

SelfCollision::~SelfCollision()
{
  //clearDataMember();
}

void SelfCollision::clearDataMember()
{
  // resize all data members
  axis_.resize(segments_);
  types_.resize(segments_);
  model_joint_names_.resize(segments_);
  model_link_names_.resize(segments_);
  chain_joint_names_.resize(degree_of_freedom_);
  Transformation_Matrix_.resize(segments_, Eigen::Matrix4d::Identity()); //used push back careful
  FK_Homogenous_Matrix_.resize(segments_, Eigen::Matrix4d::Identity());
  distance_vector_.resize(segments_, Eigen::VectorXd(7));
}

bool SelfCollision::initialize(const predictive_configuration& pd_config_param)
{
  ros::NodeHandle nh_selfCollision("predictive_control/selfCollision");

  if (!pd_config_param.initialize_success_)
  {
    ROS_ERROR("SelfCollision::initialize: Failed because of uninitialization of predictive configuration");
    return false;
  }

  // initialize predictive confisuration class
  pd_config_ = pd_config_param;

  // initilize robot model
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

  // construct chain using tree inforamtion. Note: make sure chain root link or chain base link
  tree.getChain( pd_config_.chain_base_link_, pd_config_.chain_tip_link_, chain); //
  if (chain.getNrOfJoints() == 0 || chain.getNrOfSegments() == 0)
  {
    ROS_ERROR("Failed to initialize kinematic chain");
    return false;
  }

  segments_ = tree.getNrOfSegments(); //chain_.getNrOfSegments();
  degree_of_freedom_ = chain.getNrOfSegments();

  // resize all data members
  axis_.resize(segments_);
  types_.resize(segments_);
  model_joint_names_.resize(segments_);
  model_link_names_.resize(segments_+1);  // number of links always 1 higer than joints
  chain_joint_names_.resize(degree_of_freedom_);
  Transformation_Matrix_.resize(segments_, Eigen::Matrix4d::Identity()); //used push back careful
  FK_Homogenous_Matrix_.resize(segments_, Eigen::Matrix4d::Identity());
  distance_vector_.resize(segments_, Eigen::VectorXd(7));

  int i = 0u;
  for (auto it = model_.links_.begin(); it != model_.links_.end(); ++it, ++i)
  {
    model_link_names_[i] = it->first;
  }

  ROS_WARN("======= MODEL LINK NAMES  ========== ");
  for (auto it = model_link_names_.begin(); it != model_link_names_.end(); ++it)
  {
     std::cout << *it << std::endl;
  }

  // intialize data member of class
  initializeDataMember(model_.joints_);

  marker_pub_ = nh_selfCollision.advertise<visualization_msgs::MarkerArray>("collision_ball", 1);
  ROS_INFO("===== Collision Ball marker published with topic: ~/predictive_control/selfCollision/collision_ball =====");

  // TEMPORARY SOLUTION TO CHANGE JOINT VALUES
  std::cout << "CONFIRM FOR START TIME EXECUTION: 'y' " << std::endl;
  char ch;
  std::cin >> ch;
  while (ch != 'y')
  {
    std::cout << "CONFIRM AGAIN FOR TIME EXECUTION: 'y' " << std::endl;
    std::cin >> ch;
  }

  ROS_WARN("SelfCollision::initialize: SUCCESSED!!!");
  return true;
}

// initilize data member of class
void SelfCollision::initializeDataMember(const std::map< std::string, boost::shared_ptr<urdf::Joint> >& joints)
{

  int i = 0u;
  //Eigen::MatrixXd transformation_matrix = Eigen::Matrix4d::Identity();

  // DEBUG
  if (pd_config_.activate_output_)
  {
    for (auto it = joints.begin(); it != joints.end(); ++it)
    {
      std::cout<<"\033[95m"<<"________________________"<< it->first <<"___________________"<<"\033[36;0m"<<std::endl;
    }
  }

  // joint name extract from chain model
  i = 0u;
  for (int i = 0u; i < degree_of_freedom_; ++i)
  {
    chain_joint_names_[i] = (chain.getSegment(i).getJoint().getName());
  }

  //mode joint information
  i = 0u;
  for (auto it = joints.begin(); it != joints.end(); ++it, ++i)
  {
    types_(i) = model_.getJoint(it->first)->type;
    model_joint_names_[i] = it->first;

    // set axis of joint rotation
    axis_[i](0) = model_.getJoint(it->first).get()->axis.x;
    axis_[i](1) = model_.getJoint(it->first).get()->axis.y;
    axis_[i](2) = model_.getJoint(it->first).get()->axis.z;

    // transformation matrix
    transformURDFToEigenMatrix( it->second->parent_to_joint_origin_transform , Transformation_Matrix_[i]);
    //Transformation_Matrix_.push_back(transformation_matrix);
  }

  // DEBUG
  if (pd_config_.activate_output_)
  {
    ROS_WARN("======= JOINT TYPE ============");
    std::cout << types_.transpose() << std::endl;


    ROS_WARN("======== MODEL JOINT NAMES ============");
    for (auto it = model_joint_names_.begin(); it != model_joint_names_.end(); ++it)
      ROS_INFO("%s", it->c_str());

    ROS_WARN("======== CHAIN JOINT NAMES ============");
    for (auto it = chain_joint_names_.begin(); it != chain_joint_names_.end(); ++it)
    ROS_INFO("%s", it->c_str());

    ROS_INFO("=========== JOINT AXIS ============");
    for (auto it = axis_.begin(); it != axis_.end(); ++it)
    {
      std::cout<<"\033[20m"<< it->transpose() <<"\033[36;0m"<<std::endl;
    }

    ROS_WARN("===== TRANSFORMATION MATRIX =====");
    int i = 0u;
    for (auto it = Transformation_Matrix_.begin(); it != Transformation_Matrix_.end(); ++it, ++i)
    {
      std::cout<< model_joint_names_.at(i) << "\n" << *it << std::endl;
      //std::cout << distance_vector_.at(i) << std::endl;
    }
  }

}

void SelfCollision::updateCollisionVolume(const Eigen::VectorXd& joints_angle)
{

  // first compute fk matrix than generate collision volume
  calculateForwardKinematics(joints_angle);

  // generate collision matrix and viualize bounding ball
  generateCollisionVolume(FK_Homogenous_Matrix_, Transformation_Matrix_);

  // DEBUG
  /*if (pd_config_.activate_output_)
  {
    ROS_WARN("========= COLLISION MATRIX ==========");
    for (int i = 0u; i < FK_Homogenous_Matrix_.size(); ++i)
    {
      ROS_WARN("------ FK_Matrix ------ %s ", model_link_names_.at(i).c_str());
      std::cout << FK_Homogenous_Matrix_.at(i) << std::endl;
    }
  }*/
/*
  // DEBUG
  if (pd_config_.activate_output_)
  {
    ROS_WARN("========= COLLISION MATRIX ==========");
    for (auto it = collision_matrix_.begin(); it != collision_matrix_.end(); ++it)
    {
      std::cout << *it << std::endl;
    }
  }*/

  // publish
  marker_pub_.publish(marker_array_);
}

// create collision detection, specifically center position of collision matrix
void SelfCollision::generateCollisionVolume(const std::vector<Eigen::MatrixXd>& FK_Homogenous_Matrix,
                                             const std::vector<Eigen::MatrixXd>& Transformation_Matrix)
{
  Eigen::Matrix4d matrix;
  Eigen::VectorXd bounding_vector(7);
  collision_matrix_.clear();

  // set ball radius
  Eigen::Vector3d ball_rad;
  ball_rad(0) = pd_config_.ball_radius_; // x
  ball_rad(1) = pd_config_.ball_radius_; //z
  ball_rad(2) = pd_config_.ball_radius_; //

  /*
  int id = 0u;
  // iterate all element of forward kinematic matrix
  for (auto  it_fk = FK_Homogenous_Matrix_.begin(); it_fk != FK_Homogenous_Matrix_.end(); it_fk++, id++)
  {
    // find index of joint name in model joint name
    //auto it = std::find(model_joint_names_.begin(), model_joint_names_.end(), chain_joint_names_.at(id));
    //auto index = std::distance(model_joint_names_.begin(), it);

    if (id < degree_of_freedom_)
    {
      auto index = computeIndexFromVector(model_joint_names_, chain_joint_names_.at(id));

      matrix = FK_Homogenous_Matrix[index];

      // generate collision matrix
      if (index != 0 && Transformation_Matrix[index](2,3) > pd_config_.ball_radius_)
      {
        matrix(0,3) = FK_Homogenous_Matrix[index-1](0,3)
                      + 0.5*(FK_Homogenous_Matrix[index](0,3) - FK_Homogenous_Matrix[index-1](0,3));//(Transformation_Matrix_[index](2,3) / 2.0);
        matrix(1,3) = FK_Homogenous_Matrix[index-1](1,3)
                      + 0.5*(FK_Homogenous_Matrix[index](1,3) - FK_Homogenous_Matrix[index-1](1,3));//(Transformation_Matrix_[index](2,3) / 2.0);
        matrix(2,3) = FK_Homogenous_Matrix[index-1](2,3)
                      + 0.5*(FK_Homogenous_Matrix[index](2,3) - FK_Homogenous_Matrix[index-1](2,3));//(Transformation_Matrix_[index](2,3) / 2.0);

        // filled collision matrix
        ROS_WARN("%s added to collision matrix", model_joint_names_.at(index).c_str());
        collision_matrix_.push_back(matrix);

        tranformEiegnMatrixToEigenVector(matrix, bounding_vector);
        if (pd_config_.activate_output_)
          createStaticFrame(bounding_vector, "point_" + std::to_string(id));

        // visualize bounding ball
        visualizeCollisionVolume(bounding_vector, ball_rad, pd_config_.chain_root_link_, id);
      }
    }
    else
    {
      // visualize remaning bounding ball which are not part of kinematic tree
      tranformEiegnMatrixToEigenVector(*it_fk, bounding_vector);
      visualizeCollisionVolume(bounding_vector, ball_rad, pd_config_.chain_root_link_, id);
    }
  }*/


  // visulaization of collision voulme
  for (int i = 0u; i < segments_; ++i)
  {

    /*auto it = std::find(chain_joint_names_.begin(), chain_joint_names_.end(), model_joint_names_.at(i));
    // found chain joint name in model joint name so skip below executution inorder to avoid redundancy
    if (it != chain_joint_names_.end())
    {
      continue;
    }*/

    matrix = FK_Homogenous_Matrix[i];

    // update collision bounding ball, segments_-1
    if (i != 0 && ( Transformation_Matrix[i](0,3) > pd_config_.ball_radius_) )
                          //|| Transformation_Matrix[i](1,3) > pd_config_.ball_radius_
                          //|| Transformation_Matrix[i](2,3) > pd_config_.ball_radius_) )
    {
/*      ball_rad(0) = 0.5*(Transformation_Matrix[i+1](0,3) - Transformation_Matrix[i](0,3));
      ball_rad(1) = 0.5*(Transformation_Matrix[i+1](1,3) - Transformation_Matrix[i](1,3));
      ball_rad(2) = 0.5*(Transformation_Matrix[i+1](2,3) - Transformation_Matrix[i](2,3));
*/
      matrix(0,3) = FK_Homogenous_Matrix[i-1](0,3)
                    + 0.5*(FK_Homogenous_Matrix[i](0,3) - FK_Homogenous_Matrix[i-1](0,3));//(Transformation_Matrix_[index](2,3) / 2.0);
      matrix(1,3) = FK_Homogenous_Matrix[i-1](1,3)
                    + 0.5*(FK_Homogenous_Matrix[i](1,3) - FK_Homogenous_Matrix[i-1](1,3));//(Transformation_Matrix_[index](2,3) / 2.0);
      matrix(2,3) = FK_Homogenous_Matrix[i-1](2,3)
                    + 0.5*(FK_Homogenous_Matrix[i](2,3) - FK_Homogenous_Matrix[i-1](2,3));//(Transformation_Matrix_[index](2,3) / 2.0);

      tranformEiegnMatrixToEigenVector(matrix, bounding_vector);
      if (pd_config_.activate_output_)
        createStaticFrame(bounding_vector, "point_" + std::to_string(i));
      collision_matrix_.push_back(matrix);
      visualizeCollisionVolume(bounding_vector, ball_rad, pd_config_.chain_root_link_,i);
    }
    else
    {
      tranformEiegnMatrixToEigenVector(matrix, bounding_vector);
      visualizeCollisionVolume(bounding_vector, ball_rad, pd_config_.chain_root_link_,i);
    }
  }

  ROS_INFO("======================");
  ROS_INFO("======================");
  ROS_INFO("======================");

  // generate collision matrix
  /*for (int i = 0u; i < degree_of_freedom_; ++i)
  {
    auto index = computeIndexFromVector(model_joint_names_, chain_joint_names_.at(i));
    matrix = FK_Homogenous_Matrix[index];

    // generate collision matrix
    if (index != 0 && Transformation_Matrix[index](2,3) > pd_config_.ball_radius_)
    {
      matrix(0,3) = FK_Homogenous_Matrix[index-1](0,3)
                    + 0.5*(FK_Homogenous_Matrix[index](0,3) - FK_Homogenous_Matrix[index-1](0,3));//(Transformation_Matrix_[index](2,3) / 2.0);
      matrix(1,3) = FK_Homogenous_Matrix[index-1](1,3)
                    + 0.5*(FK_Homogenous_Matrix[index](1,3) - FK_Homogenous_Matrix[index-1](1,3));//(Transformation_Matrix_[index](2,3) / 2.0);
      matrix(2,3) = FK_Homogenous_Matrix[index-1](2,3)
                    + 0.5*(FK_Homogenous_Matrix[index](2,3) - FK_Homogenous_Matrix[index-1](2,3));//(Transformation_Matrix_[index](2,3) / 2.0);

      // filled collision matrix
      ROS_WARN("%s added to collision matrix", model_joint_names_.at(index).c_str());
      collision_matrix_.push_back(matrix);
      ROS_WARN_STREAM("****************** "<<i << "****************** ");
      tranformEiegnMatrixToEigenVector(matrix, bounding_vector);
      if (pd_config_.activate_output_)
        createStaticFrame(bounding_vector, "point_" + std::to_string(i));
      visualizeCollisionVolume(bounding_vector, ball_rad, pd_config_.chain_root_link_,i);
    }
    else
    {
      // visualize remaning bounding ball which are not part of kinematic tree
      tranformEiegnMatrixToEigenVector(FK_Homogenous_Matrix[index], bounding_vector);
      visualizeCollisionVolume(bounding_vector, ball_rad, pd_config_.chain_root_link_, i);
    }
  }*/

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

void SelfCollision::visualizeCollisionVolume(const geometry_msgs::PoseStamped &center,
                                              const Eigen::Vector3d &radius,
                                             const uint32_t &ball_id)
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

// generate rotation matrix using joint angle and axis of rotation
/// Note: if angle value has less floating point accuracy than gives wrong answers like wrong 1.57, correct 1.57079632679.
void SelfCollision::generateTransformationMatrixFromJointValues(const unsigned int& current_segment_id, const double &joint_value, Eigen::MatrixXd &trans_matrix)
{
  trans_matrix = Eigen::Matrix4d::Identity();

  // check axis of ration about x-axis
  if (axis_.at(current_segment_id) == Eigen::Vector3i(1, 0, 0))
  {
    trans_matrix(1,1) = cos(joint_value);	trans_matrix(1,2) = -1*sin(joint_value);
    trans_matrix(2,1) = sin(joint_value);	trans_matrix(2,2) = cos(joint_value);
  }

  // check axis of ration about y-axis
  if (axis_.at(current_segment_id) == Eigen::Vector3i(0, 1, 0))
  {
    trans_matrix(0,0) = cos(joint_value);    trans_matrix(0,2) = sin(joint_value);
    trans_matrix(2,0) = -1*sin(joint_value); trans_matrix(2,2) = cos(joint_value);
  }

  // check axis of ration about z-axis
  if (axis_.at(current_segment_id) == Eigen::Vector3i(0, 0, 1))
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
void SelfCollision::calculateForwardKinematics(const Eigen::VectorXd& joints_angle)
{
  // initialize local member and parameters
  Eigen::MatrixXd till_joint_FK_Matrix = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd dummy_RotTrans_Matrix = Eigen::Matrix4d::Identity();

  ROS_INFO_STREAM("Forward Kinematics with joint values: ");
  std::cout << "[ " << joints_angle.transpose() << " ]" << std::endl;
  //ROS_INFO_STREAM(joints_angle.transpose());

  // compute tf transformation between root frame and base link of manipulator
  if (pd_config_.chain_root_link_ != pd_config_.chain_base_link_)
  {
    ROS_WARN("'%s' and '%s' are not same frame", pd_config_.chain_root_link_.c_str(), pd_config_.chain_base_link_.c_str());
    for (int index = 0u; index < segments_; ++index)
    {
      auto it = std::find(chain_joint_names_.begin(), chain_joint_names_.end(), model_joint_names_.at(index));
      // found chain joint name in model joint name so skip below executution inorder to avoid redundancy
      if (it != chain_joint_names_.end())
      {
        continue;
      }

      /*ROS_INFO("calculateForwardKinematics: Fixed Joint");
      till_joint_FK_Matrix = till_joint_FK_Matrix * Transformation_Matrix_[index];
      FK_Homogenous_Matrix_[index] = till_joint_FK_Matrix;*/

      getTransform(pd_config_.chain_root_link_, model_link_names_.at(index), FK_Homogenous_Matrix_[index]);
    }

    ROS_INFO("============================");
    till_joint_FK_Matrix = Eigen::Matrix4d::Identity();
    int index = computeIndexFromVector(model_link_names_, pd_config_.chain_root_link_);
    ROS_WARN_STREAM("*****------*******"<<index);
    till_joint_FK_Matrix = FK_Homogenous_Matrix_[index];
  }
   ROS_WARN("Calculating FK Matrix from '%s' to '%s '", pd_config_.chain_base_link_.c_str(), pd_config_.chain_tip_link_.c_str());
  // seraching for one joint at one time than start loop again for searching next joint
  //for (int j = 0u; j < degree_of_freedom_;)
  {
    // iterate to all joints
    for (int j = 0u, revolute_joint_number = 0u; j < degree_of_freedom_;) //segments_ (segments_- degree_of_freedom_)
    {
      //auto it = std::find(model_joint_names_.begin(), model_joint_names_.end(), chain_joint_names_.at(i));
      //auto index = std::distance(model_joint_names_.begin(), it);
      int index = computeIndexFromVector(model_joint_names_, chain_joint_names_.at(j));

      // revolute joints update
      if (types_(index) == REVOLUTE)
      {
        generateTransformationMatrixFromJointValues(revolute_joint_number, joints_angle(revolute_joint_number), dummy_RotTrans_Matrix);
        till_joint_FK_Matrix = till_joint_FK_Matrix * ( Transformation_Matrix_[index] * dummy_RotTrans_Matrix);
        FK_Homogenous_Matrix_[index] = till_joint_FK_Matrix;
        ++revolute_joint_number;
        ++j;
        //break;
      }

      // fixed joints update
      if (types_(index) == FIXED)
      {
        ROS_INFO("calculateForwardKinematics: Fixed Joint");
        till_joint_FK_Matrix = till_joint_FK_Matrix * Transformation_Matrix_[index];
        FK_Homogenous_Matrix_[index] = till_joint_FK_Matrix;
        ++j;
        //break;
      }
    }
  }

  // take last segment value that is FK Matrix
  if (pd_config_.activate_output_)
  {
    ROS_WARN("===== FORWARD KINEMATICS MATRIX =======");
    std::cout << FK_Homogenous_Matrix_[computeIndexFromVector(model_joint_names_,
                                                              chain_joint_names_.at(degree_of_freedom_-1))] << std::endl;
  }
}

bool SelfCollision::getTransform(const std::string& from, const std::string& to, Eigen::MatrixXd& matrix)
{
  bool transform = false;
  matrix = Eigen::Matrix4d::Identity();
  tf::StampedTransform stamped_tf;

  // make sure source and target frame exist
  if (tf_listener_.frameExists(to) & tf_listener_.frameExists(from))
  {
    try
    {
      // find transforamtion between souce and target frame
      tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.2));
      tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);

      // convert quternion to rpy
      geometry_msgs::Quaternion quat;
      quat.w = stamped_tf.getRotation().getW();
      quat.x = stamped_tf.getRotation().getX();
      quat.y = stamped_tf.getRotation().getY();
      quat.z = stamped_tf.getRotation().getZ();

      KDL::Rotation rot;
      tf::quaternionMsgToKDL(quat, rot);

      matrix(0,0) = rot(0,0);	  matrix(0,1) = rot(0,1);	  matrix(0,2) = rot(0,2);	  matrix(0,3) = stamped_tf.getOrigin().x();
      matrix(1,0) = rot(1,0);	  matrix(1,1) = rot(1,1);	  matrix(1,2) = rot(1,2);	  matrix(1,3) = stamped_tf.getOrigin().y();
      matrix(2,0) = rot(2,0);	  matrix(2,1) = rot(2,1);	  matrix(2,2) = rot(2,2);	  matrix(2,3) = stamped_tf.getOrigin().z();
      matrix(3,0) = 0;	  matrix(3,1) = 0;	  matrix(3,2) = 0;	  matrix(3,3) = 1;

      if (pd_config_.activate_output_)
      {
        std::cout << matrix << std::endl;
      }
      transform = true;
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("pd_frame_tracker::getTransform: %s", ex.what());
    }
  }

  else
  {
    ROS_WARN("pd_frame_tracker::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",
             from.c_str(), to.c_str());
  }

  return transform;
}


// create static frame, just for visualization purpose
void SelfCollision::createStaticFrame(const Eigen::VectorXd& vector,
                                       const std::string &frame_name)
{
  geometry_msgs::TransformStamped static_transformStamped;

  // frame information
  static_transformStamped.header.stamp = ros::Time(0).now();
  static_transformStamped.header.frame_id = pd_config_.chain_root_link_;
  static_transformStamped.child_frame_id = frame_name;

  // pose of frame relative to header frame_id
  static_transformStamped.transform.translation.x = vector(0);
  static_transformStamped.transform.translation.y = vector(1);
  static_transformStamped.transform.translation.z = vector(2);
  static_transformStamped.transform.rotation.w = vector(3);
  static_transformStamped.transform.rotation.x = vector(4);
  static_transformStamped.transform.rotation.y = vector(5);
  static_transformStamped.transform.rotation.z = vector(6);

  ROS_INFO("Created intermediate 'Static Frame' with '%s' parent frame id and '%s' child frame id",
           pd_config_.chain_root_link_.c_str(), frame_name.c_str()
           );

  static_broadcaster_.sendTransform(static_transformStamped);
  ros::spinOnce();
}

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
  matrix(2,0) = rot(2,0);	  matrix(2,1) = rot(2,1);	  matrix(2,2) = rot(2,2);	  matrix(2,3) = pose.position.z; //shift_dist;
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
