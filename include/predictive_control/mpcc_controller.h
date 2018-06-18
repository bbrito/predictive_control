
#ifndef PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H
#define PREDICTIVE_CONTROL_PREDICTIVE_CONTROLLER_H

// ros includes
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>

// eigen includes
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

// std includes
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// predicitve includes
#include <predictive_control/predictive_configuration.h>
#include <predictive_control/kinematic_calculations.h>
#include <predictive_control/collision_detection.h>
#include <predictive_control/predictive_trajectory_generator.h>

// actions, srvs, msgs
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <predictive_control/moveAction.h>
#include <predictive_control/moveActionGoal.h>
#include <predictive_control/collision_avoidance.h>

// joint trajectory interface
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <predictive_control/trajAction.h>
#include <predictive_control/trajActionGoal.h>

#include <nav_msgs/Path.h>


/*
struct hold_pose
{
    bool hold_success_;
    Eigen::VectorXd pose_hold_vector_;
};*/

class MPCC
{
    /** Managing execution of all classes of predictive control
     * - Handle self collsion avoidance
     * - Extract current position and velocity of manipulator joints
     * - Publish controlled joint velocity
     */
        //Info: static member for transform std::vector to Eigen::vector

public:

    /**
     * @brief MPCC: Default constructor, allocate memory
     */
    MPCC();

    /**
     * @brief ~MPCC: Default distructor, free memory
     */
    ~MPCC();

    /**
     * @brief initialize: Initialize all helper class of predictive control and subscibe joint state and publish controlled joint velocity
     * @return: True with successuflly initialize all classes else false
     */
    bool initialize();

    /**
     * @brief StateCallBack: Get current state of the robot
     * @param msg: Read data from mobile_robot_state_publisher_node default type:
     */
    void StateCallBack(const geometry_msgs::Pose::ConstPtr& msg);

    /**
     * @brief controlSquence: Known as main control of all classes
     */
    void controlSquence(void);

    /**
     * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
     * @param from: source frame from find transformation
     * @param to: target frame till find transformation
     * @param stamped_pose: Resultant poseStamed between source and target frame
     * @return: true if transform else false
     */
    /*bool getTransform(const std::string& from,
                                        const std::string& to,
                                        geometry_msgs::PoseStamped& stamped_pose
                                        );*/

    /**
     * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
     * @param from: source frame from find transformation
     * @param to: target frame till find transformation
     * @param stamped_pose: Resultant poseStamed between source and target frame
     * @return: true if transform else false
     */
    bool getTransform(const std::string& from,
                                        const std::string& to,
                                        Eigen::VectorXd& stamped_pose
                                        );


    bool transformEigenToGeometryPose(const Eigen::VectorXd& eigen_vector, geometry_msgs::Pose& pose);

    /** public data member */
    // joint state subsciber to get current joint value
    ros::Subscriber robot_state_sub_;

    // controlled joint velocity, should be control velocity of controller
    ros::Publisher controlled_velocity_pub_;

    // publishes error vector between tracking and target frame
    ros::Publisher cartesian_error_pub_;

    // publish trajectory
    ros::Publisher traj_pub_, tr_path_pub_;

private:

    ros::NodeHandle nh;

    // DEBUG
    bool plotting_result_;

     tf::TransformListener tf_listener_;

    // Clock frequency
    double clock_frequency_;

    // Timmer
    ros::Timer timer_;

    // activate output of this node
    bool activate_debug_output_;
    // used to set desired position by mannually or using interactive marker node
    bool tracking_;
    std::string target_frame_;


    // store pose value for visualize trajectory
    //geometry_msgs::PoseArray traj_pose_array_;
        visualization_msgs::MarkerArray traj_marker_array_;

    // Distance between traget frame and tracking frame relative to base link
    Eigen::Vector3d current_state_, last_state_;
    Eigen::Vector3d goal_pose_, prev_pose_,next_pose_;
    Eigen::VectorXd tf_traget_from_tracking_vector_;

    Eigen::VectorXd min_velocity_limit_;
    Eigen::VectorXd max_velocity_limit_;

	//MoveIt TRAJECTORY VARIABLE
	moveit_msgs::RobotTrajectory traj;

	//TRajectory execution variables
	double next_point_dist, goal_dist, prev_point_dist;
	int idx, idy;
	double epsilon_;

    // Kinematic variables
	//To be done kinematic model car

    // Current and last position and velocity from joint state callback
    //Eigen::VectorXd current_position_;
    Eigen::VectorXd last_position_;
    //Eigen::VectorXd current_velocity_;
    Eigen::VectorXd last_velocity_;

    // Type of variable used to publish joint velocity
    geometry_msgs::Twist controlled_velocity_;

    // predictive configuration
    boost::shared_ptr<predictive_configuration> controller_config_;

    // kinematic calculation
    //TO BE REPLACED BY CAR CLASS CALCULATION
    //boost::shared_ptr<Kinematic_calculations> kinematic_solver_;

    // self collision detector/avoidance
    //boost::shared_ptr<CollisionRobot> collision_detect_;
    //boost::shared_ptr<CollisionAvoidance> collision_avoidance_;

    // static collision detector/avoidance
    //boost::shared_ptr<StaticCollision> static_collision_avoidance_;

    // predictive trajectory generator
    boost::shared_ptr<pd_frame_tracker> pd_trajectory_generator_;

    // move to goal position action
    boost::scoped_ptr<actionlib::SimpleActionServer<predictive_control::moveAction> > move_action_server_;

	boost::scoped_ptr<actionlib::SimpleActionServer<predictive_control::trajAction> > moveit_action_server_;

    /// Action interface
    predictive_control::moveResult move_action_result_;
    predictive_control::moveFeedback move_action_feedback_;
	predictive_control::trajActionFeedback moveit_action_feedback_;
	predictive_control::trajActionResult moveit_action_result_;
    void moveGoalCB();
    void movePreemptCB();
	void moveitGoalCB();
    void actionSuccess();
    void actionAbort();

    /**
     * @brief spinNode: spin node means ROS is still running
     */
    void spinNode();

    /**
     * @brief runNode: Continue updating this function depend on clock frequency
     * @param event: Used for computation of duration of first and last event
     */
    void runNode(const ros::TimerEvent& event);

    /**
     * @brief publishZeroJointVelocity: published zero joint velocity is statisfied cartesian distance
     */
    void publishZeroJointVelocity();

    /**
     * @brief publishErrorPose: published error pose between traget pose and tracking frame, it should approach to zero
     */
    void publishErrorPose(const Eigen::VectorXd& error);


    void publishTrajectory(void);
    void publishPathFromTrajectory(const moveit_msgs::RobotTrajectory& traj);
    /**
     * @brief checkVelocityLimitViolation: check velocity limit violate, limit containts lower and upper limit
     * @param joint_velocity: current velocity joint values
     * @param velocity_tolerance: tolerance in joint values after reaching minimum and maximum values
     * @return true with velocity limit violation else false
     */
    bool checkVelocityLimitViolation(const std_msgs::Float64MultiArray& joint_velocity,
                                                                     const double& velocity_tolerance = 0.0
                                                                    );
    void enforceVelocityInLimits(const std_msgs::Float64MultiArray& joint_velocity,
									     std_msgs::Float64MultiArray& enforced_joint_velocity);

    /**
     * @brief transformStdVectorToEigenVector: tranform std vector to eigen vectors as std vectos are slow to random access
     * @param vector: std vectors want to tranfrom
     * @return Eigen vectors transform from std vectos
     */
    /*template<typename T>
    static inline Eigen::VectorXd transformStdVectorToEigenVector(const std::vector<T>& vector)
    {
        // resize eigen vector
        Eigen::VectorXd eigen_vector = Eigen::VectorXd(vector.size());

        // convert std to eigen vector
        for (uint32_t i = 0; i < vector.size(); ++i)
        {
            std::cout << vector.at(i) << std::endl;
            eigen_vector(i) = vector.at(i);
     }

        return eigen_vector;
    }*/

    /**
     * @brief clearDataMember: clear vectors means free allocated memory
     */
    void clearDataMember();

	/**
     * @brief executeTrajectory: changes the goal state of the mpcc to each point of trajectory
     */
	void executeTrajectory(const moveit_msgs::RobotTrajectory & traj);
};

#endif
