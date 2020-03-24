
#ifndef LMPCC_LMPCC_H
#define LMPCC_LMPCC_H

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

// Visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// predicitve includes
#include <lmpcc/lmpcc_configuration.h>
#include <lmpcc/control_feedback.h>

// actions, srvs, msgs
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// lmpcc messages
#include <lmpcc_msgs/lmpcc_feedback.h>
#include <lmpcc_msgs/lmpcc_obstacle.h>
#include <lmpcc_msgs/lmpcc_obstacle_array.h>
#include <lmpcc_msgs/IntTrigger.h>

//Dynamic Reconfigure server
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <lmpcc/PredictiveControllerConfig.h>

//TF
#include <tf2_ros/transform_broadcaster.h>

//Joint states
#include <sensor_msgs/JointState.h>
//splines
#include <tkspline/spline.h>
#include <lmpcc/Clothoid.h>

#include <lmpcc/Control.h>
#include <lmpcc/LMPCCReset.h>
//reset msgs
#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <carla_msgs/CarlaEgoVehicleControl.h>

typedef double real_t;

//Whens earching for the closest point on the path, this variable indicates the distance that the algorithm searches behind the current spline point.
#define MAX_STEP_BACK_TOLERANCE 0.1f

class MPCC
{
    /** Managing execution of all classes of predictive control
     * - Handle static and dynamic collision avoidance
     */

public:

    //DYnamic reconfigure server
    boost::shared_ptr<dynamic_reconfigure::Server<lmpcc::PredictiveControllerConfig> > reconfigure_server_;
    boost::recursive_mutex reconfig_mutex_;
    void reconfigureCallback(lmpcc::PredictiveControllerConfig& config, uint32_t level);

    /**
     * @brief MPCC: Default constructor, allocate memory
     */
    MPCC()
    {

    };

    /**
     * @brief ~MPCC: Default distructor, free memory
     */
    ~MPCC();

    /**
     * @brief initialize: Initialize all helper class of predictive control and subscibe joint state and publish controlled joint velocity
     * @return: True with successfully initialize all classes else false
     */
    bool initialize();

    /**
     * @brief StateCallBack: Get current state of the robot
     * @param msg: Read data from mobile_robot_state_publisher_node default type:
     */
    void StateCallBack(const geometry_msgs::Pose::ConstPtr& msg);

    void VRefCallBack(const std_msgs::Float64::ConstPtr& msg);

    void ObstacleCallBack(const lmpcc_msgs::lmpcc_obstacle_array& received_obstacles);

    void PlanNetCallBack(const nav_msgs::Path& traj);

    /**
     * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
     * @param from: source frame from find transformation
     * @param to: target frame till find transformation
     * @param stamped_pose: Resultant poseStamed between source and target frame
     * @return: true if transform else false
     */
    bool getTransform(const std::string& from,
                      const std::string& to,
                      geometry_msgs::PoseStamped& stamped_pose
    );

    /**
     * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
     * @param from: source frame from find transformation
     * @param to: target frame till find transformation
     * @param stamped_pose: Resultant poseStamed between source and target frame
     * @return: true if transform else false
     */

    bool transformEigenToGeometryPose(const Eigen::VectorXd& eigen_vector, geometry_msgs::Pose& pose);


    /**
     * @brief transformStdVectorToEigenVector: tranform std vector to eigen vectors as std vectos are slow to random access
     * @param vector: std vectors want to tranfrom
     * @return Eigen vectors transform from std vectos
     */
    template<typename T>
    static inline Eigen::VectorXd transformStdVectorToEigenVector(const std::vector<T>& vector)
    {
        // resize eigen vector
        Eigen::VectorXd eigen_vector = Eigen::VectorXd(vector.size());

        // convert std to eigen vector
        for (uint32_t i = 0; i < vector.size(); ++i)
        {
            eigen_vector(i) = vector.at(i);
        }

        return eigen_vector;
    }

    /** public data member */
    //Service clients
    ros::ServiceClient reset_simulation_client_, reset_ekf_client_;

    // joint state subsciber to get current joint value
    ros::Subscriber robot_state_sub_;

    // waypoints subscriber
    ros::Subscriber waypoints_sub_,ped_stop_sub_,v_ref_sub_;

    // subscriber for obstacle feed
    ros::Subscriber obstacle_feed_sub_;

    ros::Subscriber plan_subs_;

    ros::ServiceServer reset_server_;

    // controlled joint velocity, should be control velocity of controller
    ros::Publisher controlled_velocity_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber  obstacles_state_sub_;

    ros::Publisher  joint_state_pub_;

    // publish trajectory
    ros::Publisher traj_pub_, pred_traj_pub_, pred_cmd_pub_,cost_pub_,robot_collision_space_pub_,brake_pub_, spline_traj_pub_, contour_error_pub_, feedback_pub_;
    //Predicted trajectory
    nav_msgs::Path pred_traj_;
    nav_msgs::Path traj_ref_;
    nav_msgs::Path pred_cmd_;
    nav_msgs::Path spline_traj_,spline_traj2_;
    int traj_i;
    //Controller options
    bool enable_output_;
    bool reset_world_;
    bool plan_;
    bool replan_;
    bool debug_;

    bool simulation_mode_;
    real_t te_;

    tf2_ros::TransformBroadcaster state_pub_,path_pose_pub_;
    std_msgs::Float64 cost_;
    std_msgs::Float64 brake_;
    double contour_error_;
    double lag_error_;

    //Spline trajectory generation
    tk::spline ref_path_x, ref_path_y;

    //MPCC Implementation
    std::vector<double> X_road, Y_road, Theta_road;
    double dist_spline_pts_;
    double total_length_;
    double path_length_;
    std::vector<double> ss,xx,yy;
    int n_clothoid,n_pts;
    //Search window parameters
    double window_size_;
    int n_search_points_;
    int exit_code_;
    bool goal_reached_;

    //reset simulation msg
    std_srvs::Empty reset_msg_;
    robot_localization::SetPose reset_pose_msg_;


private:

    ros::NodeHandle nh;

    tf::TransformListener tf_listener_;

    // Clock frequency
    double clock_frequency_;

    double r_discs_;
    Eigen::VectorXd x_discs_;

    // Timmer
    ros::Timer timer_;

    std::string target_frame_;

    // store pose value for visualize trajectory
    //geometry_msgs::PoseArray traj_pose_array_;
    visualization_msgs::MarkerArray traj_marker_array_;

    // Distance between traget frame and tracking frame relative to base link
    Eigen::VectorXd current_state_, last_state_;

    double slack_weight_;
    double repulsive_weight_;
    double Wcontour_;
    double Wlag_;
    double Kw_;
    double reference_velocity_;
    double speed_;
    double velocity_weight_;
    double waypoints_size_;
    double last_waypoints_size_;

    //TRajectory execution variables
    double next_point_dist, goal_dist, prev_point_dist,obstacle_distance;

    visualization_msgs::Marker ellips1;

    // Kinematic variables
    //To be done kinematic model car

    // Obstacles
    lmpcc_msgs::lmpcc_obstacle_array obstacles_;
    lmpcc_msgs::lmpcc_obstacle_array obstacles_init_;
    lmpcc_msgs::IntTrigger obstacle_trigger;

    // Current and last position and velocity from joint state callback
    //Eigen::VectorXd current_position_;
    Eigen::VectorXd last_position_;
    //Eigen::VectorXd current_velocity_;
    Eigen::VectorXd last_velocity_;

    // Type of variable used to publish joint velocity
    geometry_msgs::Twist controlled_velocity_;

    // predictive configuration
    boost::shared_ptr<predictive_configuration> controller_config_;

    void getWayPointsCallBack(nav_msgs::Path waypoints);

    void Plan(geometry_msgs::PoseWithCovarianceStamped msg);

    /**
     * @brief spinNode: spin node means ROS is still running
     */
    void spinNode();

    void computeEgoDiscs();
    /**
     * @brief runNode: Continue updating this function depend on clock frequency
     * @param event: Used for computation of duration of first and last event
     */
    void runNode(const ros::TimerEvent& event);

    void ControlLoop();

    /**
     * @brief publishZeroJointVelocity: published zero joint velocity is statisfied cartesian distance
     */
    void publishZeroJointVelocity();

    /**
     * @brief publishErrorPose: published error pose between traget pose and tracking frame, it should approach to zero
     */
    void publishErrorPose(const Eigen::VectorXd& error);


    void publishTrajectory(void);

    double quaternionToangle(geometry_msgs::Quaternion q);
    /**
     * @brief publishPredictedTrajectory: publish predicted trajectory
     */
    void publishPredictedTrajectory(void);

    void publishSplineTrajectory(void);

    void publishPredictedOutput(void);

    void publishPredictedCollisionSpace(void);

    void publishCost(void);

    void publishContourError(void);

    void broadcastTF();

    void broadcastPathPose();

    int spline_closest_point(int cur_traj_i, std::vector<double> ss_vec, double &s_guess, double window, int n_tries);

    inline void Ref_path(std::vector<double> x, std::vector<double> y, std::vector<double> theta);

    void ConstructRefPath();

    void publishFeedback(int& it, double& time, bool in_collision);
    
    void reset_solver();

    bool transformPose(const std::string& from, const std::string& to, geometry_msgs::Pose& pose);

    bool ResetCallBack(lmpcc::LMPCCReset::Request  &req, lmpcc::LMPCCReset::Response &res);

    void VReCallBack(const std_msgs::Float64::ConstPtr& msg);

    bool ResetSimulation();

};

#endif
