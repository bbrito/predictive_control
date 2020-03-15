#ifndef PREDICTIVE_CONFIGURATION_H
#define PREDICTIVE_CONFIGURATION_H

// ros includes
#include<ros/ros.h>

//c++ includes
#include<iostream>
#include<string>
#include<vector>
#include <algorithm>
#include <iomanip>	//print false or true
#include <math.h>

class predictive_configuration
{
    /**
     *  @brief All neccessary configuration parameter of predictive control repository
     *         Read data from parameter server
     *         Updated old data with new data
     *  Note:  All data member name used like xyz_ and all parameter name is normal like xyz.
     */

public:

    /** function member of class **/

    // constructor and distructor
    /**
     * @brief predictive_configuration: defualt constructor of this class
     */
    predictive_configuration();

    /**
      * @brief ~predictive_configuration: defualt distructor of this class
      */
    ~predictive_configuration();

    /**
     * @brief intialize:  check parameter on paramter server and read from there
     * @param node_handle_name: node handler initialize from name, as parameter set inside that name
     * @return true all parameter initialize successfully else false
     */
    bool initialize();  //const std::string& node_handle_name

    /**
     * @brief updateConfiguration: update configuration parameter with new parameter
     * @param new_config: changed configuration parameter
     * @return true all parameter update successfully else false
     */
    bool updateConfiguration(const predictive_configuration& new_config);

    /** data member of class **/
    // DEBUG
    bool activate_debug_output_;
    bool activate_controller_node_output_;
    bool initialize_success_;
    bool sync_mode_;
    bool gazebo_simulation_,simulation_mode_;

    /** inputs and output topic definition **/
    std::string cmd_, cmd_sim_;
    std::string robot_state_topic_, reset_topic_;

    // use for finding kinematic chain and urdf model
    std::string robot_base_link_;
    std::string global_path_frame_;  //  End effector of arm
    std::string target_frame_;
    std::string sub_ellipse_topic_;
    std::string obs_state_topic_;
    std::string waypoint_topic_;
    std::string vref_topic_;

    // limiting parameter, use to enforce joint to be in limit
    std::vector<std::string> collision_check_obstacles_;

    // Initialize vectors for reference path points
    std::vector<double> ref_x_;
    std::vector<double> ref_y_;
    std::vector<double> ref_theta_;

    // Numbers of points for spline and clothoid fitting
    int n_points_clothoid_;
    int n_points_spline_;

    double slack_weight_;
    double repulsive_weight_;
    double reference_velocity_;
    double ini_vel_x_;
    // predictive control
    double clock_frequency_;  //hz clock Frequency

    int max_num_iteration_;

    int n_obstacles_;
    int n_discs_;
    double ego_l_;
    double ego_w_;

private:


};

#endif
