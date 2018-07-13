
#ifndef PREDICTIVE_CONTROL_PREDICITVE_CONFIGURATION_H
#define PREDICTIVE_CONTROL_PREDICITVE_CONFIGURATION_H

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
  bool plotting_result_;
  bool activate_controller_node_output_;
  bool initialize_success_;
  bool set_velocity_constraints_;

	/** inputs and output topic definition **/
  std::string output_cmd;
	std::string robot_state_topic_;

  // kinematic of robotics manipulator
  unsigned int degree_of_freedom_;

  // use for finding kinematic chain and urdf model
  std::string robot_base_link_;
  std::string tracking_frame_;  //  End effector of arm
  std::string sub_ellipse_topic_;

  // limiting parameter, use to enforce joint to be in limit
  std::vector<std::string> collision_check_obstacles_;
  std::vector<double> vel_min_limit_;
  std::vector<double> vel_max_limit_;

  std::vector<double> lsq_state_weight_factors_;
  std::vector<double> lsq_state_terminal_weight_factors_;
  std::vector<double> lsq_control_weight_factors_;
  std::vector<double> lsq_control_terminal_weight_factors_;

  double slack_weight_;
  double repulsive_weight_;

  // predictive control
  double clock_frequency_;  //hz clock Frequency
  double sampling_time_;
  int state_dim_;
  int control_dim_;

  // self collision distance
  double ball_radius_;
  double minimum_collision_distance_;
  double collision_weight_factor_;

  // acado configuration
  bool use_lagrange_term_;
  bool use_LSQ_term_;
  bool use_mayer_term_;
  int max_num_iteration_;
  int discretization_intervals_;
  double kkt_tolerance_;
  double integrator_tolerance_;
  double start_time_horizon_;
  double end_time_horizon_;

  int n_obstacles_;
  int n_discs_;
  double ego_l_;
  double ego_w_;


private:

  /**
   * @brief free_allocated_memory: remove all allocated data just for memory management
   */
  void free_allocated_memory();

  /**
   * @brief print_configuration_parameter: debug purpose print set data member of this class
   */
  void print_configuration_parameter();

};

#endif
