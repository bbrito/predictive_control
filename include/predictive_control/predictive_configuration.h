
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
  bool initialize(const std::string& node_handle_name);

  /**
   * @brief updateConfiguration: update configuration parameter with new parameter
   * @param new_config: changed configuration parameter
   * @return true all parameter update successfully else false
   */
  bool updateConfiguration(const predictive_configuration& new_config);

  /** data member of class **/
  // DEBUG
  bool activate_output_;
  bool initialize_success_;

  // kinematic of robotics manipulator
  unsigned int degree_of_freedom_;

  // use for finding kinematic chain and urdf model
  std::string chain_base_link_;
  std::string chain_tip_link_;
  std::string chain_root_link_;
  std::string target_frame_;    //  Frame want to follow/track
  std::string tracking_frame_;  //  End effector of arm

  // limiting parameter, use to enforce joint to be in limit
  std::vector<std::string> joints_name_;
  std::vector<double> joints_min_limit_;
  std::vector<double> joints_max_limit_;

  // predictive control
  double clock_frequency_;  //hz clock Frequency

  // self collision distance
  double ball_radius_;

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
