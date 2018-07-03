This repository contains a MPC regulator for Jackal.

# predictive_trajectory_generator.cpp
- Contains code to pregenerate MPC solver using ACADO

# predictive_configuration.cpp
- Contains code to MPC load parameters from predictive_config_parameter.yaml in the config directory

# mpcc_controller.cpp
- Contains the controller class that is launched by the predictive_control_node

# Singularity image installing all dependencies
singularity pull shub://bbrito/singularity_images:2.2

# Build SIngularity Image

# Run singularity image
sudo singularity shell --writable mpc.img

# Source ros instalation
source /opt/ros/kinetic/setup.bash 

# Install ACADO
cd /home/bdebrito_sing/ACADOtoolkit/build
make install

# Go to code folder
cd /home/bdebrito_sing/catkin_ws
catkin_make
source devel/setup.bash

# Running instructions
This contains the necessary information to get the MPC regulator for jackal working in simulation. Launching requires three or four steps. Launching the simulator in gazebo, launching the moveit move group, optionally launching the obstacle publisher and finally launching the mpc controller node. This can be done with to the following commands:

- roslaunch jackal_gazebo jackal_world.launch
- roslaunch jackal_moveit_description move_group.launch
- (roslaunch obstacle_feed obstacle_feed.launch)
- roslaunch predictive_control predictive_controller.launch

The move group will wait for the mpc controller to be launched, after which a rviz instance with a specific configuration will be launched. Both the obstacle_feed and predictive_controller node contain a configuration file in a config directory. Several parameters can be set in this file.

The MPC regulator is called with a reference trajectory using an action service. Currently, the MPC node only uses the end position as goal. MoveIt is used to interface with the action service and is set up such, that is sends a planned motion as reference in the action service call.

Withing the predictive controller node source files, mpcc_controller.cpp contains the controller class that is launched by predictive_control_node.cpp. This controller class iteratively calls the pregenerated mpc solver. Within the file predictive_trajectory_generator.cpp, the MPC controller is defined using ACADO, which is generated during compilation.

