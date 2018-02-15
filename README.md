# predictive_control
- Reactive motion planning using Model predictive control(MPC)

# Kinematic_calculation:
- Impliment compute_mass_matrix, compute_initeria_matrix. 
- improve code structure but not priority.
- Add function for calculating Inverse Kinematics using KDL

# predicitve_config:
- Change to read data from yaml to Dynamic config. 

# install LaTex
sudo apt-get install texlive-full

# Compile ACADO Toolkit
- cmake ..
- make
- lsq_term.cpp for teminal cost
- getting_started.cpp for self collision avoidance 

# Extension to robot body
- cob_robot: mt_experiment
- predictive_control: current
- Note: careful with yaml file becuse base and tip link match to cartesian_control.yaml
Therefore, it gives error about Filed kinematic chain ....

Launch:
 roslaunch predictive_control mpc_experiment_cob.launch  

MIN_DISTANCE: for obstracle avoidance is 0.50 define in obstacle_distance_data_types.hpp

# Experiment with Shunk arm
- Start power on butten with arm connection
- Connect can device to the computer and type set bund rate by using sudo ip link set can0 up type can bitrate 500000
- roslauch predictive_control robot_bringup.launch
- second terminal => rosservice call /arm/driver/init
                  => try: rosservice call /arm/driver/init .... it gives true
- roslauch predictive_control cartesian_controller.launch                  
