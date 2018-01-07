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
