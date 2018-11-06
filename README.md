# Local Model Predictive Contouring Control for Dynamic Environments

This repository contains the code for the paper:

**<a href="https://arxiv.org/abs/1803.10892">Local Model Predictive Contouring Control for Dynamic Environments</a>**
<br>
<a href="">Boaz Floor</a>,
<a href="http://www.tudelft.nl/staff/bruno.debrito/">Bruno Brito</a>,
<a href="http://www.tudelft.nl/staff/L.Ferranti/">Laura Ferranti</a>,
<a href="http://www.tudelft.nl/staff/j.alonsomora/">Javier Alonso-Mora</a>
<br>
Submitted to [ICRA 2019].

We present a local motion planner, namely, a \acl{lmpcc} design, for an \acl{agv} (AGV) traversing a dynamic environment. Our design allows the AGV to execute reactive motion while tracking a global plan, thanks to local parametrization of the path. In addition, our framework allows for avoidance of static obstacles (given in an occupancy grid map) and moving obstacles represented by ellipses. Furthermore, we provide a new bound to correct the approximation of the Minkowski sum of an ellipsoid obstacle and the union of discs representation of the controlled vehicle to guarantee collision avoidance. We show that the general definition of the framework applies to both unicycle and bicycle kinematic models, commonly used to represent robots and autonomous cars, respectively. Simulation results for a car and experimental results with a mobile robot are presented.

The code was deployd and tested on a Jackal mobile robot.

<div align='center'>
<img src="images/paper.png"></img>
</div>

If you find this code useful in your research then please cite
```
@article{schwarting2017parallel,
  title={Parallel autonomy in automated vehicles: Safe motion generation with minimal intervention},
  author={Schwarting, Wilko and Alonso-Mora, Javier and Paull, Liam and Karaman, Sertac and Rus, Daniela},
  year={2017},
  publisher={Institute of Electrical and Electronics Engineers (IEEE)}
}
```

# Instalation instructions

# predictive_trajectory_generator.cpp
- Contains code to pregenerate MPC solver using ACADO

# predictive_configuration.cpp
- Contains code to MPC load parameters from predictive_config_parameter.yaml in the config directory

# mpcc_controller.cpp
- Contains the controller class that is launched by the predictive_control_node