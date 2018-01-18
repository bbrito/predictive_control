
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>	//inverse of matrix
#include <predictive_control/kinematic_calculations.h>
#include <predictive_control/collision_detection.h>

int main(int argc, char **argv)
{
try
{
  ros::init(argc, argv, "kinematic_test");
  ros::NodeHandle node_handler;

  if (node_handler.hasParam("/robot_description"))
  {
    Kinematic_calculations kin_solver;
    kin_solver.initialize();

    Eigen::VectorXd joint_angles(7);
    joint_angles(0) = 0;	joint_angles(1) = 0.00;	joint_angles(2) = 0.0;
    joint_angles(3) = 0.0;	joint_angles(4) = 0.0;	joint_angles(5) = 0.0;
    joint_angles(6) = 0.0; //1.57079632679
    //joint_angles.resize(7,0.0);
    //joint_angles.Constant(0.0);

    Eigen::MatrixXd FK_Matrix;
    Eigen::MatrixXd Jacobian_Matrix;
    //kin_solver.calculateForwardKinematics(joint_angles, FK_Matrix);
    kin_solver.calculateJacobianMatrix(joint_angles, FK_Matrix, Jacobian_Matrix);

    std::cout<<"FK_Matrix: \n"
        <<"\033[0;32m" << FK_Matrix	<<"\033[36;0m"<<std::endl;

    std::cout<<"Jacobian Matrix: \n"
        <<"\033[0;33m"<< Jacobian_Matrix <<"\033[36;0m" <<std::endl;


    //----------------------------------------------------------------------------------------
    predictive_configuration pd_config;
    pd_config.initialize();
    pd_config.activate_output_ = true;

    SelfCollision self_collision_check_;
    self_collision_check_.initialize(pd_config);

    ros::Duration(1.0).sleep();

    int i = 0;
    while(i<1)
    {
    	self_collision_check_.updateCollisionVolume(joint_angles);
    	i++;
    }
    //-----------------------------------------------------------------------------------------
    /*CollisionRobot collision_robot;
    collision_robot.initializeCollisionRobot();
    collision_robot.updateCollisionVolume(kin_solver.FK_Homogenous_Matrix_,
                         kin_solver.Transformation_Matrix_);*/

    ros::spin();

  }

  else
  {
    ROS_ERROR("Robot_description not available");
    exit(1);
  }

} catch (ros::Exception& e)

{
  ROS_ERROR("%s", e.what());
  exit(1);
}

return 0;
}
