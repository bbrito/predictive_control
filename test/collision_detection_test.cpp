
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
			joint_angles(0) = 1.57;	joint_angles(1) = 1.57;	joint_angles(2) = 1.57;
			joint_angles(3) = 1.57;	joint_angles(4) = 0.0;	joint_angles(5) = 0.0;
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


			/*
			// Check Inverse jacobian calculation using 2*2 Jacobian matrix
			Eigen::MatrixXd J_Inv_Mat_bySVD, J_Test(2,2);
			J_Test(0,0) = 1;
			J_Test(0,1) = 0;
			J_Test(1,0) = 2;
			J_Test(1,1) = 2;
			kin_solver.calculateInverseJacobianbySVD(J_Test, J_Inv_Mat_bySVD);
			std::cout<<"\033[95m"<<"Inverse Jacobian Matrix by using SVD: \n"	<<"\033[36;0m" << J_Inv_Mat_bySVD <<std::endl;

			// Check Inverse jacobian calculation using 2*2 Jacobian matrix
			Eigen::MatrixXd J_Inv_Mat_byDirect;
			kin_solver.calculateInverseJacobianbyDirect(J_Test, J_Inv_Mat_byDirect);
			std::cout<<"\033[95m"<<"Inverse Jacobian Matrix by using Direct: \n"	<<"\033[36;0m" << J_Inv_Mat_byDirect <<std::endl;

			kin_solver.printDataMembers();
			*/

			kin_solver.calculateForwardKinematicsUsingKDLSolver(joint_angles, FK_Matrix);

			std::cout<<"KDL FK_Matrix: \n"
					<<"\033[0;32m" << FK_Matrix	<<"\033[36;0m"<<std::endl;

			kin_solver.calculateJacobianMatrixUsingKDLSolver(joint_angles, FK_Matrix, Jacobian_Matrix);

			std::cout<<"KDL FK_Matrix: \n"
					<<"\033[0;32m" << FK_Matrix	<<"\033[36;0m"<<std::endl;

			std::cout<<"KDL Jacobian Matrix: \n"
					<<"\033[0;33m"<< Jacobian_Matrix <<"\033[36;0m" <<std::endl;

			//-----------------------------------------------------------------------------------------
			CollisionRobot collision_robot;
			collision_robot.initializeCollisionRobot();

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
