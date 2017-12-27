
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>	//inverse of matrix
#include <predictive_control/kinematic_calculations.h>

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
			joint_angles(0) = 0.0;	joint_angles(1) = 0.0;	joint_angles(2) = 0.0;
			joint_angles(3) = 0.0;	joint_angles(4) = 0.0;	joint_angles(5) = 0.0;
			joint_angles(6) = 0.0; //1.57079632679
			//joint_angles.resize(7,0.0);
			//joint_angles.Constant(0.0);

			Eigen::MatrixXd FK_Matrix;
			kin_solver.calculateForwardKinematics(joint_angles, FK_Matrix);

			kin_solver.printDataMembers();

			ROS_INFO("Done");
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
