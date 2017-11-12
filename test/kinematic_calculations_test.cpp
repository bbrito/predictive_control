
#include <ros/ros.h>

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

			// Print data member of kinematic solver class
			kin_solver.print_data_memebers();
			std::cout<<std::endl;

			KDL::JntArray jnt_angles = KDL::JntArray(7);
			jnt_angles(0) = 1.57;	jnt_angles(1) = 1.57;	jnt_angles(2) = 1.57;	jnt_angles(3) = 1.57;

			// Print fk and Jacobian matrix
			kin_solver.print_fk_and_jacobian_matrix(jnt_angles);
			std::cout<<std::endl;

			// Print fk and Jacobian matrix with KDL solver
			kin_solver.print_kdl_fk_and_jacobian_matrix(jnt_angles);
			std::cout<<std::endl;

			//Cross check get function values
			Eigen::MatrixXd J_Mat;
			kin_solver.compute_and_get_jacobian(jnt_angles, J_Mat);
			std::cout<<"\033[95m"<<"Compute and get Jacobian Matrix: \n"	<<"\033[36;0m" << J_Mat <<std::endl;

			// Check Inverse jacobian calculation using 2*2 Jacobian matrix
			Eigen::MatrixXd J_Inv_Mat_bySVD, J_Test(2,2);
			J_Test(0,0) = 1;
			J_Test(0,1) = 0;
			J_Test(1,0) = 2;
			J_Test(1,1) = 2;
			kin_solver.calculate_inverse_jacobian_bySVD(J_Test, J_Inv_Mat_bySVD);
			std::cout<<"\033[95m"<<"Inverse Jacobian Matrix by using SVD: \n"	<<"\033[36;0m" << J_Inv_Mat_bySVD <<std::endl;

			// Check Inverse jacobian calculation using 2*2 Jacobian matrix
			Eigen::MatrixXd J_Inv_Mat_byDirect;
			kin_solver.calculate_inverse_jacobian_bySVD(J_Test, J_Inv_Mat_byDirect);
			std::cout<<"\033[95m"<<"Inverse Jacobian Matrix by using Direct: \n"	<<"\033[36;0m" << J_Inv_Mat_byDirect <<std::endl;

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
