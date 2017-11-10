
#include <ros/ros.h>
#include <predictive_control/kinematic_calculations.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinematic_test");
	ros::NodeHandle node_handler;


	if (node_handler.hasParam("/robot_description"))
	{
		Kinematic_calculations kin_solver;
		kin_solver.initialize();

		kin_solver.print_data_memebers();
		kin_solver.print_fk_and_jacobian_matrix();
		kin_solver.print_kdl_fk_and_jacobian_matrix();
	}
return 0;
}
