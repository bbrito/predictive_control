
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

		/*
		std::vector<double> lmt_vec;
		kin_solver.get_joint_limits("pose_max_limit", lmt_vec);

		for (auto it = lmt_vec.begin(); it != lmt_vec.end(); ++it)
		{
			std::cout << *it << std::endl;
		}
		*/

		KDL::JntArray jnt_angles = KDL::JntArray(7);
		jnt_angles(0) = 1.57;	jnt_angles(1) = 1.57;	jnt_angles(2) = 1.57;	jnt_angles(3) = 1.57;
		std::cout << kin_solver.get_jacobian(jnt_angles) << std::endl;
		//kin_solver.computeJacobian(jnt_angles);
		//kin_solver.kdl_computeJacobian(jnt_angles);

	}
return 0;
}
