
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
