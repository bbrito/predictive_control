
#include <ros/ros.h>

#include <predictive_control/predictive_controller.h>

int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, ros::this_node::getName());
		ros::NodeHandle node_handler;

		if (node_handler.hasParam("/robot_description"))
		{
			predictive_control_node pd_control_node;
			pd_control_node.main_predictive_control();


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
