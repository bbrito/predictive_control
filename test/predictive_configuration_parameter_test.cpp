
#include <ros/ros.h>

#include <predictive_control/predictive_configuration.h>


int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, ros::this_node::getName());
		ros::NodeHandle node_handler;

		if (node_handler.hasParam("/robot_description"))
		{
			// Read and update data member of predicitve_config class.
			predictive_configuration config;
			if ( config.initialize("predictive_config") )
			{
				ROS_WARN(" -------- CONFIGURATION SUCCESSED!!! ------------- ");

				config.active_output_ = true;
				config.updateConfiguration(config);
			}

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
