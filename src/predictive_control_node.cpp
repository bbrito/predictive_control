
#include <ros/ros.h>

int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, ros::this_node::getName());

	}

	catch (ros::Exception& e)
	{
		ROS_ERROR("Error occured: %s ", e.what());
		exit(1);
	}


}
