
#include <ros/ros.h>
//#include <predictive_control/predictive_controller.h>

int main(int argc, char **argv)
{
	try
	{
		ros::init(argc, argv, ros::this_node::getName());
    //predictive_control_node pd_control_node;
    //pd_control_node.main_predictive_control();
		ros::spin();
	}

	catch (ros::Exception& e)
	{
		ROS_ERROR("Error occured: %s ", e.what());
		exit(1);
	}


}
