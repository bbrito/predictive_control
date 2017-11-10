
//This file containts read parameter from server, callback, call class objects, control all class

#include <predictive_control/predictive_controller.h>


predictive_config::predictive_config()
{
	nh =  ros::this_node::getName() ;

	// Chain param
	dof = 7;
	base_link = "arm_base_link";
	tip_link = "arm_7_link";
	root_frame = "arm_base_link";

	limits_tolerance = 10.0;

}

predictive_config::~predictive_config()
{
;
}


bool predictive_config::initializeParam()
{
	//Chain_base and chain tip links, root frame
	if (!nh.getParam ("chain_base_link", base_link) )
	{
		ROS_WARN(" Parameter 'chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("chain_tip_link", tip_link) )
	{
		ROS_WARN(" Parameter 'chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("root_frame", root_frame) )
	{
		ROS_WARN(" Parameter 'root_frame' not set on %s node " , ros::this_node::getName().c_str());
	}

	//Get joint names
	if (!nh.getParam ("joint_names", jnts_name) )
	{
		ROS_WARN(" Parameter 'joint names' not set on %s node " , ros::this_node::getName().c_str());
	}

	dof = jnts_name.size();

	return true;
}


