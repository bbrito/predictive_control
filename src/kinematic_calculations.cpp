
#include <predictive_control/kinematic_calculations.h>

bool Kinematic_calculations::initialize(const std::string& rbt_description, const std::string& base_link, const std::string& tip_link, const std::string& root_frame)
{
	//Initialize data member of class
	chain_base_link_= base_link;
	chain_tip_link_ = tip_link;
	root_frame_		= root_frame;

	//Tree from parameter server
	KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromParam("/robot_description", kdl_tree))
        ROS_ERROR("Failed to construct kdl tree");

    //Get kinematic chain
    kdl_tree.getChain( chain_base_link_, chain_tip_link_, kinematic_chain_ );

    segments_ = kinematic_chain_.getNrOfSegments();
    if (segments_ != 0)
    {
    	for (uint16_t i = 0; i < segments_; ++i)
    	{


    	}
    }
    else	//segment is zero means kinematic chain is not found
    {
		ROS_ERROR(" Kinematics_calculations::initialize ... Failed to initialize kinematic chain ");
		return false;
    }

}

