
#ifndef PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
#define PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_

//ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>

//KDL kinematic
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class Kinematic_calculations
{
private:

	//---------------------------------- DATA MEMBER ---------------------------
	unsigned int segments_;					//< Number of segment in kinematic chain
	unsigned int dof_;						//< Number of joints/dof in kinematic chain

	std::string chain_base_link_;			//< Base link of kinematic chain, not neccessary same as root frame
	std::string chain_tip_link_;			//< Tip/End-effector link of kinematic chain
	std::string root_frame_;				//< Root frame of kinematic chain, first frame/segment of kinematic chain

	std::vector<KDL::Joint>	jnts_;			//< Joint information
	std::vector<KDL::Frame>	trans_mat_;		//<	Transformation matrix at each joint
	std::vector<KDL::Frame>	homo_mat_;		//<	Homo matrix at each joint,

	KDL::Chain	kinematic_chain_;			//< kinematic chain of given robot
	Eigen::Matrix<double, 6, 7> jac_mat_;	//< Jacobian Matrix

public:

	// Default constructor of class
	Kinematic_calculations():
		dof_(0),
		segments_(0),
		chain_base_link_("arm_1_link"),
		chain_tip_link_("arm_7_link"),
		root_frame_("world")
		{
			jac_mat_.setZero();
		};

	bool initialize(const std::string& rbt_description, const std::string& base_link, const std::string& tip_link, const std::string& root_frame);

};


#endif //PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
