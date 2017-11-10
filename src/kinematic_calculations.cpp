
#include <predictive_control/kinematic_calculations.h>


Kinematic_calculations::Kinematic_calculations()
{
	dof = 7;
	segments = 7;
	base_link = "arm_1_link";
	tip_link = "arm_7_link";
	root_frame = "world";

	clear_data_member();
}

Kinematic_calculations::~Kinematic_calculations()
{
	clear_data_member();
}


void Kinematic_calculations::clear_data_member()
{
	jnts_name.clear();
	frame_names.clear();

	jnt_pos_min_limit.clear();
	jnt_pos_max_limit.clear();
	jnt_vel_limit.clear();

	jnt_rot_axis.clear();

	jnts.clear();
	frames.clear();
	jnt_homo_mat.clear();
	jnt_fk_mat.clear();
}

bool Kinematic_calculations::initialize(const std::string rbt_description_param, const std::string& base_link_param, const std::string& tip_link_param, const std::string& root_frame_param)
{
	base_link = base_link_param;
	tip_link = tip_link_param;
	root_frame = root_frame_param;

	// Generate KDL chain and urdf model by parse robot description
	KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromParam("/robot_description", kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    kdl_tree.getChain( base_link, tip_link, chain );
    if ( chain.getNrOfJoints() == 0 || chain.getNrOfSegments() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    dof = chain.getNrOfJoints();
    segments = chain.getNrOfSegments();

    // Set joint position and velocity limits
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
        return false;
    }

    // Set frame Name, joint names
    frame_names.clear();
    jnts_name.clear();
    for (uint16_t i = 0; i < segments; ++i)
    {
    	frame_names.push_back(chain.getSegment(i).getName());
    	jnts_name.push_back(chain.getSegment(i).getJoint().getName());
    }

    // Set joint axis, angle, Create transformation matrix
    for (uint16_t i = 0; i < segments; ++i)
    {
    	jnts.push_back( this->chain.getSegment(i).getJoint());
    	frames.push_back( this->chain.getSegment(i).getFrameToTip() );

    	// Joint angle
    	double roll,pitch,yaw;
    	chain.getSegment(i).getFrameToTip().M.GetRPY(roll,pitch,yaw);

    	// Axis of rotation
    	KDL::Vector rot;
    	chain.getSegment(i).getFrameToTip().M.GetRotAngle(rot);
    	jnt_rot_axis.push_back(rot);

    	jnt_homo_mat.push_back(this->frames.at(i));
    	if (jnts.at(i).getType() == 0)	//revolute joint
    		create_transformation_matrix( i, roll, pitch, yaw );

    }

    // Get joint position and velocity limit
    for (uint16_t i = 0; i < dof ; ++i)
    {
    	jnt_pos_min_limit.push_back(model.getJoint( jnts_name[i])->limits->lower );
    	jnt_pos_max_limit.push_back(model.getJoint( jnts_name[i])->limits->upper );
    	jnt_vel_limit.push_back(model.getJoint ( jnts_name[i])->limits->velocity );
    }

    return true;
}

/*
void Kinematic_calculations::convert_kdl_vec_to_Eigen_vec(const KDL::Vector& kdl_vec, Eigen::VectorXd& egn_vec)
{
	egn_vec.Constant(0.0);

	for (uint16_t i = 0; i < 3; ++i)
	{
		egn_vec(i) = kdl_vec.data[i];
	}

}
*/


void Kinematic_calculations::create_transformation_matrix(const uint16_t& segment_number, const double& roll,const double& pitch, const double& yaw)

{

	//x-axis rotation
	if ( (this->jnt_rot_axis.at(segment_number).x() == 1 || this->jnt_rot_axis.at(segment_number).x() == -1) && this->jnt_rot_axis.at(segment_number).y() == 0 && this->jnt_rot_axis.at(segment_number).z() == 0 )
	{
		double angle = roll;

		if (_DEBUG_)
			std::cout<<"rot about x-axis with angel: "<< angle<<std::endl;

		this->jnt_homo_mat.at(segment_number).M(0,0) = 1;	this->jnt_homo_mat.at(segment_number).M(0,1) = 0;			this->jnt_homo_mat.at(segment_number).M(0,2) = 0;
		this->jnt_homo_mat.at(segment_number).M(1,0) = 0;	this->jnt_homo_mat.at(segment_number).M(1,1) = cos(angle);	this->jnt_homo_mat.at(segment_number).M(1,2) = -1*sin(angle);
		this->jnt_homo_mat.at(segment_number).M(2,0) = 0;	this->jnt_homo_mat.at(segment_number).M(2,1) = sin(angle);	this->jnt_homo_mat.at(segment_number).M(2,2) = cos(angle);

	}

	if ( this->jnt_rot_axis.at(segment_number).x() == 0 && (this->jnt_rot_axis.at(segment_number).y() == 1 || this->jnt_rot_axis.at(segment_number).y() == -1) && this->jnt_rot_axis.at(segment_number).z() == 0 )
	{
		double angle = pitch;

		if (_DEBUG_)
			std::cout<<"rot about y-axis with angel: "<< angle<<std::endl;

		this->jnt_homo_mat.at(segment_number).M(0,0) = cos(angle);		this->jnt_homo_mat.at(segment_number).M(0,1) = 0;			this->jnt_homo_mat.at(segment_number).M(0,2) = sin(angle);
		this->jnt_homo_mat.at(segment_number).M(1,0) = 0;				this->jnt_homo_mat.at(segment_number).M(1,1) = 1;			this->jnt_homo_mat.at(segment_number).M(1,2) = 0;
		this->jnt_homo_mat.at(segment_number).M(2,0) = -1*sin(angle);	this->jnt_homo_mat.at(segment_number).M(2,1) = 0;			this->jnt_homo_mat.at(segment_number).M(2,2) = cos(angle);
	}

	if ( this->jnt_rot_axis.at(segment_number).x() == 0 && this->jnt_rot_axis.at(segment_number).y() == 0 && (this->jnt_rot_axis.at(segment_number).z() == 1 || this->jnt_rot_axis.at(segment_number).z() == -1) )
	{

		double angle = yaw;

		if (_DEBUG_)
			std::cout<<"rot about z-axis with angel: "<< angle<<std::endl;

		this->jnt_homo_mat.at(segment_number).M(0,0) = cos(angle);	this->jnt_homo_mat.at(segment_number).M(0,1) = -1*sin(angle);	this->jnt_homo_mat.at(segment_number).M(0,2) = 0;
		this->jnt_homo_mat.at(segment_number).M(1,0) = sin(angle);	this->jnt_homo_mat.at(segment_number).M(1,1) = cos(angle);	this->jnt_homo_mat.at(segment_number).M(1,2) = 0;
		this->jnt_homo_mat.at(segment_number).M(2,0) = 0;			this->jnt_homo_mat.at(segment_number).M(2,1) = 0;			this->jnt_homo_mat.at(segment_number).M(2,2) = 1;
	}
}

void Kinematic_calculations::createRoatationMatrix(const double& angle, const std::vector<unsigned int>& rot_axis, KDL::Frame& lcl_homo_mat)
{

	//x-axis rotation
	if ( (rot_axis[0] == 1 || rot_axis[0] == -1) && rot_axis[1] == 0 && rot_axis[2] == 0 )
	{

		if (_DEBUG_)
			std::cout<<"rot about x-axis with angel: "<< angle<<std::endl;

		lcl_homo_mat.M(0,0) = 1;	lcl_homo_mat.M(0,1) = 0;			lcl_homo_mat.M(0,2) = 0;
		lcl_homo_mat.M(1,0) = 0;	lcl_homo_mat.M(1,1) = cos(angle);	lcl_homo_mat.M(1,2) = -1*sin(angle);
		lcl_homo_mat.M(2,0) = 0;	lcl_homo_mat.M(2,1) = sin(angle);	lcl_homo_mat.M(2,2) = cos(angle);

	}

	if ( rot_axis[0] == 0 && (rot_axis[1] == 1 || rot_axis[1] == -1) && rot_axis[2] == 0 )
	{

		if (_DEBUG_)
			std::cout<<"rot about y-axis with angel: "<< angle<<std::endl;

		lcl_homo_mat.M(0,0) = cos(angle);		lcl_homo_mat.M(0,1) = 0;			lcl_homo_mat.M(0,2) = sin(angle);
		lcl_homo_mat.M(1,0) = 0;				lcl_homo_mat.M(1,1) = 1;			lcl_homo_mat.M(1,2) = 0;
		lcl_homo_mat.M(2,0) = -1*sin(angle);	lcl_homo_mat.M(2,1) = 0;			lcl_homo_mat.M(2,2) = cos(angle);
	}

	if ( rot_axis[0] == 0 && rot_axis[1] == 0 && (rot_axis[2] == 1 || rot_axis[2] == -1) )
	{
		if (_DEBUG_)
			std::cout<<"rot about z-axis with angel: "<< angle<<std::endl;

		lcl_homo_mat.M(0,0) = cos(angle);	lcl_homo_mat.M(0,1) = -1*sin(angle);	lcl_homo_mat.M(0,2) = 0;
		lcl_homo_mat.M(1,0) = sin(angle);	lcl_homo_mat.M(1,1) = cos(angle);	lcl_homo_mat.M(1,2) = 0;
		lcl_homo_mat.M(2,0) = 0;			lcl_homo_mat.M(2,1) = 0;			lcl_homo_mat.M(2,2) = 1;
	}
}

//todo: here hard-coded rot_axis, set proper way
void Kinematic_calculations::forward_kinematics(const KDL::JntArray& jnt_angels)
{
	KDL::Frame fk_mat = KDL::Frame::Identity();
	std::vector<unsigned int> rot_axis{0,0,1};
	unsigned int cnt = 0;

	//Print on consol about joint angles
	ROS_INFO("FK with Joint angles:");
	for (uint8_t i = 0; i < jnts_name.size(); ++i)
	{
		ROS_INFO_STREAM(jnts_name.at(i) << " : " << jnt_angels(i));
	}

	//Find transformation between base_link & root frame if different
	//----------------------------------------------------------------------
	if (this->root_frame != this->base_link)
	{
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try
		{	//world arm_1_link
			listener.waitForTransform(this->root_frame,this->base_link, ros::Time(0), ros::Duration(5.0));	//link2,3 = 3.0,link4,5 = 4.0, link6,7 = 5.0
			listener.lookupTransform(this->root_frame, this->base_link, ros::Time(0), transform);

			geometry_msgs::TransformStamped msg;
			tf::transformStampedTFToMsg(transform,  msg);
			fk_mat = tf2::transformToKDL(msg);

		}
		catch (tf::TransformException& ex)
		{
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();
		 }

	}

	// Compute fk from chain base_link to chain tip link, think with dof
	//-------------------------------------------------------------------------
	for (uint16_t i = 0; i < this->segments; ++i)
	{
		//if revolute joint than multiply with joint angles
		if ( jnts.at(i).getType() == 0 )
		{

			KDL::Frame lcl_homo_mat = KDL::Frame::Identity();
			createRoatationMatrix( jnt_angels(cnt), rot_axis, lcl_homo_mat );
			jnt_homo_mat[i] = jnt_homo_mat[i] * lcl_homo_mat;
			cnt++;
		}

		fk_mat = fk_mat * jnt_homo_mat[i];
		jnt_fk_mat.push_back(fk_mat);
	}

	this->fk_mat = fk_mat;

}

//todo: can not find fk from root frame
void Kinematic_calculations::kdl_forward_kinematics(const KDL::JntArray& jnt_angels)
{
	using namespace KDL;
	KDL::Frame fk_mat = KDL::Frame::Identity();
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(this->chain);

	//Print on consol about joint angles
	ROS_INFO("KDL::FK with Joint angles:");
	for (uint8_t i = 0; i < jnts_name.size(); ++i)
	{
		ROS_INFO_STREAM(jnts_name.at(i) << " : " << jnt_angels(i));
	}

	//Find transformation between base_link & root frame if different
	//----------------------------------------------------------------------
	if (this->root_frame != this->base_link)
	{
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try
		{	//world arm_1_link
			listener.waitForTransform( root_frame, base_link, ros::Time(0), ros::Duration(5.0));	//link2,3 = 3.0,link4,5 = 4.0, link6,7 = 5.0
			listener.lookupTransform( root_frame, base_link, ros::Time(0), transform);

			geometry_msgs::TransformStamped msg;
			tf::transformStampedTFToMsg(transform,  msg);
			fk_mat = tf2::transformToKDL(msg);

		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("kdl_forward_kinematics ... Can not find transformation %s",ex.what());
			ros::Duration(1.0).sleep();
		}

	}


	// Compute fk from chain base_link to chain tip link, think with dof
	//-------------------------------------------------------------------------
	bool kinematic_status = fksolver.JntToCart(jnt_angels, fk_mat);

}


void Kinematic_calculations::compute_jacobian(const KDL::JntArray& jnt_angels)
{
	//todo: change dimension of matrix (means 7) accord to dof

	typedef Eigen::Matrix<double, 3, 1>       Cart3Vector;
	Cart3Vector p(0,0,0);	Cart3Vector z_0(0,0,1); 	Cart3Vector p_0(0,0,0);

	//Compute end-effector position by using forward kinematics
	//-----------------------------------------------------------
	forward_kinematics(jnt_angels);

	// dist from end-effector to base-link
	p(0) = fk_mat.p.x();	p(1) = fk_mat.p.y();	p(2) = fk_mat.p.z();

	if (_DEBUG_)
		{
				std::cout<<"\033[94m"<<"End-effector pose vector relative to base link " <<"\033[0m"<<std::endl;
				std::cout<<"\033[94m" << p  <<"\033[0m" <<std::endl;
		}


	//compute linear and angular velocity at each joint
	for(uint16_t i = 0; i < this->segments; ++i)
	{
		Cart3Vector J_v(0,0,0), J_o(0,0,0);
		if (i == 0)
		{
			if ( this-> jnts.at(i).getType() == 0 )	//revolute joint
			{
				J_v = z_0.cross( p );
				J_o = z_0;
			}

			if ( this-> jnts.at(i).getType() == 8 )	//prismatic joint
			{
				J_v = z_0;
				J_o = p_0;
			}
			else
			{
				;
			}
		}
		else
		{
			Cart3Vector z_i(0,0,0);	Cart3Vector p_i(0,0,0);

			//third col of rot matrix at each jnt with respect to base link
			z_i(0) = jnt_fk_mat.at(i).M(0,2);		z_i(1) = jnt_fk_mat.at(i).M(1,2);		z_i(2) = jnt_fk_mat.at(i).M(2,2);
			p_i(0) = jnt_fk_mat.at(i).p(0);			p_i(1) = jnt_fk_mat.at(i).p(1);		p_i(2) = jnt_fk_mat.at(i).p(2);

			if ( this-> jnts.at(i).getType() == 0 )	//revolute joint
			{
				J_v = z_i.cross( p - p_i );
				J_o = z_i;
			}

			if ( this-> jnts.at(i).getType() == 8 )	//prismatic joint
			{
				J_v = z_i;
				J_o = p_0;
			}
			else
			{
				;
			}
		}

		JacobianMatrix(0,i) = J_v(0);	JacobianMatrix(1,i) = J_v(1);	JacobianMatrix(2,i) = J_v(2);
		JacobianMatrix(3,i) = J_o(0);	JacobianMatrix(4,i) = J_o(1);	JacobianMatrix(5,i) = J_o(2);
	}
}

void Kinematic_calculations::kdl_compute_jacobian(const KDL::JntArray& jnt_angels)
{

	KDL::ChainJntToJacSolver jacobi_solver = KDL::ChainJntToJacSolver(this->chain);

	//Print on consol about joint angles
	ROS_INFO("KDL::Jacobian with Joint angles:");
	for (uint8_t i = 0; i < jnts_name.size(); ++i)
	{
		ROS_INFO_STREAM(jnts_name.at(i) << " : " << jnt_angels(i));
	}

	// Create object of KDL::Jacobian, initialize all elements with zeros
	KDL::Jacobian j_kdl = KDL::Jacobian(this->dof);
	j_kdl.data.Constant(0.0);

	// FK solver
	int jacobian_state = jacobi_solver.JntToJac(jnt_angels, j_kdl);

	// Conversion from KDL::Jacobian to Eigen::MatrixXd, Here Jacobian Matrix has 6 rows, columns are same as dof
	for (unsigned int i = 0; i < 6; ++i)
	{
		for (unsigned int j = 0; j < this->dof; ++j)
		{
			JacobianMatrix(i,j) = j_kdl(i,j);
		}
	}

}

void Kinematic_calculations::set_joint_names(std::vector<std::string>& jnt_names_param)
{
	jnts_name.clear();
	jnts_name = jnt_names_param;
}

void Kinematic_calculations::set_joint_limits(const std::string& name_of_limit, const std::vector<double>& limit_vec)
{
	if (name_of_limit == "jnt_pose_min_limit" || name_of_limit == "joint_pose_min_limit" || name_of_limit == "pose_min_limit")
	{
		jnt_pos_min_limit = limit_vec ;
	}

	else if (name_of_limit == "jnt_pose_max_limit" || name_of_limit == "joint_pose_max_limit" || name_of_limit == "pose_max_limit")
	{
		jnt_pos_max_limit = limit_vec ;
	}

	else if (name_of_limit == "jnt_vel_limit" || name_of_limit == "joint_velocity_limit" || name_of_limit == "vel_limit")
	{
		jnt_vel_limit = limit_vec;
	}
	else
	{
		ROS_WARN("Set_joint_limits ... Can not set joint limit with passed limit name ");
	}
}

void Kinematic_calculations::set_min_joint_position_limits( const std::vector<double>& limit_vec)
{
	jnt_pos_min_limit = limit_vec;
}

void Kinematic_calculations::set_max_joint_position_limits( const std::vector<double>& limit_vec)
{
	jnt_pos_max_limit = limit_vec;
}

void Kinematic_calculations::set_joint_velocity_limits( const std::vector<double>& limit_vec)
{
	jnt_vel_limit = limit_vec;
}


KDL::Frame Kinematic_calculations::get_forward_kinematics(void)
{
	return this->fk_mat;
}

void Kinematic_calculations::get_forward_kinematics(KDL::Frame& fk_mat)
{
	fk_mat = this->fk_mat;
}


Eigen::MatrixXd Kinematic_calculations::get_jacobian(const KDL::JntArray& jnt_angles)
{
	this->compute_jacobian(jnt_angles);

	return this->JacobianMatrix;
}

void Kinematic_calculations::get_jacobian(const KDL::JntArray& jnt_angles, Eigen::MatrixXd& j_mat)
{
	this->compute_jacobian(jnt_angles);
	j_mat = this->JacobianMatrix;
}

void Kinematic_calculations::get_joint_limits(const std::string& name_of_limit, std::vector<double>& limit_vec)
{
	if (name_of_limit == "jnt_pose_min_limit" || name_of_limit == "joint_pose_min_limit" || name_of_limit == "pose_min_limit")
	{
		limit_vec = jnt_pos_min_limit;
	}

	else if (name_of_limit == "jnt_pose_max_limit" || name_of_limit == "joint_pose_max_limit" || name_of_limit == "pose_max_limit")
	{
		limit_vec = jnt_pos_max_limit;
	}

	else if (name_of_limit == "jnt_vel_limit" || name_of_limit == "joint_velocity_limit" || name_of_limit == "vel_limit")
	{
		limit_vec = jnt_vel_limit;
	}
	else
	{
		ROS_WARN("get_joint_limits ... Can not get joint limit with passed limit name");
	}
}

void Kinematic_calculations::get_min_joint_position_limits( std::vector<double>& limit_vec)
{
	limit_vec = jnt_pos_min_limit;
}

void Kinematic_calculations::get_max_joint_position_limits( std::vector<double>& limit_vec)
{
	limit_vec = jnt_pos_max_limit;
}

void Kinematic_calculations::get_joint_velocity_limits( std::vector<double>& limit_vec)
{
	limit_vec = jnt_vel_limit;
}

void Kinematic_calculations::convert_kdl_frame_to_Eigen_matrix(const KDL::Frame& kdl_frame, Eigen::Matrix4d& egn_mat)
{
	egn_mat.Constant(0.0);
	KDL::Rotation rot_mat = kdl_frame.M;
	KDL::Vector pos_mat = kdl_frame.p;

	egn_mat(0,0) = rot_mat(0,0);	egn_mat(0,1) = rot_mat(0,1);	egn_mat(0,2) = rot_mat(0,2);	egn_mat(0,3) = pos_mat(0);
	egn_mat(1,0) = rot_mat(1,0);	egn_mat(1,1) = rot_mat(1,1);	egn_mat(1,2) = rot_mat(1,2);	egn_mat(1,3) = pos_mat(1);
	egn_mat(2,0) = rot_mat(2,0);	egn_mat(2,1) = rot_mat(2,1);	egn_mat(2,2) = rot_mat(2,2);	egn_mat(2,3) = pos_mat(2);
	egn_mat(3,0) = 0;				egn_mat(3,1) = 0;				egn_mat(3,2) = 0;				egn_mat(3,3) = 1;

}

void Kinematic_calculations::print_data_memebers(void)
{
		std::cout<<"\n \n";
		std::cout<<"\033[95m"<<"############################################ "	<<"\033[0m"<<std::endl;
		std::cout<<"\033[95m"<<" Kinematic calculation "	<<"\033[0m"<<std::endl;
		std::cout<<"\033[95m"<<"############################################ "	<<"\033[0m"<<std::endl;

		std::cout<<"\033[36;1m"<<"base_link_: "			<< this->base_link 		<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"tip_link_: "			<< this->tip_link 		<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Root frame: "			<< this->root_frame 	<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"DOF: "				<< this->dof 			<<"\033[36;0m"<<std::endl;
		std::cout<<"\033[36;1m"<<"Number_segments: "	<< this-> segments 		<<"\033[36;0m"<<std::endl;

		// Print joint information
		std::cout<<"\033[95m"<<"Joints: "	<<"\033[36;0m"<<std::endl;
		for (std::vector<KDL::Joint>::const_iterator it = this->jnts.begin(); it!= jnts.end(); ++it)
		{
			std::cout<<"\033[30;1m"	<<"\t joint axis: "	<< it->JointAxis().x() << " "<< it->JointAxis().y() << " "<< it->JointAxis().z()
									<<", jnt name: "	<< it->getName()
									<<", jnt type: "	<< it->getType()
									<<", jnt name: "	<< it->getTypeName()
									<<" jnt origin: "	<< it->JointOrigin().x()<< " "<< it->JointOrigin().y() << " "<< it->JointOrigin().z()

					<<"\033[30;0m"<<std::endl;
		}

		// Print frame name
		std::cout<<"\033[95m"<<"Frame name: "	<<"\033[36;0m"<<std::endl;
		for (uint16_t i = 0; i < frame_names.size(); ++i)
		{
			std::cout<<"\033[70;1m" << frame_names.at(i) << " , ";
		}
		std::cout <<"\033[70;0m"	<<std::endl;

		// Print joint name
		std::cout<<"\033[95m"<<"Joints name: "	<<"\033[36;0m"<<std::endl;
		for (uint16_t i = 0; i < jnts_name.size(); ++i)
		{
			std::cout<<"\033[70;1m" << jnts_name.at(i) <<" , ";
		}
		std::cout <<"\033[70;0m"	<<std::endl;


		// Print joint position, velocity limit
		std::cout<<"\033[95m"<<"Min joint position limit: "	<<"\033[36;0m"<<std::endl;
		for (auto it = jnt_pos_min_limit.begin(); it != jnt_pos_min_limit.end(); ++it)
		{
			std::cout<<"\033[0;33m" << *it << " , ";
		}
		std::cout <<"\033[70;0m"	<<std::endl;

		std::cout<<"\033[95m"<<"Max joint position limit: "	<<"\033[36;0m"<<std::endl;
		for (auto it = jnt_pos_max_limit.begin(); it != jnt_pos_max_limit.end(); ++it)
		{
			std::cout<<"\033[0;31m" << *it << " , ";
		}
		std::cout <<"\033[70;0m"	<<std::endl;

		std::cout<<"\033[95m"<<"Joint velocity limit: "	<<"\033[36;0m"<<std::endl;
		for (auto it = jnt_vel_limit.begin(); it != jnt_vel_limit.end(); ++it)
		{
			std::cout<<"\033[0;32m" << *it << " , ";
		}
		std::cout <<"\033[70;0m"	<<std::endl;

		// Print Homogenous matrix
		std::cout<<"\033[95m"<<"Homo matrix: "	<<"\033[36;0m"<<std::endl;
		for (std::vector<KDL::Frame>::const_iterator it = this->jnt_homo_mat.begin(); it!= jnt_homo_mat.end(); ++it)
		{
			KDL::Rotation rot_mat = it->M;
			KDL::Vector pos_mat = it->p;

			//for (unsigned int i = 0; i < it->)
			std::cout<<"\033[32;1m"	<<"\t  "	<<	" rxx "<< rot_mat(0,0) <<	" rxy "<< rot_mat(0,1) <<	" rxz "<< rot_mat(0,2) << "\n"
									<<"\t  "	<<	" ryx "<< rot_mat(1,0) <<	" ryy "<< rot_mat(1,1) <<	" ryz "<< rot_mat(1,2) << "\n"
									<<"\t  "	<<	" rzx "<< rot_mat(2,0) <<	" rzy "<< rot_mat(2,1) <<	" rzz "<< rot_mat(2,2) << "\n"
									<<"\t  "	<<	" px "<< pos_mat.x() <<	" py "<< pos_mat.y() <<	" pz "<< pos_mat.z() << "\n"
					<<"\033[32;0m"<<std::endl;
		}

		std::cout<<"\033[95m"<<"############################################ "	<<"\033[0m"<<std::endl;
		std::cout<<"\033[95m"<<"############################################ "	<<"\033[0m"<<std::endl;
		std::cout<<"\n \n";
}

void Kinematic_calculations::print_fk_and_jacobian_matrix()
{
	KDL::JntArray jnt_angles = KDL::JntArray(dof);
	jnt_angles(0) = 1.57;	jnt_angles(1) = 1.57;	jnt_angles(2) = 1.57;	jnt_angles(3) = 1.57;
	//forward_kinematics(jnt_angles);
	compute_jacobian(jnt_angles);

/*
	KDL::Rotation rot_mat = this->fk_mat.M;
	KDL::Vector pos_mat = this->fk_mat.p;

	std::cout<<"\033[95m"<<"FK matrix: "	<<"\033[36;0m"<<std::endl;
	std::cout<<"\033[32;1m"	<<	" rxx "<< rot_mat(0,0) 	<<	" rxy "<< rot_mat(0,1) 	<<	" rxz "<< rot_mat(0,2)	<< "\n"
							<<	" ryx "<< rot_mat(1,0) 	<<	" ryy "<< rot_mat(1,1) 	<<	" ryz "<< rot_mat(1,2)	<< "\n"
							<<	" rzx "<< rot_mat(2,0) 	<<	" rzy "<< rot_mat(2,1) 	<<	" rzz "<< rot_mat(2,2)	<< "\n"
							<<	" px "<< pos_mat.x() 	<<	" py "<< pos_mat.y() 	<<	" pz "<< pos_mat.z()
			<<"\033[32;0m"<<std::endl;
*/
	Eigen::Matrix4d egn_mat;
	convert_kdl_frame_to_Eigen_matrix(fk_mat, egn_mat);

	std::cout<<"\033[95m"<<"Forward_kinematics in the form of Eigen Matrix: \n"	<<"\033[36;0m" << egn_mat <<std::endl;
	std::cout<<"\033[95m"<<"Jacobian Matrix: \n"	<<"\033[36;0m" << JacobianMatrix <<std::endl;
//	ROS_INFO_STREAM(" Forward_kinematics Eigen Matrix: \n " << egn_mat);
//	ROS_INFO_STREAM(" Jacobian Matrix: \n " << JacobianMatrix);

}

void Kinematic_calculations::print_kdl_fk_and_jacobian_matrix()
{
	KDL::JntArray jnt_angles = KDL::JntArray(dof);
	jnt_angles(0) = 1.57;	jnt_angles(1) = 1.57;	jnt_angles(2) = 1.57;	jnt_angles(3) = 1.57;

	kdl_forward_kinematics(jnt_angles);
	kdl_compute_jacobian(jnt_angles);


	Eigen::Matrix4d egn_mat;
	convert_kdl_frame_to_Eigen_matrix(fk_mat, egn_mat);

	std::cout<<"\033[95m"<<"Forward_kinematics in the form of Eigen Matrix: \n"	<<"\033[36;0m" << egn_mat <<std::endl;
	std::cout<<"\033[95m"<<"Jacobian Matrix: \n"	<<"\033[36;0m" << JacobianMatrix <<std::endl;

}
