
#include <predictive_control/kinematic_calculations.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <geometry_msgs/TransformStamped.h>

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
/* homogenous transformation matrix
    ROS_WARN_STREAM(model.getJoint("arm_6_joint")->parent_to_joint_origin_transform.position.x<< "  " << model.getJoint("arm_6_joint")->parent_to_joint_origin_transform.position.y <<
    	    "  "<<model.getJoint("arm_6_joint")->parent_to_joint_origin_transform.position.z);

    ROS_WARN_STREAM(model.getJoint("arm_6_joint")->parent_to_joint_origin_transform.rotation.w<< "  " << model.getJoint("arm_6_joint")->parent_to_joint_origin_transform.rotation.x <<
    	    "  "<<model.getJoint("arm_6_joint")->parent_to_joint_origin_transform.rotation.y<<"  "<<model.getJoint("arm_6_joint")->parent_to_joint_origin_transform.rotation.z);

    ROS_WARN_STREAM(model.getJoint("arm_3_joint")->parent_to_joint_origin_transform.position.x<< "  " << model.getJoint("arm_3_joint")->parent_to_joint_origin_transform.position.y <<
    	    "  "<<model.getJoint("arm_3_joint")->parent_to_joint_origin_transform.position.z);

    ROS_WARN_STREAM(model.getJoint("arm_3_joint")->parent_to_joint_origin_transform.rotation.w<< "  " << model.getJoint("arm_3_joint")->parent_to_joint_origin_transform.rotation.x <<
    	    "  "<<model.getJoint("arm_3_joint")->parent_to_joint_origin_transform.rotation.y<<"  "<<model.getJoint("arm_3_joint")->parent_to_joint_origin_transform.rotation.z);
*/
    /*
    //model.getJoint("arm_6_joint")->limits->

    ROS_ERROR_STREAM(model.getLink("arm_6_link")->inertial->origin.position.x<< model.getLink("arm_6_link")->inertial->origin.position.y<< model.getLink("arm_6_link")->inertial->origin.position.z);
    ROS_ERROR_STREAM(model.getLink("arm_6_link")->visual->origin.position.x<< model.getLink("arm_6_link")->visual->origin.position.y<< model.getLink("arm_6_link")->visual->origin.position.z);
    ROS_ERROR_STREAM(model.getLink("arm_6_link")->collision->origin.position.x<< model.getLink("arm_6_link")->collision->origin.position.y<< model.getLink("arm_6_link")->collision->origin.position.z);
*/
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

    // Get link length
    link_length.clear();
    for (int i = 0u; i < segments ; ++i)
    {
    	geometry_msgs::Vector3 length_vec;

    	//ROS_WARN_STREAM(jnt_homo_mat.at(i+1).p.x() << "  "<< jnt_homo_mat.at(i+1).p.y()<< "  "<< jnt_homo_mat.at(i+1).p.z());
    	ROS_DEBUG_STREAM(jnt_homo_mat.at(i).p.x() << "  "<< jnt_homo_mat.at(i).p.y()<< "  "<< jnt_homo_mat.at(i).p.z());

    	//todo: check better way to length should not be negative, ceil, floor should be use
    	length_vec.x = std::abs(jnt_homo_mat.at(i).p.x());
    	length_vec.y = std::abs(jnt_homo_mat.at(i).p.y());
    	length_vec.z = std::abs(jnt_homo_mat.at(i).p.z());

    	ROS_DEBUG_STREAM(jnts_name.at(i).c_str() << " length: " << length_vec);
    	link_length.push_back(length_vec);
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
	transformation_matrix.clear();
	std::vector<unsigned int> rot_axis{0,0,1};
	unsigned int cnt = 0;
	jnt_fk_mat.clear();

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
			//jnt_homo_mat[i] = jnt_homo_mat[i] * lcl_homo_mat;
			transformation_matrix.push_back(jnt_homo_mat[i] * lcl_homo_mat);
			fk_mat = fk_mat * (jnt_homo_mat[i] * lcl_homo_mat);		// FK_Mat = Fk_Mat_old * Homo_Mat; Joint_1: FK_Mat = I * Homo_Mat;
			cnt++;
		}
		else if ( jnts.at(i).getType() == 8 )
		{
			fk_mat = fk_mat * jnt_homo_mat[i];
			transformation_matrix.push_back(jnt_homo_mat[i]);
		}
		jnt_fk_mat.push_back(fk_mat);
	}

	// Make sure every time, FK_mat initialize to I
	this->fk_mat = KDL::Frame::Identity();
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
	//bool kinematic_status = fksolver.JntToCart(jnt_angels, fk_mat);
	jntToCartSolver_pos_.reset(new KDL::ChainFkSolverPos_recursive(chain));
	int jacobian_state = jntToCartSolver_pos_->JntToCart(jnt_angels, fk_mat);

	this->fk_mat = fk_mat;

}


void Kinematic_calculations::compute_jacobian(const KDL::JntArray& jnt_angels)
{
	//todo: change dimension of matrix (means 7) accord to dof

	// Make sure every time, Jacobian Matrix should be first initialize with zero
	JacobianMatrix.Constant(0.0);

	typedef Eigen::Matrix<double, 3, 1>       Cart3Vector;
	Cart3Vector p(0,0,0);	Cart3Vector z_0(0,0,1); 	Cart3Vector p_0(0,0,0);

	//Compute end-effector position by using forward kinematics
	//-----------------------------------------------------------
	forward_kinematics(jnt_angels);

	// dist from end-effector to base-link
	p(0) = fk_mat.p.x();	p(1) = fk_mat.p.y();	p(2) = fk_mat.p.z();
	ROS_INFO("End-effector Position w.r.t to base_link: [%f , %f, %f]", fk_mat.p.x(), fk_mat.p.y(), fk_mat.p.z());

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

	// Cross check of computation of Jacobian matrix
	if (JacobianMatrix.isZero())
	{
		ROS_WARN("Computed of Jacobian is not correct, recompute again");
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
	// FK solver
	//int jacobian_state = jacobi_solver.JntToJac(jnt_angels, j_kdl);
	jntToJacSolver_.reset(new KDL::ChainJntToJacSolver(chain));
	unsigned int solver_state = jntToJacSolver_->JntToJac(jnt_angels, j_kdl );
	// Conversion from KDL::Jacobian to Eigen::MatrixXd, Here Jacobian Matrix has 6 rows, columns are same as dof
	for (unsigned int i = 0; i < 6; ++i)
	{
		for (unsigned int j = 0; j < this->dof; ++j)
		{
			JacobianMatrix(i,j) = j_kdl(i,j);
		}
	}

}

void Kinematic_calculations::calculate_inverse_jacobian_bySVD( const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv )
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

	//singular values lie on diagonal of matrix, easily invert
	Eigen::VectorXd singularValues = svd.singularValues();
	Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());

	for (uint32_t i = 0; i < singularValues.rows(); ++i)
	{
		double denominator = singularValues(i) * singularValues(i);
		//singularValuesInv(i) = 1.0 / singularValues(i);
		singularValuesInv(i) = (singularValues(i) < 1e-6) ? 0.0 : singularValues(i) / denominator;
	}

	Eigen::MatrixXd result  = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
	jacobianInv = result;
}


void Kinematic_calculations::calculate_inverse_jacobian_byDirect( const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv )
{
	Eigen::MatrixXd result;
    Eigen::MatrixXd jac_t = jacobian.transpose();
    uint32_t rows = jacobian.rows();
    uint32_t cols = jacobian.cols();

    if (cols >= rows)
    {
    	result = jac_t * (jacobian * jac_t).inverse();
    }
    else
    {
    	result = (jac_t * jacobian).inverse() * jac_t;
    }

    jacobianInv = result;
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
		jnt_pos_min_limit.clear();
		jnt_pos_min_limit = limit_vec ;
	}

	else if (name_of_limit == "jnt_pose_max_limit" || name_of_limit == "joint_pose_max_limit" || name_of_limit == "pose_max_limit")
	{
		jnt_pos_max_limit.clear();
		jnt_pos_max_limit = limit_vec ;
	}

	else if (name_of_limit == "jnt_vel_limit" || name_of_limit == "joint_velocity_limit" || name_of_limit == "vel_limit")
	{
		jnt_vel_limit.clear();
		jnt_vel_limit = limit_vec;
	}
	else
	{
		ROS_WARN("Set_joint_limits ... Can not set joint limit with passed limit name ");
	}
}

void Kinematic_calculations::set_min_joint_position_limits( const std::vector<double>& limit_vec)
{
	jnt_pos_min_limit.clear();
	jnt_pos_min_limit = limit_vec;
}

void Kinematic_calculations::set_max_joint_position_limits( const std::vector<double>& limit_vec)
{
	jnt_pos_max_limit.clear();
	jnt_pos_max_limit = limit_vec;
}

void Kinematic_calculations::set_joint_velocity_limits( const std::vector<double>& limit_vec)
{
	jnt_vel_limit.clear();
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

void Kinematic_calculations::compute_and_get_FK(const Eigen::VectorXd& jnt_angles, Eigen::Matrix4d& FK_Mat)
{
	KDL::JntArray jnt_angles_lcl;
	jnt_angles_lcl.data = jnt_angles;

	forward_kinematics(jnt_angles_lcl);

	FK_Mat(0,0) = fk_mat.M(0,0);	FK_Mat(0,1) = fk_mat.M(0,1);	FK_Mat(0,2) = fk_mat.M(0,2);	FK_Mat(0,3) = fk_mat.p(0);
	FK_Mat(1,0) = fk_mat.M(1,0);	FK_Mat(1,1) = fk_mat.M(1,1);	FK_Mat(1,2) = fk_mat.M(1,2);	FK_Mat(1,3) = fk_mat.p(1);
	FK_Mat(2,0) = fk_mat.M(2,0);	FK_Mat(2,1) = fk_mat.M(2,1);	FK_Mat(2,2) = fk_mat.M(2,2);	FK_Mat(2,3) = fk_mat.p(2);
	FK_Mat(3,0) = 0.0;				FK_Mat(3,1) = 0.0;				FK_Mat(3,2) = 0.0;				FK_Mat(3,3) = 1.0;

	//std::cout << FK_Mat << std::endl;
}

void Kinematic_calculations::compute_and_get_gripper_pose(const Eigen::VectorXd& jnt_angles, Eigen::VectorXd& gripper_pose)
{
	KDL::JntArray jnt_angles_lcl;
	jnt_angles_lcl.data = jnt_angles;

	forward_kinematics(jnt_angles_lcl);

	double r,p,y;
	fk_mat.M.GetRPY(r,p,y);

	gripper_pose(0) = fk_mat.p(0);		gripper_pose(1) = fk_mat.p(1);		gripper_pose(2) = fk_mat.p(2);
	gripper_pose(3) = r;				gripper_pose(4) = p;				gripper_pose(5) = y;
	//std::cout << FK_Mat << std::endl;
}

Eigen::MatrixXd Kinematic_calculations::get_jacobian(const KDL::JntArray& jnt_angles)
{
	return this->JacobianMatrix;
}

void Kinematic_calculations::get_jacobian(const KDL::JntArray& jnt_angles, Eigen::MatrixXd& j_mat)
{
	j_mat = this->JacobianMatrix;
}

void Kinematic_calculations::compute_and_get_jacobian(const KDL::JntArray& jnt_angles, Eigen::MatrixXd& j_mat)
{

	JacobianMatrix.Constant(0.0);
	compute_jacobian( jnt_angles );

	if (JacobianMatrix.isZero())
	{
		ROS_WARN("Computed of Jacobian is not correct");
	}

	j_mat = JacobianMatrix;
}

void Kinematic_calculations::compute_and_get_jacobian(const Eigen::VectorXd& jnt_angles, Eigen::MatrixXd& j_mat)
{
	KDL::JntArray jnt_angles_lcl;
	jnt_angles_lcl.data = jnt_angles;

	JacobianMatrix.Constant(0.0);
	compute_jacobian( jnt_angles_lcl );

	if (JacobianMatrix.isZero())
	{
		ROS_WARN("Computed of Jacobian is not correct");
	}

	j_mat = JacobianMatrix;
}


void Kinematic_calculations::compute_and_get_currrent_gripper_poseStamped(const Eigen::VectorXd& jnt_angles, geometry_msgs::PoseStamped& current_gripper_poseStamped)
{
	KDL::JntArray kdl_jnt_angles;
	kdl_jnt_angles.data = jnt_angles;

	// gives gripper position and rotation kdl frame matrix
	forward_kinematics(kdl_jnt_angles);

	//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	current_gripper_poseStamped.pose.orientation.w =  0.5 * sqrt(1.0 + fk_mat.M(0,0) + fk_mat.M(1,1) + fk_mat.M(2,2));
	current_gripper_poseStamped.pose.orientation.x = ( fk_mat.M(2,1) - fk_mat.M(1,2) ) / (4.0 * current_gripper_poseStamped.pose.orientation.w);
	current_gripper_poseStamped.pose.orientation.y = ( fk_mat.M(0,2) - fk_mat.M(2,0) ) / (4.0 * current_gripper_poseStamped.pose.orientation.w);
	current_gripper_poseStamped.pose.orientation.z = ( fk_mat.M(1,0) - fk_mat.M(0,1) ) / (4.0 * current_gripper_poseStamped.pose.orientation.w);

	// position of gripper
	current_gripper_poseStamped.pose.position.x = fk_mat.p(0);
	current_gripper_poseStamped.pose.position.y = fk_mat.p(1);
	current_gripper_poseStamped.pose.position.z = fk_mat.p(2);

	current_gripper_poseStamped.header.frame_id = root_frame;
	current_gripper_poseStamped.header.stamp = ros::Time::now();

	ros::Duration(0.01).sleep();
}

void Kinematic_calculations::compute_gripper_pose_and_jacobian(const std::vector<double>& jnt_position, geometry_msgs::PoseStamped& gripper_pose, Eigen::MatrixXd& jacobian_mat)
{
	KDL::JntArray kdl_jnt_angles = KDL::JntArray(jnt_position.size());

	for (int i=0u; i < jnt_position.size(); ++i)
	{
		kdl_jnt_angles(i) = jnt_position.at(i);
	}

	JacobianMatrix.Constant(0.0);
	compute_jacobian(kdl_jnt_angles);

	ros::Duration(0.01).sleep();

	//http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	gripper_pose.pose.orientation.w =  0.5 * sqrt(1.0 + fk_mat.M(0,0) + fk_mat.M(1,1) + fk_mat.M(2,2));
	gripper_pose.pose.orientation.x = ( fk_mat.M(2,1) - fk_mat.M(1,2) ) / (4.0 * gripper_pose.pose.orientation.w);
	gripper_pose.pose.orientation.y = ( fk_mat.M(0,2) - fk_mat.M(2,0) ) / (4.0 * gripper_pose.pose.orientation.w);
	gripper_pose.pose.orientation.z = ( fk_mat.M(1,0) - fk_mat.M(0,1) ) / (4.0 * gripper_pose.pose.orientation.w);

	// position of gripper
	gripper_pose.pose.position.x = fk_mat.p(0);
	gripper_pose.pose.position.y = fk_mat.p(1);
	gripper_pose.pose.position.z = fk_mat.p(2);

	if (JacobianMatrix.isZero())
	{
		ROS_WARN("Computed of Jacobian is not correct");
	}

	jacobian_mat = JacobianMatrix;
}

void Kinematic_calculations::compute_and_get_each_joint_pose(const std::vector<double>& jnt_position, std::vector<geometry_msgs::PoseStamped>& each_joint_stamped, std::vector<geometry_msgs::Vector3>& link_length)
{
	KDL::JntArray kdl_jnt_angles = KDL::JntArray(jnt_position.size());

	// convert std vector to kdl vector
	for (int i=0u; i < jnt_position.size(); ++i)
	{
		kdl_jnt_angles(i) = jnt_position.at(i);
	}

	// compute forward kinematics for joint transformation matrix relative to root frame
	forward_kinematics(kdl_jnt_angles);

	// fill poseStamed vector
	/*for ( int i=0u; i < jnt_homo_mat.size()& jnt_fk_mat.size(); ++i )
	{


		geometry_msgs::PoseStamped stamped;
		stamped.header.frame_id = root_frame;
		stamped.header.stamp = ros::Time().now();
		stamped.pose.position.x = jnt_fk_mat.at(i).p.x();
		stamped.pose.position.y = jnt_fk_mat.at(i).p.y();
		stamped.pose.position.z = jnt_fk_mat.at(i).p.z();
		jnt_homo_mat.at(i).M.GetQuaternion(stamped.pose.orientation.x, stamped.pose.orientation.y, stamped.pose.orientation.z, stamped.pose.orientation.w);

		//ROS_INFO_STREAM("compute_and_get_each_joint_pose: ... Joint pose relative to root frame \n" << stamped);

		each_joint_stamped.push_back(stamped);
	}*/

	each_joint_stamped.clear();

	for ( auto it=jnt_fk_mat.begin(); it != jnt_fk_mat.end(); ++it )
	{
		geometry_msgs::PoseStamped stamped;
		stamped.header.frame_id = root_frame;
		stamped.header.stamp = ros::Time().now();
		stamped.pose.position.x = it->p.x();
		stamped.pose.position.y = it->p.y();
		stamped.pose.position.z = it->p.z();
		it->M.GetQuaternion(stamped.pose.orientation.x, stamped.pose.orientation.y, stamped.pose.orientation.z, stamped.pose.orientation.w);

		//ROS_INFO_STREAM("compute_and_get_each_joint_pose: ... Joint pose relative to root frame \n" << stamped);

		each_joint_stamped.push_back(stamped);
	}

	link_length.clear();
	link_length = this->link_length;
}

void Kinematic_calculations::compute_and_get_each_joint_pose(const std::vector<double>& jnt_position, std::map<std::string, geometry_msgs::PoseStamped>& self_collsion_matrix)
{
	KDL::JntArray kdl_jnt_angles = KDL::JntArray(jnt_position.size());

	// convert std vector to kdl vector
	for (int i=0u; i < jnt_position.size(); ++i)
	{
		kdl_jnt_angles(i) = jnt_position.at(i);
	}

	// compute forward kinematics for joint transformation matrix relative to root frame
	forward_kinematics(kdl_jnt_angles);
	std::string key = "point_";
	int i=0u;
	for ( auto it=jnt_fk_mat.begin(), it1 = jnt_homo_mat.begin(); it != jnt_fk_mat.end() & it1 != jnt_homo_mat.end() ; ++it, ++it1, ++i)
	{
		geometry_msgs::PoseStamped stamped;
		stamped.header.frame_id = root_frame;
		stamped.header.stamp = ros::Time().now();
		stamped.pose.position.x = it->p.x();
		stamped.pose.position.y = it->p.y();
		stamped.pose.position.z = it->p.z();
		it->M.GetQuaternion(stamped.pose.orientation.x, stamped.pose.orientation.y, stamped.pose.orientation.z, stamped.pose.orientation.w);

		self_collsion_matrix[ key+std::toString(i) ] = stamped;

		if (it1->p.z() > 0.14)
		{
			ROS_WARN("Create new point for self collision avoidance");
			geometry_msgs::PoseStamped point_stamped = stamped;
			auto it_previous = it - 1;
			/*
			point_stamped.pose.position.x = it_previous->p.x() + (it1->p.x() * 0.5) ;
			point_stamped.pose.position.y = it_previous->p.y() + (it1->p.y() * 0.5);
			point_stamped.pose.position.z = it_previous->p.z() + (it1->p.z() * 0.5);*/

			KDL::Frame frame, new_point_frame;
			frame.M.Identity();
			frame.p.x(it_previous->p.x());
			frame.p.y(it_previous->p.y());
			frame.p.z(it_previous->p.z()*0.5);

			Eigen::Vector4d vec(it->p.x(), it->p.y(), it->p.z()*0.5, 1.0), new_vec;

			new_vec = fk_mat * vec.data();

			point_stamped.pose.position.x = new_vec(0);				point_stamped.pose.position.y = new_vec(1);
			point_stamped.pose.position.z = new_vec(2);

			i = i+1;
			self_collsion_matrix[ key+std::toString(i) ] = point_stamped;
			create_static_frame(point_stamped, key+std::toString(i));
		}
		//ROS_INFO_STREAM("compute_and_get_each_joint_pose: ... Joint pose relative to root frame \n" << stamped);
	}
}

bool Kinematic_calculations::create_static_frame(const geometry_msgs::PoseStamped& stamped, const std::string& frame_name)
{
	geometry_msgs::TransformStamped static_transformStamped;
	static_transformStamped.header.stamp = stamped.header.stamp;
	static_transformStamped.header.frame_id = stamped.header.frame_id;
	static_transformStamped.child_frame_id = frame_name;

	static_transformStamped.transform.translation.x = stamped.pose.position.x;
	static_transformStamped.transform.translation.y = stamped.pose.position.y;
	static_transformStamped.transform.translation.z = stamped.pose.position.z;
	static_transformStamped.transform.rotation = stamped.pose.orientation;

	static_broadcaster.sendTransform(static_transformStamped);
	ros::spinOnce();
	return true;
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

void Kinematic_calculations::get_joints_name(std::vector<std::string>& jnts_name_param)
{
	jnts_name_param = jnts_name;
}

void Kinematic_calculations::get_frame_names(std::vector<std::string>& frame_names_param)
{
	frame_names_param = frame_names;
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

void Kinematic_calculations::print_fk_and_jacobian_matrix(const KDL::JntArray& jnt_angles)
{
	/*
	KDL::JntArray jnt_angles = KDL::JntArray(dof);
	jnt_angles(0) = 1.57;	jnt_angles(1) = 1.57;	jnt_angles(2) = 1.57;	jnt_angles(3) = 1.57;*/
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

void Kinematic_calculations::print_kdl_fk_and_jacobian_matrix(const KDL::JntArray& jnt_angles)
{
/*
	KDL::JntArray jnt_angles = KDL::JntArray(dof);
	jnt_angles(0) = 1.57;	jnt_angles(1) = 1.57;	jnt_angles(2) = 1.57;	jnt_angles(3) = 1.57;
*/
	kdl_forward_kinematics(jnt_angles);
	kdl_compute_jacobian(jnt_angles);


	Eigen::Matrix4d egn_mat;
	convert_kdl_frame_to_Eigen_matrix(fk_mat, egn_mat);

	std::cout<<"\033[95m"<<"Forward_kinematics using KDL in the form of Eigen Matrix: \n"	<<"\033[36;0m" << egn_mat <<std::endl;
	std::cout<<"\033[95m"<<"KDL Jacobian Matrix: \n"	<<"\033[36;0m" << JacobianMatrix <<std::endl;

}
