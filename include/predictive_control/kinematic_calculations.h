
#ifndef PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
#define PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_

// ros includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>

// eigen includes
#include <Eigen/Core>
#include <Eigen/LU>				//inverse of matrix
#include <boost/shared_ptr.hpp>

// kdl includes
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// c++ includes
#include <iostream>
#include <map>
#include <string>
//#include <iomanip> std::setprecision(5)

// acado includes
#include <acado_optimal_control.hpp>



#define _DEBUG_  false

class Kinematic_calculations
	{
	private:

		unsigned int segments;
		unsigned int dof;

		std::string base_link;
		std::string tip_link;
		std::string root_frame;

		std::vector<KDL::Vector> jnt_rot_axis;
		//Eigen::MatrixXi axis_of_rotation;	//todo: replace kdl::vector with Eigen vector

		KDL::Chain	chain;
		std::vector<KDL::Frame>	frames;
		std::vector<KDL::Joint>	jnts;
		std::vector<KDL::Frame>	jnt_homo_mat;	//homo matrix of each frame with prevoius joint

		std::vector<double> jnt_pos_min_limit;
		std::vector<double> jnt_pos_max_limit;
		std::vector<double> jnt_vel_limit;
		std::vector<std::string> frame_names;
		std::vector<std::string> jnts_name;

		std::vector<KDL::Frame> jnt_fk_mat;	//ff_mat
		KDL::Frame fk_mat;
		Eigen::Matrix<double, 6, 7> JacobianMatrix;	//Jacobian Matrix	todo: change 7 with dof


		void clear_data_member();
		void convert_kdl_frame_to_Eigen_matrix(const KDL::Frame& kdl_frame, Eigen::Matrix4d& egn_mat);
		//void convert_kdl_vec_to_Eigen_vec(const KDL::Vector& kdl_vec, Eigen::VectorXd& egn_mat);

		void create_transformation_matrix(const uint16_t& segment_number, const double& roll,const double& pitch, const double& yaw);
		void createRoatationMatrix(const double& angle, const std::vector<unsigned int>& rot_axis, KDL::Frame& lcl_homo_mat);

		// Computation of forward kinematic, jacobian matrix
		void forward_kinematics(const KDL::JntArray& jnt_angels);
		void compute_jacobian(const KDL::JntArray& jnt_angels);

		// Computation of forward kinematic, jacobian matrix using kdl library
		void kdl_forward_kinematics(const KDL::JntArray& jnt_angels);
		void kdl_compute_jacobian(const KDL::JntArray& jnt_angels);


		//void compute_interia_matrix();
		//void compute_mass_matrix();
	public:

		//Kinematics(const std::string rbt_description = "/robot_description", const std::string& base_link="arm_base_link", const std::string& tip_link="arm_7_link", const std::string& root_frame="world");

		Kinematic_calculations();
		~Kinematic_calculations();

		bool initialize(const std::string rbt_description_param = "/robot_description", const std::string& base_link_param="arm_base_link", const std::string& tip_link_param="arm_7_link", const std::string& root_frame_param="world");

		// Calculate inverse of jacobian by using singular value decomposition (SVD), direct method
		void calculate_inverse_jacobian_bySVD( const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv );
		void calculate_inverse_jacobian_byDirect( const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& jacobianInv );

		// Get FK matrix
		void get_forward_kinematics(KDL::Frame& fk_mat);
		void get_forward_kinematics(Eigen::MatrixXd& fk_mat);
		KDL::Frame get_forward_kinematics(void);

		// Get Jacobian matrix
		void get_jacobian(const KDL::JntArray& jnt_angles, Eigen::MatrixXd& j_mat);
		void compute_and_get_jacobian(const KDL::JntArray& jnt_angles, Eigen::MatrixXd& j_mat);
		Eigen::MatrixXd get_jacobian(const KDL::JntArray& jnt_angles);

		// Get position, velocity joint limits
		void get_joint_limits(const std::string& name_of_limit, std::vector<double>& limit_vec);
		void get_min_joint_position_limits( std::vector<double>& limit_vec);
		void get_max_joint_position_limits( std::vector<double>& limit_vec);
		void get_joint_velocity_limits( std::vector<double>& limit_vec );

		// Get joint and frame names
		void get_joints_name(std::vector<std::string>& jnts_name_param);
		void get_frame_names(std::vector<std::string>& frame_names_param);

		// Set joint and frame names
		void set_joint_names(std::vector<std::string>& jnt_names_param);
		void set_joint_limits(const std::string& name_of_limit, const std::vector<double>& limit_vec);

		// Set position, velocity joint limits
		void set_min_joint_position_limits( const std::vector<double>& limit_vec);
		void set_max_joint_position_limits( const std::vector<double>& limit_vec);
		void set_joint_velocity_limits( const std::vector<double>& limit_vec );

		// Debug function for kinematic calculation class
		void print_data_memebers(void);
		void print_fk_and_jacobian_matrix(const KDL::JntArray& jnt_angles);
		void print_kdl_fk_and_jacobian_matrix(const KDL::JntArray& jnt_angles);

	};

#endif //PREDICTIVE_CONTROL_KINEMATIC_CALCULATIONS_H_
