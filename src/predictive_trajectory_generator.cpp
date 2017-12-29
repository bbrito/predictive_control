
//This file containts cost function intsert in to generated trajectory.

#include <predictive_control/predictive_trajectory_generator.h>

pd_frame_tracker::pd_frame_tracker()
{
  ;
}

pd_frame_tracker::~pd_frame_tracker()
{
  ;
}

bool pd_frame_tracker::initialize()
{
  return true;
}

// calculate quternion product
void pd_frame_tracker::calculateQuaternionProduct(const geometry_msgs::Quaternion& quat_1, const geometry_msgs::Quaternion& quat_2, geometry_msgs::Quaternion& quat_resultant)
{
  Eigen::Vector3d quat_1_lcl(quat_1.x, quat_1.y, quat_1.z);
  Eigen::Vector3d quat_2_lcl(quat_2.x, quat_2.y, quat_2.z);

  // cross product
  Eigen::Vector3d cross_prod = quat_1_lcl.cross(quat_2_lcl);

  // multiplication of quternion
  // Modelling and Control of Robot Manipulators by L. Sciavicco and B. Siciliano
  quat_resultant.w = ( (quat_1.w * quat_2.w) - (quat_1_lcl.dot(quat_2_lcl)) );
  quat_resultant.x = ( (quat_1.w * quat_2.x) + (quat_2.w * quat_1.x) + cross_prod(0) );
  quat_resultant.y = ( (quat_1.w * quat_2.y) + (quat_2.w * quat_1.y) + cross_prod(1) );
  quat_resultant.z = ( (quat_1.w * quat_2.z) + (quat_2.w * quat_1.z) + cross_prod(2) );

}

// calculate inverse of quternion
void pd_frame_tracker::calculateQuaternionInverse(const geometry_msgs::Quaternion& quat, geometry_msgs::Quaternion& quat_inv)
{
  // Modelling and Control of Robot Manipulators by L. Sciavicco and B. Siciliano
  quat_inv = geometry_msgs::Quaternion();
  quat_inv.w = quat.w;
  quat_inv.x = -quat.x;
  quat_inv.y = -quat.y;
  quat_inv.z = -quat.z;
}
