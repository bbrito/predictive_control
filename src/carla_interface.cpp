#include "ros/ros.h"
#include <ros/console.h>
#include <nav_msgs/Path.h>



void waypointCallback(nav_msgs::Path msg)
{
  double ysqr, t3, t4;
  std::vector<double> x, y, theta;
  int j = 0;
  for(int i = 0; i < msg.poses.size(); i+=2)
  {
    ysqr = msg.poses[i].pose.orientation.y * msg.poses[i].pose.orientation.y;
    t3 = +2.0 * (msg.poses[i].pose.orientation.w * msg.poses[i].pose.orientation.z
                            + msg.poses[i].pose.orientation.x *msg.poses[i].pose.orientation.y);
    t4 = +1.0 - 2.0 * (ysqr + msg.poses[i].pose.orientation.z * msg.poses[i].pose.orientation.z);

    theta.push_back(std::atan2(t3, t4));
    x.push_back(msg.poses[i].pose.position.x); // for shifting the current coordinates to the center of mass
    y.push_back(msg.poses[i].pose.position.y);
    j++;
  }
  std::cout<<"x"<<std::endl;
  for(int i = 0; i < j; i++)
  {
    std::cout<<x[i]<<", ";
  }
  std::cout<<"y"<<std::endl;
  for(int i = 0; i < j; i++)
  {
    std::cout<<y[i]<<", ";
  }
  std::cout<<"theta"<<std::endl;
  for(int i = 0; i < j; i++)
  {
    std::cout<<theta[i]<<", ";
  }
  
}


int main(int argc, char* argv[]) {
  // Initialize ROS
  ros::init(argc, argv, "waypoint_for_lmpcc");

  ros::NodeHandle nh("~");

  ros::Subscriber sub1 = nh.subscribe<nav_msgs::Path>(
      "/carla/ego_vehicle/waypoints", 1, waypointCallback);
  ros::spin();
  return 0;

}