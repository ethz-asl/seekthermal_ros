#include <seekthermal_ros/seekthermal_ros.h>

using namespace seekthermal_ros;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seekthermal_ros_node");
  ros::NodeHandle nh("~");

  SeekthermalRos seekthermalRos(nh);

  ros::spin();
  return 0;
}
