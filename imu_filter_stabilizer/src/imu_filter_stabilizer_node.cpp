#include "imu_filter_stabilizer/imu_filter_stabilizer.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ImuFilterStabilizer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ImuFilterStabilizer imu_filter(nh, nh_private);
  ros::spin();
  return 0;
}
