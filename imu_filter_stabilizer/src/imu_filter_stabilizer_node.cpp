#include "imu_filter_stabilizer/imu_filter_stabilizer.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ImuFilterStabilizer");

  ImuFilterStabilizer imu_filter;
  ros::spin();
  return 0;
}
