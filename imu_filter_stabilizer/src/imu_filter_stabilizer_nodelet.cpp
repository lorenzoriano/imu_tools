#include "imu_filter_stabilizer/imu_filter_stabilizer_nodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(imu_filter_stabilizer,
                        ImuFilterStabilizerNodelet,
                        ImuFilterStabilizerNodelet, nodelet::Nodelet);

void ImuFilterStabilizerNodelet::onInit()
{
  NODELET_INFO("Initializing IMU Filter Nodelet");

  filter_ = new ImuFilterStabilizer;
}
