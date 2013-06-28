#ifndef IMU_FILTER_STABILIZER_IMU_FILTER_NODELET_H
#define IMU_FILTER_STABILIZER_IMU_FILTER_NODELET_H

#include <nodelet/nodelet.h>

#include "imu_filter_stabilizer/imu_filter_stabilizer.h"

class ImuFilterStabilizerNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    ImuFilterStabilizer * filter_;  // FIXME: change to smart pointer
};

#endif
