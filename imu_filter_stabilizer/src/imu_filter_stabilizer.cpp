#include "imu_filter_stabilizer/imu_filter_stabilizer.h"
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Scalar.h>
#include <cmath>

ImuFilterStabilizer::ImuFilterStabilizer() {

    ros::NodeHandle n;
    ros::NodeHandle priv_n("~");

    imu_suscriber_ = n.subscribe("imu/data", 0,
                                 &ImuFilterStabilizer::imuCallback, this);
    imu_publisher_ = n.advertise<sensor_msgs::Imu>("imu/data_stabilized", 5);
    reset_srv_ = priv_n.advertiseService("reset",
                                         &ImuFilterStabilizer::resetCallback,
                                         this);

    priv_n.param("tol", tol_, 0.001);

    initialized_ = false;
    stabilized_ = false;

}

void ImuFilterStabilizer::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_raw) {

    tf::Quaternion current_orientation;
    tf::quaternionMsgToTF(imu_msg_raw->orientation, current_orientation);

    if (!initialized_) {
        last_quaternion_ = current_orientation;
        initialized_ = true;
        return;
    }

    sensor_msgs::Imu msg = *imu_msg_raw.get();

    if (stabilized_) {
        last_quaternion_ = current_orientation;
        tf::Quaternion new_orientation = current_orientation * ref_orientation_.inverse();
        tf::quaternionTFToMsg(new_orientation, msg.orientation);
        imu_publisher_.publish(msg);
        return;
    }

    //not stabilized, doing the math
    tfScalar angle_diff = current_orientation.angleShortestPath(last_quaternion_);
    if (fabs(angle_diff) < tol_) {
        //stabilized!
        stabilized_ = true;
        ref_orientation_ = current_orientation;
        ROS_INFO("IMU has stabilized with an angle difference of %f", angle_diff);
        return;
    }
    else {
        last_quaternion_ = current_orientation;
        stabilized_ = false;
    }

}

bool ImuFilterStabilizer::resetCallback(std_srvs::Empty::Request& , std_srvs::Empty::Response&) {
    initialized_ = false;
    stabilized_ = false;
    return true;
}
