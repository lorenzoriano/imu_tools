#ifndef IMU_FILTER_STABILIZER_IMU_FILTER_H
#define IMU_FILTER_STABILIZER_IMU_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <tf/LinearMath/Quaternion.h>
#include <std_srvs/Empty.h>

class ImuFilterStabilizer {
public:
    ImuFilterStabilizer(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_raw);
    bool resetCallback(std_srvs::Empty::Request& , std_srvs::Empty::Response&);

    ros::Publisher imu_publisher_;
    ros::Subscriber imu_suscriber_;
    ros::ServiceServer reset_srv_;
    tf::Quaternion last_quaternion_;
    ros::Time last_time_;
    tf::Quaternion ref_orientation_;

    bool initialized_;
    bool stabilized_;
    double tol_;


};

#endif
