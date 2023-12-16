#ifndef HOKUYO_FILTER_ROS_H__
#define HOKUYO_FILTER_ROS_H__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h> // for Pose2D
#include "std_msgs/Int8.h" // for Int32
#include "std_msgs/Float32.h"


class HokuyoFilterRos {
public:
    HokuyoFilterRos();
    ~HokuyoFilterRos();

    void scanCallback(const sensor_msgs::LaserScan& msg);
    void fireCallback(const std_msgs::Int8& fire_msg);
    void cleanLaser(std::vector<float> msg_ranges, float range_min, float range_max);
    
private:
    ros::NodeHandle node_;
    ros::Subscriber scanSubscriber_;
    ros::Publisher scanPublisher_;
    ros::Subscriber fireSub_;
    ros::Publisher posePub_;
    int SUB_ARRAY_SIZE = 140;

    std::string p_scan_topic_;
    std::string p_scan_filter_topic_;
    int p_scan_subscriber_queue_size_;
    float LASER_RANGE_MIN;
    float LASER_RANGE_MAX;
    int num_readings;
    std::vector<float> ranges;


    std::vector<float> getSubArray(const std::vector<float>& ranges, int fire_msg);
    std::pair<size_t, float> findMinElement(const std::vector<float>& array);
    float calculateAngle(size_t minIndex, int fire_msg);
};

#endif
