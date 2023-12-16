#include "ros/ros.h"
#include "hokuyo_filter/HokuyoFilterRos.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include <vector>
#include <utility>
#include <stdexcept>

int main(int argc, char **argv) {
    ros::init(argc, argv, "hokuyo_filter");
    HokuyoFilterRos hokuyoFilter;

    /*ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Subscriber for the fire_detection topic
    ros::Subscriber fireSub = nh.subscribe("fire_detection", 10, &HokuyoFilterRos::fireCallback, &hokuyoFilter);

    // Publisher for the min_pose topic
    ros::Publisher posePub = nh.advertise<geometry_msgs::Pose2D>("min_pose", 10);

    while (ros::ok()) {
        // Main loop content here (if any)
        ros::spinOnce();
        loop_rate.sleep();
    }*/
    ros::spin();
    return 0;
}
